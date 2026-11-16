#include "webui.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "mdns.h"
#include "nvs.h"
#include "gps.h"
#include "logger.h"
#include "motion.h"
#include "power.h"
#include "sdkconfig.h"

static const char *TAG = "webui";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static bool           s_wifi_ready;
static volatile bool  s_active;
static volatile bool  s_stop_requested;
static volatile int64_t s_last_request_us;
static httpd_handle_t s_server;
static char           s_ssid[32];

// Home Wi-Fi (station) support: saved name/password are tried first, then the
// board falls back to its own hotspot.
#define HOSTNAME          "gpslogger"
#define STA_TIMEOUT_MS    15000
#define STA_MAX_RETRIES   3
#define BIT_CONNECTED     BIT0
#define BIT_FAILED        BIT1

typedef enum { MODE_OFF, MODE_CONNECTING, MODE_HOME, MODE_HOTSPOT } wifi_mode_kind_t;

static wifi_mode_kind_t   s_mode;
static EventGroupHandle_t s_events;
static volatile bool      s_sta_connecting;
static int                s_sta_retries;
static char               s_ip[16];
static char               s_last_error[96];

#define GPX_CHUNK_POINTS 32

// ---- helpers ----

static void touch(void)
{
    s_last_request_us = esp_timer_get_time();
}

// Ride names look like "20261007-153253.trk"; reject anything else so a
// request can never reach outside the log folder.
static bool valid_ride_name(const char *n)
{
    size_t len = strlen(n);
    if (len < 5 || len > 31 || strcmp(n + len - 4, ".trk") != 0) return false;
    for (size_t i = 0; i < len - 4; i++) {
        if (!((n[i] >= '0' && n[i] <= '9') || n[i] == '-')) return false;
    }
    return true;
}

static bool get_name_param(httpd_req_t *req, char *name, size_t size)
{
    char query[64];
    return httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, "name", name, size) == ESP_OK &&
           valid_ride_name(name);
}

// degrees*1e7 as a fixed-point string, e.g. -18649430 -> "-1.8649430"
static int fmt_e7(char *out, size_t size, int32_t v)
{
    const char *sign = v < 0 ? "-" : "";
    uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
    return snprintf(out, size, "%s%lu.%07lu", sign,
                    (unsigned long)(a / 10000000), (unsigned long)(a % 10000000));
}

static int fmt_time(char *out, size_t size, uint32_t t)
{
    time_t tt = t;
    struct tm tm;
    gmtime_r(&tt, &tm);
    return strftime(out, size, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

static bool open_ride(const char *name, FILE **f, long *points)
{
    char path[48];
    snprintf(path, sizeof(path), LOGGER_BASE_PATH "/%s", name);
    *f = fopen(path, "rb");
    if (!*f) return false;
    uint8_t hdr[8];
    if (fread(hdr, 1, 8, *f) != 8 || memcmp(hdr, LOGGER_FILE_MAGIC, 4) != 0 ||
        hdr[5] != sizeof(log_point_t)) {
        fclose(*f);
        return false;
    }
    fseek(*f, 0, SEEK_END);
    *points = (ftell(*f) - 8) / (long)sizeof(log_point_t);
    fseek(*f, 8, SEEK_SET);
    return true;
}

static bool load_home_wifi(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_str(h, "ssid", ssid, &ssid_len) == ESP_OK && ssid[0] &&
              nvs_get_str(h, "pass", pass, &pass_len) == ESP_OK;
    nvs_close(h);
    return ok;
}

static esp_err_t save_home_wifi(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    if (ssid) {
        err = nvs_set_str(h, "ssid", ssid);
        if (err == ESP_OK) err = nvs_set_str(h, "pass", pass);
    } else {
        nvs_erase_all(h);
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

// Decodes application/x-www-form-urlencoded text in place ('+' and %XX).
static void url_decode(char *s)
{
    char *o = s;
    for (; *s; s++) {
        if (*s == '+') {
            *o++ = ' ';
        } else if (*s == '%' && s[1] && s[2]) {
            char hex[3] = { s[1], s[2], 0 };
            *o++ = (char)strtol(hex, NULL, 16);
            s += 2;
        } else {
            *o++ = *s;
        }
    }
    *o = '\0';
}

// Strips leading/trailing spaces. Phone keyboards often add a trailing space
// after autocompleting a word, and "MyWiFi " is a different network to "MyWiFi".
static void trim(char *s)
{
    char *start = s;
    while (*start == ' ' || *start == '\t') start++;
    if (start != s) memmove(s, start, strlen(start) + 1);
    size_t n = strlen(s);
    while (n && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
}

// Copies a string into JSON-safe form (quotes, backslashes, control chars).
static void json_escape(char *out, size_t size, const char *in)
{
    size_t o = 0;
    for (; *in && o + 7 < size; in++) {
        unsigned char c = (unsigned char)*in;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c < 0x20) o += snprintf(out + o, size - o, "\\u%04x", c);
        else out[o++] = c;
    }
    out[o] = '\0';
}

// ---- HTTP handlers ----

static esp_err_t index_get(httpd_req_t *req)
{
    touch();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t status_get(httpd_req_t *req)
{
    touch();
    gps_fix_t f;
    logger_status_t ls;
    gps_get_fix(&f);
    logger_get_status(&ls);
    char lat[16], lon[16], buf[400];
    fmt_e7(lat, sizeof(lat), (int32_t)(f.lat * 1e7));
    fmt_e7(lon, sizeof(lon), (int32_t)(f.lon * 1e7));
    snprintf(buf, sizeof(buf),
             "{\"fw\":\"%s\",\"partition\":\"%s\",\"fix\":%s,\"sats\":%d,\"in_view\":%d,"
             "\"hdop\":%.1f,\"lat\":%s,\"lon\":%s,\"fs_used_kb\":%lu,\"fs_total_kb\":%lu,"
             "\"full\":%s,\"idle_off_min\":%d}",
             esp_app_get_description()->version, esp_ota_get_running_partition()->label,
             f.valid ? "true" : "false", f.sats, f.sats_in_view, f.hdop, lat, lon,
             (unsigned long)ls.fs_used_kb, (unsigned long)ls.fs_total_kb,
             ls.full ? "true" : "false", CONFIG_WIFI_IDLE_OFF_MIN);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t rides_get(httpd_req_t *req)
{
    touch();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "[");
    DIR *dir = opendir(LOGGER_BASE_PATH);
    bool first = true;
    struct dirent *e;
    while (dir && (e = readdir(dir)) != NULL) {
        if (!valid_ride_name(e->d_name)) continue;
        FILE *f;
        long n;
        if (!open_ride(e->d_name, &f, &n)) continue;
        log_point_t p0 = {0}, p1 = {0};
        if (n > 0) {
            fread(&p0, sizeof(p0), 1, f);
            fseek(f, 8 + (n - 1) * (long)sizeof(log_point_t), SEEK_SET);
            fread(&p1, sizeof(p1), 1, f);
        }
        fclose(f);
        char item[320];
        snprintf(item, sizeof(item),
                 "%s{\"name\":\"%s\",\"points\":%ld,\"start\":%lu,\"duration\":%lu}",
                 first ? "" : ",", e->d_name, n, (unsigned long)p0.time,
                 (unsigned long)(n > 0 ? p1.time - p0.time : 0));
        httpd_resp_sendstr_chunk(req, item);
        first = false;
    }
    if (dir) closedir(dir);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t gpx_get(httpd_req_t *req)
{
    touch();
    char name[32];
    if (!get_name_param(req, name, sizeof(name))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad ride name");
    }
    FILE *f;
    long n;
    if (!open_ride(name, &f, &n)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "ride not found");
    }

    char base[24], disp[64];
    strlcpy(base, name, sizeof(base));
    base[strlen(base) - 4] = '\0';  // drop ".trk"
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s.gpx\"", base);
    httpd_resp_set_type(req, "application/gpx+xml");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    char head[320];
    snprintf(head, sizeof(head),
             "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
             "<gpx version=\"1.1\" creator=\"ESP32 GPS logger\" "
             "xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
             "<trk><name>%s</name><type>cycling</type><trkseg>\n", base);
    httpd_resp_sendstr_chunk(req, head);

    log_point_t *pts = malloc(GPX_CHUNK_POINTS * sizeof(log_point_t));
    char *out = malloc(GPX_CHUNK_POINTS * 128);
    esp_err_t err = (pts && out) ? ESP_OK : ESP_ERR_NO_MEM;
    size_t got;
    while (err == ESP_OK &&
           (got = fread(pts, sizeof(log_point_t), GPX_CHUNK_POINTS, f)) > 0) {
        int len = 0;
        for (size_t i = 0; i < got; i++) {
            char lat[16], lon[16], t[24];
            fmt_e7(lat, sizeof(lat), pts[i].lat_e7);
            fmt_e7(lon, sizeof(lon), pts[i].lon_e7);
            fmt_time(t, sizeof(t), pts[i].time);
            len += snprintf(out + len, 128,
                            "<trkpt lat=\"%s\" lon=\"%s\"><ele>%d</ele><time>%s</time></trkpt>\n",
                            lat, lon, pts[i].ele_m, t);
        }
        err = httpd_resp_send_chunk(req, out, len);
        touch();
    }
    fclose(f);
    free(pts);
    free(out);
    if (err != ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
        return err;
    }
    httpd_resp_sendstr_chunk(req, "</trkseg></trk></gpx>\n");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t delete_post(httpd_req_t *req)
{
    touch();
    char name[32], path[48];
    if (!get_name_param(req, name, sizeof(name))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad ride name");
    }
    snprintf(path, sizeof(path), LOGGER_BASE_PATH "/%s", name);
    if (unlink(path) != 0) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "ride not found");
    }
    ESP_LOGI(TAG, "deleted %s", name);
    return httpd_resp_sendstr(req, "deleted");
}

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t ota_post(httpd_req_t *req)
{
    touch();
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part || req->content_len == 0 || req->content_len > part->size) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "firmware missing or too big");
    }
    esp_ota_handle_t ota;
    if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
    }
    ESP_LOGI(TAG, "receiving %d byte firmware into %s", req->content_len, part->label);

    char *buf = malloc(4096);
    int remaining = req->content_len;
    esp_err_t err = buf ? ESP_OK : ESP_ERR_NO_MEM;
    while (err == ESP_OK && remaining > 0) {
        int r = httpd_req_recv(req, buf, remaining < 4096 ? remaining : 4096);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) { err = ESP_FAIL; break; }
        err = esp_ota_write(ota, buf, r);
        remaining -= r;
        touch();
    }
    free(buf);

    if (err != ESP_OK) {
        esp_ota_abort(ota);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload failed");
    }
    if (esp_ota_end(ota) != ESP_OK) {  // checks the image is a valid ESP32-S3 app
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "not a valid firmware file (use build/gps_logger.bin)");
    }
    esp_ota_set_boot_partition(part);
    ESP_LOGI(TAG, "firmware OK, rebooting into %s", part->label);
    httpd_resp_sendstr(req, "ok, rebooting");
    xTaskCreate(restart_task, "restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t wifi_off_post(httpd_req_t *req)
{
    httpd_resp_sendstr(req, "Wi-Fi turning off");
    s_stop_requested = true;  // can't stop the server from inside its own handler
    return ESP_OK;
}

static esp_err_t wifi_get(httpd_req_t *req)
{
    touch();
    char ssid[33] = "", pass[65] = "", esc[80], err[110], buf[320];
    load_home_wifi(ssid, sizeof(ssid), pass, sizeof(pass));
    json_escape(esc, sizeof(esc), ssid);
    json_escape(err, sizeof(err), s_last_error);
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"%s\",\"ip\":\"%s\",\"saved_ssid\":\"%s\",\"last_error\":\"%s\"}",
             s_mode == MODE_HOME ? "home" : "hotspot", s_ip, esc, err);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t wifi_post(httpd_req_t *req)
{
    touch();
    // URL-encoding can triple the length of a name or password with symbols in it
    char body[600], ssid[100], pass[200];
    if (req->content_len >= (int)sizeof(body)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "network name or password too long");
    }
    int len = req->content_len;
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no data");
        got += r;
    }
    body[got] = '\0';
    ESP_LOGI(TAG, "save Wi-Fi request: %d bytes", got);
    if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "network name missing");
    }
    if (httpd_query_key_value(body, "pass", pass, sizeof(pass)) != ESP_OK) {
        pass[0] = '\0';  // open network
    }
    url_decode(ssid);
    url_decode(pass);
    trim(ssid);
    if (!ssid[0] || strlen(ssid) > 32) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "network name must be 1-32 characters");
    }
    if (pass[0] && (strlen(pass) < 8 || strlen(pass) > 64)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "password must be 8-64 characters (or empty for an open network)");
    }
    esp_err_t err = save_home_wifi(ssid, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving Wi-Fi failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "could not save to flash");
    }
    s_last_error[0] = '\0';
    ESP_LOGI(TAG, "saved home Wi-Fi '%s'", ssid);
    return httpd_resp_sendstr(req, "saved");
}

static esp_err_t wifi_forget_post(httpd_req_t *req)
{
    touch();
    save_home_wifi(NULL, NULL);
    ESP_LOGI(TAG, "home Wi-Fi forgotten");
    return httpd_resp_sendstr(req, "forgotten");
}

static esp_err_t power_get(httpd_req_t *req)
{
    touch();
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"sleep_s\":%lu,\"accel\":%s}",
             (unsigned long)power_get_sleep_s(), motion_present() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t power_post(httpd_req_t *req)
{
    touch();
    char body[64], val[16];
    int len = req->content_len < (int)sizeof(body) - 1 ? req->content_len : (int)sizeof(body) - 1;
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no data");
        got += r;
    }
    body[got] = '\0';
    if (httpd_query_key_value(body, "sleep_s", val, sizeof(val)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "sleep_s missing");
    }
    char *end;
    long v = strtol(val, &end, 10);
    if (end == val || *end || v < 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "sleep_s must be a whole number");
    }
    power_set_sleep_s((uint32_t)v);
    power_event("setting: sleep delay %lu s (from web page)", (unsigned long)power_get_sleep_s());
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)power_get_sleep_s());
    return httpd_resp_sendstr(req, buf);
}

static void send_file_chunks(httpd_req_t *req, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return;
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) httpd_resp_send_chunk(req, buf, n);
    fclose(f);
}

static esp_err_t events_get(httpd_req_t *req)
{
    touch();
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    send_file_chunks(req, POWER_EVENTS_OLD_PATH);   // oldest first
    send_file_chunks(req, POWER_EVENTS_PATH);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t events_clear_post(httpd_req_t *req)
{
    touch();
    unlink(POWER_EVENTS_OLD_PATH);
    unlink(POWER_EVENTS_PATH);
    return httpd_resp_sendstr(req, "cleared");
}

static esp_err_t scan_get(httpd_req_t *req)
{
    touch();
    wifi_scan_config_t sc = { .show_hidden = false };
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scan failed");
    }
    uint16_t n = 20;
    wifi_ap_record_t *recs = calloc(n, sizeof(wifi_ap_record_t));
    if (!recs) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    esp_wifi_scan_get_ap_records(&n, recs);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "[");
    int emitted = 0;
    for (int i = 0; i < n; i++) {
        bool dup = false;  // the same network often shows up on several access points
        for (int j = 0; j < i; j++) {
            if (strcmp((char *)recs[i].ssid, (char *)recs[j].ssid) == 0) dup = true;
        }
        if (dup || !recs[i].ssid[0]) continue;
        char esc[80], item[120];
        json_escape(esc, sizeof(esc), (char *)recs[i].ssid);
        snprintf(item, sizeof(item), "%s{\"ssid\":\"%s\",\"rssi\":%d}",
                 emitted++ ? "," : "", esc, recs[i].rssi);
        httpd_resp_sendstr_chunk(req, item);
    }
    free(recs);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_sendstr_chunk(req, NULL);
}

// ---- Wi-Fi + server lifecycle ----

static void start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 30;
    cfg.send_wait_timeout = 30;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "web server failed to start");
        return;
    }
    const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = index_get },
        { .uri = "/api/status",   .method = HTTP_GET,  .handler = status_get },
        { .uri = "/api/rides",    .method = HTTP_GET,  .handler = rides_get },
        { .uri = "/ride.gpx",     .method = HTTP_GET,  .handler = gpx_get },
        { .uri = "/api/delete",   .method = HTTP_POST, .handler = delete_post },
        { .uri = "/api/ota",      .method = HTTP_POST, .handler = ota_post },
        { .uri = "/api/wifi_off", .method = HTTP_POST, .handler = wifi_off_post },
        { .uri = "/api/wifi",     .method = HTTP_GET,  .handler = wifi_get },
        { .uri = "/api/wifi",     .method = HTTP_POST, .handler = wifi_post },
        { .uri = "/api/wifi_forget", .method = HTTP_POST, .handler = wifi_forget_post },
        { .uri = "/api/scan",     .method = HTTP_GET,  .handler = scan_get },
        { .uri = "/api/power",    .method = HTTP_GET,  .handler = power_get },
        { .uri = "/api/power",    .method = HTTP_POST, .handler = power_post },
        { .uri = "/api/events",   .method = HTTP_GET,  .handler = events_get },
        { .uri = "/api/events_clear", .method = HTTP_POST, .handler = events_clear_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *ev = data;
        ESP_LOGW(TAG, "home Wi-Fi disconnected, reason %d", ev->reason);
        if (s_sta_connecting) {
            if (ev->reason == WIFI_REASON_NO_AP_FOUND) {
                strlcpy(s_last_error, "network not found (check the name; 5 GHz-only networks "
                        "can't be used)", sizeof(s_last_error));
            } else if (ev->reason == WIFI_REASON_AUTH_FAIL ||
                       ev->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
                       ev->reason == WIFI_REASON_HANDSHAKE_TIMEOUT) {
                strlcpy(s_last_error, "wrong password", sizeof(s_last_error));
            } else {
                snprintf(s_last_error, sizeof(s_last_error), "couldn't connect (reason %d)",
                         ev->reason);
            }
            if (++s_sta_retries < STA_MAX_RETRIES) esp_wifi_connect();
            else xEventGroupSetBits(s_events, BIT_FAILED);
        } else if (s_mode == MODE_HOME) {
            ESP_LOGW(TAG, "lost home Wi-Fi, reconnecting");
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_events, BIT_CONNECTED);
    }
}

static void wifi_init_once(void)
{
    if (s_wifi_ready) return;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, HOSTNAME);  // shows as "gpslogger" in the router's device list
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    s_events = xEventGroupCreate();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(HOSTNAME);
        mdns_instance_name_set("GPS Logger");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
    s_wifi_ready = true;
}

static bool try_home_wifi(void)
{
    char ssid[33], pass[65];
    if (!load_home_wifi(ssid, sizeof(ssid), pass, sizeof(pass))) return false;
    trim(ssid);  // networks saved before the trim fix may end in a space

    wifi_config_t sta = { 0 };
    strlcpy((char *)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_LOGI(TAG, "connecting to home Wi-Fi '%s'...", ssid);
    xEventGroupClearBits(s_events, BIT_CONNECTED | BIT_FAILED);
    s_sta_retries = 0;
    s_sta_connecting = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_CONNECTED | BIT_FAILED, pdFALSE,
                                           pdFALSE, pdMS_TO_TICKS(STA_TIMEOUT_MS));
    s_sta_connecting = false;
    if (bits & BIT_CONNECTED) {
        s_last_error[0] = '\0';
        return true;
    }
    if (!s_last_error[0]) strlcpy(s_last_error, "no answer from the network", sizeof(s_last_error));

    ESP_LOGW(TAG, "couldn't join '%s', starting hotspot instead", ssid);
    esp_wifi_disconnect();
    esp_wifi_stop();
    return false;
}

static void start_hotspot(void)
{
    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, s_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_ssid);
    strlcpy((char *)ap.ap.password, CONFIG_WIFI_AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.channel = 6;
    ap.ap.max_connection = 2;
    ap.ap.authmode = strlen(CONFIG_WIFI_AP_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK
                                                          : WIFI_AUTH_OPEN;
    // AP+STA so the page can scan for your home network
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    strlcpy(s_ip, "192.168.4.1", sizeof(s_ip));
}

static void wifi_start(void)
{
    wifi_init_once();
    logger_set_paused(true);  // close the current ride so it can be downloaded
    s_mode = MODE_CONNECTING;

    if (try_home_wifi()) {
        s_mode = MODE_HOME;
        ESP_LOGI(TAG, "on home Wi-Fi: open http://%s.local or http://%s", HOSTNAME, s_ip);
    } else {
        start_hotspot();
        s_mode = MODE_HOTSPOT;
        ESP_LOGI(TAG, "hotspot on: join '%s' (password '%s'), open http://192.168.4.1",
                 s_ssid, CONFIG_WIFI_AP_PASSWORD);
    }
    start_server();
    touch();
    s_active = true;
}

static void wifi_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    s_mode = MODE_OFF;
    esp_wifi_disconnect();
    esp_wifi_stop();
    s_ip[0] = '\0';
    s_active = false;
    s_stop_requested = false;
    logger_set_paused(false);
    ESP_LOGI(TAG, "Wi-Fi off, logging resumed");
}

static void button_task(void *arg)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_WIFI_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    int held_ms = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(CONFIG_WIFI_BUTTON_GPIO) == 0) {
            held_ms += 50;
            if (held_ms == 2000) {
                if (s_active) wifi_stop();
                else wifi_start();
            }
        } else {
            held_ms = 0;
        }

        if (s_active && (s_stop_requested ||
            esp_timer_get_time() - s_last_request_us >
                CONFIG_WIFI_IDLE_OFF_MIN * 60 * 1000000LL)) {
            wifi_stop();
        }
    }
}

void webui_init(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ssid, sizeof(s_ssid), "GPS-Logger-%02X%02X", mac[4], mac[5]);
    xTaskCreate(button_task, "button", 6144, NULL, 5, NULL);
    ESP_LOGI(TAG, "hold BOOT (GPIO%d) for 2 s to turn Wi-Fi on/off", CONFIG_WIFI_BUTTON_GPIO);
}

bool webui_is_active(void)
{
    return s_active;
}

const char *webui_ssid(void)
{
    return s_ssid;
}

const char *webui_address(void)
{
    return s_ip;
}

bool webui_is_connecting(void)
{
    return s_mode == MODE_CONNECTING;
}

bool webui_on_home_wifi(void)
{
    return s_mode == MODE_HOME;
}
