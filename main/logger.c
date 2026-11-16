#include "logger.h"

#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"

static const char *TAG = "logger";

#define PARTITION_LABEL  "storage"
#define BUF_POINTS       64
#define MIN_FREE_BYTES   (16 * 1024)

static logger_status_t s_status;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static FILE       *s_file;
static log_point_t s_buf[BUF_POINTS];
static int         s_buf_len;
static int64_t     s_last_flush_us;
static bool        s_have_last;
static double      s_last_lat, s_last_lon;
static volatile bool s_paused;

// ---- pure helpers ----

static double haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0, rad = M_PI / 180.0;
    double dlat = (lat2 - lat1) * rad, dlon = (lon2 - lon1) * rad;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * rad) * cos(lat2 * rad) * sin(dlon / 2) * sin(dlon / 2);
    return 2 * R * asin(sqrt(a));
}

// Days since 1970-01-01 for a proleptic Gregorian date (no timezone involved).
static int64_t days_from_civil(int y, int m, int d)
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int64_t yoe = y - era * 400;
    int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static uint32_t fix_unix_time(const gps_fix_t *f)
{
    return (uint32_t)(days_from_civil(f->year, f->month, f->day) * 86400 +
                      f->hour * 3600 + f->minute * 60 + (int)f->second);
}

static bool fix_good_enough(const gps_fix_t *f)
{
    return f->valid && f->sats >= LOGGER_MIN_SATS && f->hdop <= LOGGER_MAX_HDOP &&
           f->year >= 2024;
}

// Moved far enough from the last logged point? Slow GPS drift while stopped
// can creep past 10 m, so short hops also need some real speed.
static bool should_log(double dist_m, float speed_kmh)
{
    return (dist_m >= LOGGER_MIN_DIST_M && speed_kmh >= 2.0f) ||
           dist_m >= 3 * LOGGER_MIN_DIST_M;
}

// ---- filesystem ----

static void update_fs_usage(void)
{
    size_t total = 0, used = 0;
    if (esp_littlefs_info(PARTITION_LABEL, &total, &used) == ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        s_status.fs_total_kb = total / 1024;
        s_status.fs_used_kb  = used / 1024;
        portEXIT_CRITICAL(&s_lock);
    }
}

static bool fs_has_room(void)
{
    size_t total = 0, used = 0;
    if (esp_littlefs_info(PARTITION_LABEL, &total, &used) != ESP_OK) return false;
    return total - used > MIN_FREE_BYTES;
}

static void list_rides(void)
{
    DIR *dir = opendir(LOGGER_BASE_PATH);
    if (!dir) return;
    struct dirent *e;
    int n = 0;
    while ((e = readdir(dir)) != NULL) {
        char path[300];
        struct stat st;
        size_t nl = strlen(e->d_name);
        if (nl < 5 || strcmp(e->d_name + nl - 4, ".trk") != 0) continue;  // e.g. events.txt
        snprintf(path, sizeof(path), LOGGER_BASE_PATH "/%s", e->d_name);
        if (stat(path, &st) == 0) {
            ESP_LOGI(TAG, "  ride %s: %ld points", e->d_name,
                     (long)((st.st_size - 8) / (long)sizeof(log_point_t)));
            n++;
        }
    }
    closedir(dir);
    ESP_LOGI(TAG, "%d ride(s) stored, %lu of %lu KB used", n,
             (unsigned long)s_status.fs_used_kb, (unsigned long)s_status.fs_total_kb);
}

static void flush_buffer(void)
{
    if (!s_file || s_buf_len == 0) return;
    fwrite(s_buf, sizeof(log_point_t), s_buf_len, s_file);
    fflush(s_file);
    fsync(fileno(s_file));
    s_buf_len = 0;
    portENTER_CRITICAL(&s_lock);
    s_status.flushes++;
    portEXIT_CRITICAL(&s_lock);
    s_last_flush_us = esp_timer_get_time();
    update_fs_usage();

    if (!fs_has_room()) {
        ESP_LOGW(TAG, "flash full, stopping this ride");
        fclose(s_file);
        s_file = NULL;
        portENTER_CRITICAL(&s_lock);
        s_status.recording = false;
        s_status.full = true;
        portEXIT_CRITICAL(&s_lock);
    }
}

static bool start_ride(const gps_fix_t *f)
{
    if (!fs_has_room()) {
        portENTER_CRITICAL(&s_lock);
        s_status.full = true;
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    char name[32], path[48];
    snprintf(name, sizeof(name), "%04d%02d%02d-%02d%02d%02d.trk",
             f->year, f->month, f->day, f->hour, f->minute, (int)f->second);
    snprintf(path, sizeof(path), LOGGER_BASE_PATH "/%s", name);

    s_file = fopen(path, "wb");
    if (!s_file) {
        ESP_LOGE(TAG, "can't create %s", path);
        return false;
    }
    const uint8_t header[8] = { 'G', 'P', 'S', 'L', LOGGER_FILE_VER,
                                sizeof(log_point_t), 0, 0 };
    fwrite(header, 1, sizeof(header), s_file);
    fflush(s_file);
    s_last_flush_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    s_status.recording = true;
    strlcpy(s_status.ride_name, name, sizeof(s_status.ride_name));
    s_status.points = 0;
    s_status.distance_m = 0;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "recording new ride %s", name);
    return true;
}

static void log_point(const gps_fix_t *f, double dist_m)
{
    log_point_t p = {
        .time       = fix_unix_time(f),
        .lat_e7     = (int32_t)lround(f->lat * 1e7),
        .lon_e7     = (int32_t)lround(f->lon * 1e7),
        .ele_m      = (int16_t)lroundf(f->alt_m),
        .speed_cmps = (uint16_t)lroundf(f->speed_kmh / 0.036f),
    };
    s_buf[s_buf_len++] = p;
    s_have_last = true;
    s_last_lat = f->lat;
    s_last_lon = f->lon;

    portENTER_CRITICAL(&s_lock);
    s_status.points++;
    s_status.distance_m += dist_m;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGD(TAG, "point %lu (+%.1f m)", (unsigned long)s_status.points, dist_m);
    if (s_buf_len == BUF_POINTS) flush_buffer();
}

static void end_ride(void)
{
    if (!s_file) return;
    flush_buffer();
    if (s_file) fclose(s_file);
    s_file = NULL;
    s_have_last = false;
    portENTER_CRITICAL(&s_lock);
    s_status.recording = false;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "ride closed");
}

static void logger_task(void *arg)
{
    uint32_t last_fix_count = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (s_paused) {
            end_ride();
            continue;
        }

        if (s_file && s_buf_len > 0 &&
            esp_timer_get_time() - s_last_flush_us > LOGGER_FLUSH_MS * 1000LL) {
            flush_buffer();
        }

        gps_fix_t f;
        gps_get_fix(&f);
        if (f.fix_count == last_fix_count) continue;  // nothing new
        last_fix_count = f.fix_count;

        if (s_status.full || !fix_good_enough(&f)) continue;
        if (!s_file && !start_ride(&f)) continue;

        if (!s_have_last) {
            log_point(&f, 0);
            continue;
        }
        double d = haversine_m(s_last_lat, s_last_lon, f.lat, f.lon);
        if (should_log(d, f.speed_kmh)) log_point(&f, d);
    }
}

void logger_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = LOGGER_BASE_PATH,
        .partition_label = PARTITION_LABEL,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "flash filesystem mount failed: %s", esp_err_to_name(err));
        return;
    }
    s_status.mounted = true;
    update_fs_usage();
    list_rides();
    xTaskCreate(logger_task, "logger", 4096, NULL, 4, NULL);
}

void logger_set_paused(bool paused)
{
    s_paused = paused;
    if (paused) {
        // wait for the logger task to close the ride file
        for (int i = 0; i < 20 && s_status.recording; i++) vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool logger_is_paused(void)
{
    return s_paused;
}

void logger_get_status(logger_status_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_lock);
}
