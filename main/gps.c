#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gps";

#define NMEA_LINE_MAX   100
#define MAX_FIELDS 24

static gps_fix_t s_fix;
static volatile uint32_t s_sentences;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Verifies "$...*HH" checksum and strips the "*HH" part.
static bool nmea_checksum_ok(char *line)
{
    char *star = strrchr(line, '*');
    if (line[0] != '$' || !star || !star[1] || !star[2]) return false;
    uint8_t sum = 0;
    for (char *p = line + 1; p < star; p++) sum ^= (uint8_t)*p;
    int hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0) return false;
    *star = '\0';
    return sum == (uint8_t)((hi << 4) | lo);
}

// Splits on commas, keeping empty fields (strtok would drop them).
static int split_fields(char *s, char **fields)
{
    int n = 0;
    fields[n++] = s;
    for (char *p = s; *p && n < MAX_FIELDS; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

// NMEA "ddmm.mmmm" / "dddmm.mmmm" + hemisphere -> signed decimal degrees
static double nmea_to_deg(const char *v, const char *hemi)
{
    if (!*v) return 0;
    double raw = atof(v);
    int deg = (int)(raw / 100);
    double d = deg + (raw - deg * 100) / 60.0;
    if (*hemi == 'S' || *hemi == 'W') d = -d;
    return d;
}

static void parse_rmc(char **f, int n)
{
    // $xxRMC,time,status,lat,N,lon,E,speed_kn,course,date,...
    if (n < 10) return;
    gps_fix_t u;
    portENTER_CRITICAL(&s_lock);
    u = s_fix;
    portEXIT_CRITICAL(&s_lock);

    u.valid = (f[2][0] == 'A');
    if (strlen(f[1]) >= 6) {
        u.hour   = (f[1][0] - '0') * 10 + (f[1][1] - '0');
        u.minute = (f[1][2] - '0') * 10 + (f[1][3] - '0');
        u.second = atof(f[1] + 4);
    }
    if (strlen(f[9]) == 6) {
        u.day   = (f[9][0] - '0') * 10 + (f[9][1] - '0');
        u.month = (f[9][2] - '0') * 10 + (f[9][3] - '0');
        u.year  = 2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0');
    }
    if (u.valid) {
        u.lat        = nmea_to_deg(f[3], f[4]);
        u.lon        = nmea_to_deg(f[5], f[6]);
        u.speed_kmh  = atof(f[7]) * 1.852f;
        u.course_deg = atof(f[8]);
        u.updated_us = esp_timer_get_time();
        u.fix_count++;
    }

    portENTER_CRITICAL(&s_lock);
    s_fix = u;
    portEXIT_CRITICAL(&s_lock);
}

static void parse_gga(char **f, int n)
{
    // $xxGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...
    if (n < 10) return;
    portENTER_CRITICAL(&s_lock);
    s_fix.fix_quality = atoi(f[6]);
    s_fix.sats        = atoi(f[7]);
    s_fix.hdop        = f[8][0] ? atof(f[8]) : 99.9f;
    s_fix.alt_m       = atof(f[9]);
    portEXIT_CRITICAL(&s_lock);
}

static void parse_gsv(char **f, int n)
{
    // $xxGSV,total_msgs,msg_num,in_view,{prn,elev,az,snr}x(1..4)
    static int heard, best;
    if (n < 4) return;
    int total = atoi(f[1]), num = atoi(f[2]);
    if (num == 1) heard = best = 0;
    for (int i = 7; i < n; i += 4) {
        int snr = atoi(f[i]);
        if (snr > 0) heard++;
        if (snr > best) best = snr;
    }
    if (num == total) {
        portENTER_CRITICAL(&s_lock);
        s_fix.sats_in_view = atoi(f[3]);
        s_fix.sats_heard   = heard;
        s_fix.best_snr     = best;
        portEXIT_CRITICAL(&s_lock);
    }
}

static void handle_line(char *line)
{
    if (!nmea_checksum_ok(line)) {
        ESP_LOGD(TAG, "bad checksum: %s", line);
        return;
    }
    s_sentences++;
    char *f[MAX_FIELDS];
    int n = split_fields(line, f);
    const char *type = f[0] + 3;  // skip "$GP" / "$GN" talker id
    if (strlen(f[0]) < 6) return;
    if (strcmp(type, "RMC") == 0) parse_rmc(f, n);
    else if (strcmp(type, "GGA") == 0) parse_gga(f, n);
    else if (strcmp(type, "GSV") == 0) parse_gsv(f, n);
}

static void gps_task(void *arg)
{
    uint8_t buf[128];
    char line[NMEA_LINE_MAX];
    int len = 0;

    for (;;) {
        int r = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < r; i++) {
            char c = (char)buf[i];
            if (c == '$') len = 0;           // resync on sentence start
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
                    handle_line(line);
                    len = 0;
                }
            } else if (len < NMEA_LINE_MAX - 1) {
                line[len++] = c;
            } else {
                len = 0;                     // overlong garbage, drop it
            }
        }
    }
}

// P-MOSFET high-side switch: gate pulled up to 3.3 V by a resistor, so the GPS is
// off unless we drive the gate low. In deep sleep the pin floats and the GPS stays off.
void gps_power(bool on)
{
#if CONFIG_GPS_POWER_GPIO >= 0
    if (on) {
        gpio_set_direction(CONFIG_GPS_POWER_GPIO, GPIO_MODE_OUTPUT);
#if CONFIG_GPS_POWER_ACTIVE_HIGH
        gpio_set_level(CONFIG_GPS_POWER_GPIO, 1);   // NPN in the GND line
#else
        gpio_set_level(CONFIG_GPS_POWER_GPIO, 0);   // P-MOSFET/PNP in the VCC line
#endif
    } else {
        gpio_set_direction(CONFIG_GPS_POWER_GPIO, GPIO_MODE_INPUT);  // resistor turns it off
    }
    ESP_LOGI(TAG, "GPS power %s", on ? "on" : "off");
#endif
}

void gps_init(void)
{
    gps_power(true);
    const uart_config_t cfg = {
        .baud_rate  = GPS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(gps_task, "gps", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART%d @ %d baud, TX=GPIO%d RX=GPIO%d",
             GPS_UART_NUM, GPS_BAUD, GPS_TX_PIN, GPS_RX_PIN);
}

#if CONFIG_GPS_MODEL_ATGM336H

// ---- CASIC configuration for the ATGM336H ($PCAS commands, NMEA-style checksum) ----

static void casic_send(const char *body)
{
    uint8_t ck = 0;
    for (const char *c = body; *c; c++) ck ^= (uint8_t)*c;
    char line[64];
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, ck);
    uart_write_bytes(GPS_UART_NUM, line, n);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void gps_configure(void)
{
    casic_send("PCAS02,500");  // fix every 500 ms (2 Hz)
    ESP_LOGI(TAG, "ATGM336H: requested 2 Hz updates");
}

#else

// ---- UBX configuration (not saved in the GPS, so it's re-sent every boot) ----

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t msg[6 + 32 + 2];
    msg[0] = 0xB5; msg[1] = 0x62;
    msg[2] = cls;  msg[3] = id;
    msg[4] = len & 0xFF; msg[5] = len >> 8;
    memcpy(&msg[6], payload, len);
    uint8_t ck_a = 0, ck_b = 0;  // Fletcher checksum over class..payload
    for (int i = 2; i < 6 + len; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[6 + len] = ck_a;
    msg[7 + len] = ck_b;
    uart_write_bytes(GPS_UART_NUM, msg, 8 + len);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void ubx_set_nmea_rate(uint8_t nmea_id, uint8_t every_n_epochs)
{
    const uint8_t p[3] = { 0xF0, nmea_id, every_n_epochs };  // CFG-MSG
    ubx_send(0x06, 0x01, p, sizeof(p));
}

void gps_configure(void)
{
    // At 2 Hz the default sentence set is close to what 9600 baud can carry,
    // so keep only what we use: RMC + GGA every fix, GSV every 5th fix.
    ubx_set_nmea_rate(0x01, 0);  // GLL off
    ubx_set_nmea_rate(0x02, 0);  // GSA off
    ubx_set_nmea_rate(0x05, 0);  // VTG off
    ubx_set_nmea_rate(0x03, 5);  // GSV
    ubx_set_nmea_rate(0x00, 1);  // GGA
    ubx_set_nmea_rate(0x04, 1);  // RMC

    // CFG-RATE: measRate 500 ms, navRate 1, timeRef GPS
    const uint8_t rate[6] = { 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00 };
    ubx_send(0x06, 0x08, rate, sizeof(rate));
    ESP_LOGI(TAG, "requested 2 Hz updates, RMC+GGA only");
}

#endif  // CONFIG_GPS_MODEL_ATGM336H

void gps_get_fix(gps_fix_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_fix;
    portEXIT_CRITICAL(&s_lock);
}

uint32_t gps_sentence_count(void)
{
    return s_sentences;
}
