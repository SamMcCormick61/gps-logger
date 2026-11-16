#include "power.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "driver/rtc_io.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "logger.h"
#include "motion.h"
#include "sdkconfig.h"
#include "status_led.h"
#include "webui.h"

static const char *TAG = "power";

#define SLEEP_MIN_S      30
#define SLEEP_MAX_S      86400
#define EVENTS_MAX_BYTES 16384   // then events.txt becomes events.old and a new one starts

static uint32_t          s_wake_causes;
static esp_reset_reason_t s_reset_reason;
static volatile uint32_t s_sleep_s = CONFIG_SLEEP_IDLE_MIN * 60;

// ---- event log ----

void power_event(const char *fmt, ...)
{
    char when[32];
    time_t now = time(NULL);
    if (now > 1700000000) {   // clock set from GPS (it keeps running through deep sleep)
        struct tm tm;
        gmtime_r(&now, &tm);
        strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%SZ", &tm);
    } else {
        snprintf(when, sizeof(when), "(no clock) +%llds",
                 (long long)(esp_timer_get_time() / 1000000));
    }
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "event: %s", msg);

    struct stat st;
    if (stat(POWER_EVENTS_PATH, &st) == 0 && st.st_size > EVENTS_MAX_BYTES) {
        unlink(POWER_EVENTS_OLD_PATH);
        rename(POWER_EVENTS_PATH, POWER_EVENTS_OLD_PATH);
    }
    FILE *f = fopen(POWER_EVENTS_PATH, "a");
    if (!f) return;
    fprintf(f, "%s  %s\n", when, msg);
    fclose(f);
}

static const char *reset_text(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "power on";
    case ESP_RST_SW:        return "restart (firmware update or software)";
    case ESP_RST_PANIC:     return "CRASH (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "CRASH (watchdog)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (battery too low?)";
    case ESP_RST_USB:       return "reset from USB";
    case ESP_RST_EXT:       return "reset button";
    default:                return "reset";
    }
}

// ---- sleep delay setting ----

uint32_t power_get_sleep_s(void)
{
    return s_sleep_s;
}

void power_set_sleep_s(uint32_t s)
{
    if (s != 0 && s < SLEEP_MIN_S) s = SLEEP_MIN_S;
    if (s > SLEEP_MAX_S) s = SLEEP_MAX_S;
    s_sleep_s = s;
    nvs_handle_t h;
    if (nvs_open("power", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "sleep_s", s);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "sleep delay set to %lu s", (unsigned long)s);
}

static void load_sleep_s(void)
{
    nvs_handle_t h;
    uint32_t v;
    if (nvs_open("power", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u32(h, "sleep_s", &v) == ESP_OK) s_sleep_s = v;
        nvs_close(h);
    }
}

void power_init(void)
{
    s_wake_causes = esp_sleep_get_wakeup_causes();
    s_reset_reason = esp_reset_reason();
    if (s_wake_causes & BIT(ESP_SLEEP_WAKEUP_EXT0)) {
        ESP_LOGI(TAG, "woke from deep sleep: movement");
    } else if (s_wake_causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) {
        ESP_LOGI(TAG, "woke from deep sleep: button");
    }
#if CONFIG_ACCEL_INT_GPIO >= 0
    rtc_gpio_deinit(CONFIG_ACCEL_INT_GPIO);       // hand the pins back to normal GPIO use
#endif
    rtc_gpio_deinit(CONFIG_WIFI_BUTTON_GPIO);
}

#if CONFIG_ACCEL_INT_GPIO >= 0

static bool s_clock_set;

// Sets the system clock from the GPS once. The S3's RTC keeps it running through
// deep sleep, so wake-up events get a real time even before the next fix.
static void set_clock_from_gps(const gps_fix_t *f)
{
    if (s_clock_set || !f->valid || f->year < 2024) return;
    struct tm tm = {
        .tm_year = f->year - 1900, .tm_mon = f->month - 1, .tm_mday = f->day,
        .tm_hour = f->hour, .tm_min = f->minute, .tm_sec = (int)f->second,
    };
    setenv("TZ", "UTC0", 1);
    tzset();
    struct timeval tv = { .tv_sec = mktime(&tm) };
    settimeofday(&tv, NULL);
    s_clock_set = true;
}

static void go_to_sleep(uint32_t still_s)
{
    ESP_LOGI(TAG, "no movement for %lu s, going to deep sleep", (unsigned long)still_s);
    logger_set_paused(true);    // closes and saves the ride
    if (!motion_arm_for_sleep()) {
        ESP_LOGI(TAG, "moved just now, staying awake");
        logger_set_paused(false);
        return;
    }
    power_event("sleep: still for %lu s, GPS off, going to deep sleep", (unsigned long)still_s);
    gps_power(false);
    status_led_off();

    // movement: accelerometer INT1 goes high
    esp_sleep_enable_ext0_wakeup(CONFIG_ACCEL_INT_GPIO, 1);
    rtc_gpio_pullup_dis(CONFIG_ACCEL_INT_GPIO);
    rtc_gpio_pulldown_en(CONFIG_ACCEL_INT_GPIO);
    // BOOT button: goes low when pressed
    esp_sleep_enable_ext1_wakeup_io(1ULL << CONFIG_WIFI_BUTTON_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
    rtc_gpio_pulldown_dis(CONFIG_WIFI_BUTTON_GPIO);
    rtc_gpio_pullup_en(CONFIG_WIFI_BUTTON_GPIO);

    vTaskDelay(pdMS_TO_TICKS(100));  // let the log line get out
    esp_deep_sleep_start();
}

static void power_task(void *arg)
{
    int64_t busy_us = esp_timer_get_time();  // last time Wi-Fi was on or GPS saw real speed

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int64_t now = esp_timer_get_time();
        gps_fix_t f;
        gps_get_fix(&f);
        set_clock_from_gps(&f);
        if (webui_is_active() || webui_is_connecting() || (f.valid && f.speed_kmh > 6.0f)) {
            busy_us = now;
        }
        uint32_t sleep_s = s_sleep_s;
        if (sleep_s == 0) continue;   // never sleep
        int64_t last = motion_last_us() > busy_us ? motion_last_us() : busy_us;
        if (now - last > (int64_t)sleep_s * 1000000LL) {
            go_to_sleep((uint32_t)((now - last) / 1000000LL));
            busy_us = esp_timer_get_time();  // only reached if it decided to stay awake
        }
    }
}

#endif

void power_start(void)
{
    load_sleep_s();
    if (s_wake_causes & BIT(ESP_SLEEP_WAKEUP_EXT0)) {
        power_event("wake: movement");
    } else if (s_wake_causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) {
        power_event("wake: BOOT button");
    } else {
        power_event("start: %s, firmware %s", reset_text(s_reset_reason),
                    esp_app_get_description()->version);
    }
#if CONFIG_ACCEL_INT_GPIO >= 0
    if (!motion_present()) {
        power_event("no accelerometer found, deep sleep disabled");
        return;
    }
    xTaskCreate(power_task, "power", 4096, NULL, 2, NULL);
    if (s_sleep_s) ESP_LOGI(TAG, "deep sleep after %lu s without movement", (unsigned long)s_sleep_s);
    else           ESP_LOGI(TAG, "deep sleep turned off (sleep delay 0)");
#endif
}
