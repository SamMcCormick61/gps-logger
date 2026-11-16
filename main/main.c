#include <stdio.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "soc/soc_caps.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "logger.h"
#include "motion.h"
#include "power.h"
#include "status_led.h"
#include "webui.h"

static const char *TAG = "main";

// With rollback enabled, a freshly OTA'd image must confirm itself or the
// bootloader reverts to the previous one on the next reset.
static void confirm_running_app(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "new firmware booted OK, cancelling rollback");
        esp_ota_mark_app_valid_cancel_rollback();
    }
    ESP_LOGI(TAG, "running from partition '%s'", running->label);
}

#if CONFIG_SLEEP_CURRENT_TEST
// Measures only the S3 board: nothing else is started.
static void sleep_current_test(void)
{
#if CONFIG_GPS_POWER_GPIO >= 0
    gpio_reset_pin(CONFIG_GPS_POWER_GPIO);
    gpio_set_direction(CONFIG_GPS_POWER_GPIO, GPIO_MODE_INPUT);   // resistor switches the GPS off
    gpio_set_pull_mode(CONFIG_GPS_POWER_GPIO, GPIO_FLOATING);
#endif
#if CONFIG_STATUS_LED_GPIO >= 0
    gpio_reset_pin(CONFIG_STATUS_LED_GPIO);
    gpio_set_direction(CONFIG_STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_STATUS_LED_GPIO, 0);
#endif
    for (int i = 10; i > 0; i--) {
        ESP_LOGW(TAG, "SLEEP CURRENT TEST build: deep sleep in %d s (flash a normal build now "
                      "if you want to leave test mode)", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#if CONFIG_STATUS_LED_GPIO >= 0
    gpio_hold_en(CONFIG_STATUS_LED_GPIO);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_en();
#endif
#endif
    esp_sleep_enable_ext1_wakeup_io(1ULL << CONFIG_WIFI_BUTTON_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
    rtc_gpio_pulldown_dis(CONFIG_WIFI_BUTTON_GPIO);
    rtc_gpio_pullup_en(CONFIG_WIFI_BUTTON_GPIO);
    ESP_LOGW(TAG, "SLEEP CURRENT TEST: sleeping now. Press BOOT to wake.");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
}
#endif

void app_main(void)
{
#if CONFIG_SLEEP_CURRENT_TEST
#if CONFIG_STATUS_LED_GPIO >= 0
    gpio_hold_dis(CONFIG_STATUS_LED_GPIO);
#endif
    rtc_gpio_deinit(CONFIG_WIFI_BUTTON_GPIO);
    sleep_current_test();
#endif
    confirm_running_app();
    power_init();
    esp_err_t err = nvs_flash_init();  // Wi-Fi keeps its calibration data here
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    status_led_init();
    gps_init();
    vTaskDelay(pdMS_TO_TICKS(1000));  // let the GPS finish booting before configuring it
    gps_configure();
    logger_init();
    webui_init();
    motion_init();
    power_start();

    uint32_t last_fix_count = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        gps_fix_t f;
        logger_status_t ls;
        gps_get_fix(&f);
        logger_get_status(&ls);

        float hz = (f.fix_count - last_fix_count) / 5.0f;
        last_fix_count = f.fix_count;

        if (f.valid) {
            ESP_LOGI(TAG, "%02d:%02d:%02.0fZ  %.6f, %.6f  alt %.0fm  %.1f km/h  "
                          "sats %d  hdop %.1f  %.1f fix/s",
                     f.hour, f.minute, f.second, f.lat, f.lon, f.alt_m,
                     f.speed_kmh, f.sats, f.hdop, hz);
        } else if (gps_sentence_count() == 0) {
            ESP_LOGW(TAG, "no data from GPS yet - check TX/RX wiring and power");
        } else {
            ESP_LOGI(TAG, "waiting for fix... in view %d, heard %d, best signal %d dB-Hz, used %d",
                     f.sats_in_view, f.sats_heard, f.best_snr, f.sats);
        }

        if (webui_is_connecting()) {
            ESP_LOGI(TAG, "Wi-Fi starting: trying home network  (logging paused)");
        } else if (webui_is_active()) {
            if (webui_on_home_wifi()) {
                ESP_LOGI(TAG, "Wi-Fi on (home): open http://gpslogger.local or http://%s  "
                              "(logging paused)", webui_address());
            } else {
                ESP_LOGI(TAG, "Wi-Fi on (hotspot): join '%s', open http://192.168.4.1  "
                              "(logging paused)", webui_ssid());
            }
        } else if (ls.recording) {
            ESP_LOGI(TAG, "REC %s  %lu points  %.2f km  flash %lu/%lu KB",
                     ls.ride_name, (unsigned long)ls.points, ls.distance_m / 1000.0,
                     (unsigned long)ls.fs_used_kb, (unsigned long)ls.fs_total_kb);
        } else if (ls.full) {
            ESP_LOGW(TAG, "NOT RECORDING: flash full");
        } else if (f.valid) {
            ESP_LOGI(TAG, "not recording yet: need %d+ sats and hdop <= %.0f",
                     LOGGER_MIN_SATS, LOGGER_MAX_HDOP);
        }
    }
}
