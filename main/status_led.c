#include "status_led.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "logger.h"
#include "sdkconfig.h"
#include "webui.h"

#if CONFIG_STATUS_LED_GPIO >= 0

static const char *TAG = "led";

#define TICK_MS     50
#define BRIGHTNESS  12   // out of 255; plenty to see, saves battery

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static volatile bool        s_off;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_chan) return;
    const uint8_t grb[3] = { g, r, b };  // WS2812 wants green first
    const rmt_transmit_config_t tx = { .loop_count = 0 };
    rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &tx);
    rmt_tx_wait_all_done(s_chan, 100);
}

static void led_flash(uint8_t r, uint8_t g, uint8_t b, int ms)
{
    led_set(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(ms));
    led_set(0, 0, 0);
}

static void led_task(void *arg)
{
    const uint8_t L = BRIGHTNESS;
    uint32_t last_points = 0, last_flushes = 0;
    int idle_ticks = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        if (s_off) continue;
        logger_status_t ls;
        gps_fix_t f;
        logger_get_status(&ls);
        gps_get_fix(&f);
        idle_ticks++;

        if (ls.full) {
            if (idle_ticks >= 4) { led_flash(L, 0, 0, 100); idle_ticks = 0; }
            continue;
        }
        if (ls.flushes != last_flushes) {
            last_flushes = ls.flushes;
            led_flash(0, 0, L, 150);
            idle_ticks = 0;
            continue;
        }
        if (ls.points != last_points) {
            last_points = ls.points;
            led_flash(0, L, 0, 60);
            idle_ticks = 0;
            continue;
        }
        bool fix_ok = f.valid && f.sats >= LOGGER_MIN_SATS && f.hdop <= LOGGER_MAX_HDOP;
        if (webui_is_active()) {
            if (idle_ticks >= 1000 / TICK_MS) { led_flash(0, L, L, 100); idle_ticks = 0; }
        } else if (ls.recording && fix_ok) {
            if (idle_ticks >= 3000 / TICK_MS) { led_flash(0, L, 0, 60); idle_ticks = 0; }
        } else {
            if (idle_ticks >= 2000 / TICK_MS) { led_flash(L, 0, 0, 60); idle_ticks = 0; }
        }
    }
}

void status_led_init(void)
{
    // The data line was held low through deep sleep: release it before RMT takes it.
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_dis();
#endif
    gpio_hold_dis(CONFIG_STATUS_LED_GPIO);
    gpio_reset_pin(CONFIG_STATUS_LED_GPIO);
    const rmt_tx_channel_config_t ch = {
        .gpio_num = CONFIG_STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 0.1 us ticks
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
    };
    // WS2812 bit timing: 0 = 0.3 us high + 0.9 us low, 1 = 0.9 us high + 0.3 us low
    const rmt_bytes_encoder_config_t enc = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    if (rmt_new_tx_channel(&ch, &s_chan) != ESP_OK ||
        rmt_new_bytes_encoder(&enc, &s_enc) != ESP_OK ||
        rmt_enable(s_chan) != ESP_OK) {
        ESP_LOGE(TAG, "LED init failed on GPIO%d", CONFIG_STATUS_LED_GPIO);
        s_chan = NULL;
        return;
    }
    ESP_LOGI(TAG, "status LED on GPIO%d: red, green, blue test", CONFIG_STATUS_LED_GPIO);
    led_flash(BRIGHTNESS, 0, 0, 300);
    led_flash(0, BRIGHTNESS, 0, 300);
    led_flash(0, 0, BRIGHTNESS, 300);
    xTaskCreate(led_task, "led", 3072, NULL, 3, NULL);
}

void status_led_off(void)
{
    s_off = true;
    vTaskDelay(pdMS_TO_TICKS(300));  // let any flash in progress finish
    led_set(0, 0, 0);
    // Before deep sleep: hold the LED data line low. Left floating, the WS2812
    // can pick up noise and glow dim red.
    if (s_chan) rmt_disable(s_chan);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(CONFIG_STATUS_LED_GPIO, 0);
    gpio_hold_en(CONFIG_STATUS_LED_GPIO);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP && !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_en();
#endif
}

#else

void status_led_init(void) {}  // no LED on this board
void status_led_off(void) {}

#endif
