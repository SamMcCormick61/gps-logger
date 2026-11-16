#include "motion.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_ACCEL_INT_GPIO >= 0

static const char *TAG = "motion";

// Supports two ST accelerometers; both have WHO_AM_I at 0x0F.
//   LIS3DH  : address 0x18/0x19, WHO_AM_I 0x33
//   LIS3DSH : address 0x1E/0x1D, WHO_AM_I 0x3F (chip marked "DSH")
#define WHO_AM_I        0x0F

// LIS3DH registers
#define DH_CTRL_REG1    0x20
#define DH_CTRL_REG2    0x21
#define DH_CTRL_REG3    0x22
#define DH_CTRL_REG4    0x23
#define DH_CTRL_REG5    0x24
#define DH_CTRL_REG6    0x25
#define DH_REFERENCE    0x26
#define DH_INT1_CFG     0x30
#define DH_INT1_SRC     0x31
#define DH_INT1_THS     0x32
#define DH_INT1_DUR     0x33

// LIS3DSH registers
#define DSH_CTRL_REG4   0x20   // ODR + axis enable
#define DSH_CTRL_REG1   0x21   // state machine 1 control
#define DSH_CTRL_REG3   0x23   // interrupt pin config
#define DSH_CTRL_REG5   0x24   // full scale
#define DSH_CTRL_REG6   0x25
#define DSH_ST1_1       0x40   // state machine 1 program
#define DSH_THRS1_1     0x57
#define DSH_MASK1_B     0x59
#define DSH_MASK1_A     0x5A
#define DSH_SETT1       0x5B
#define DSH_OUTS1       0x5F   // reading it clears the latched interrupt
#define DSH_OUT_X_L     0x28   // X/Y/Z, 16-bit little-endian, 0.06 mg per LSB at +-2 g

typedef enum { CHIP_NONE, CHIP_LIS3DH, CHIP_LIS3DSH } chip_t;

static i2c_master_dev_handle_t s_dev;
static chip_t                  s_chip;
static volatile int64_t        s_last_us;

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    const uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, 50);
}

static esp_err_t rd(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, 50);
}

// Reads X/Y/Z in mg (LIS3DSH only).
static bool read_xyz_mg(int *xyz)
{
    uint8_t reg = DSH_OUT_X_L, b[6];
    if (i2c_master_transmit_receive(s_dev, &reg, 1, b, 6, 50) != ESP_OK) return false;
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        xyz[i] = raw * 6 / 100;
    }
    return true;
}

// Clears the latched movement interrupt. Returns true if movement had been latched.
static bool clear_latch(void)
{
    uint8_t v = 0;
    if (s_chip == CHIP_LIS3DH) {
        return rd(DH_INT1_SRC, &v) == ESP_OK && (v & 0x40);
    }
    bool was_high = gpio_get_level(CONFIG_ACCEL_INT_GPIO) == 1;
    rd(DSH_OUTS1, &v);
    return was_high || v != 0;
}

static void motion_task(void *arg)
{
    int prev[3] = { 0 };
    bool have_prev = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (s_chip == CHIP_LIS3DH) {
            if (clear_latch()) s_last_us = esp_timer_get_time();
            continue;
        }
        // LIS3DSH has no high-pass filter, so while awake compare each reading with
        // the last one: gravity is the same every time and cancels out.
        int xyz[3];
        if (!read_xyz_mg(xyz)) continue;
        if (have_prev) {
            for (int i = 0; i < 3; i++) {
                if (abs(xyz[i] - prev[i]) > CONFIG_ACCEL_WAKE_THRESHOLD_MG) {
                    s_last_us = esp_timer_get_time();
                    break;
                }
            }
        }
        memcpy(prev, xyz, sizeof(prev));
        have_prev = true;
    }
}

// Logs every address that answers; returns how many did.
static int scan_bus(i2c_master_bus_handle_t bus, const char *label)
{
    int n = 0;
    for (uint16_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(bus, a, 20) == ESP_OK) {
            ESP_LOGI(TAG, "I2C scan %s: device at 0x%02x", label, a);
            n++;
        }
    }
    if (!n) ESP_LOGI(TAG, "I2C scan %s: nothing answered", label);
    return n;
}

static void setup_lis3dh(void)
{
    // 10 Hz low-power mode, X/Y/Z on: about 4 uA, keeps running while the S3 sleeps
    wr(DH_CTRL_REG1, 0x2F);
    wr(DH_CTRL_REG2, 0x01);   // high-pass filter on interrupt 1, so gravity/tilt is ignored
    wr(DH_CTRL_REG3, 0x40);   // interrupt 1 activity -> INT1 pin
    wr(DH_CTRL_REG4, 0x00);   // +-2 g
    wr(DH_CTRL_REG5, 0x08);   // latch INT1 until INT1_SRC is read
    wr(DH_CTRL_REG6, 0x00);   // INT pins active high
    wr(DH_INT1_THS, CONFIG_ACCEL_WAKE_THRESHOLD_MG / 16);  // 16 mg per step
    wr(DH_INT1_DUR, 0);
    uint8_t dummy;
    rd(DH_REFERENCE, &dummy); // resets the high-pass filter to the current position
    wr(DH_INT1_CFG, 0x2A);    // any of X, Y or Z above threshold
    rd(DH_INT1_SRC, &dummy);
}

static void setup_lis3dsh(void)
{
    wr(DSH_CTRL_REG6, 0x80);  // reboot memory content
    vTaskDelay(pdMS_TO_TICKS(20));
    wr(DSH_CTRL_REG6, 0x10);  // auto-increment address on multi-byte reads
    wr(DSH_CTRL_REG4, 0x47);  // 25 Hz, X/Y/Z on
    wr(DSH_CTRL_REG5, 0x00);  // +-2 g
    wr(DSH_CTRL_REG1, 0x00);  // wake state machine off while awake (armed before sleep)
    wr(DSH_CTRL_REG3, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));  // first samples
}

// Before deep sleep: set the wake trigger just above how the board is lying now.
// The chip can only compare |each axis| with one threshold, so it is set above the
// axis that carries gravity, plus the wake margin.
static void arm_lis3dsh(void)
{
    int xyz[3] = { 0, 0, 1000 };
    read_xyz_mg(xyz);
    int peak = 0;
    for (int i = 0; i < 3; i++) if (abs(xyz[i]) > peak) peak = abs(xyz[i]);
    int ths = (peak + CONFIG_ACCEL_WAKE_THRESHOLD_MG) * 128 / 2000 + 1;  // 15.6 mg per step
    if (ths > 127) ths = 127;
    ESP_LOGI(TAG, "resting at x %d y %d z %d mg, wake above %d mg",
             xyz[0], xyz[1], xyz[2], ths * 2000 / 128);

    wr(DSH_CTRL_REG1, 0x00);
    wr(DSH_THRS1_1, (uint8_t)ths);
    wr(DSH_ST1_1 + 0, 0x05);  // state 1: wait until |any axis| > THRS1
    wr(DSH_ST1_1 + 1, 0x11);  // state 2: CONT = raise interrupt and start again
    wr(DSH_MASK1_B, 0xFC);    // +X -X +Y -Y +Z -Z
    wr(DSH_MASK1_A, 0xFC);
    wr(DSH_SETT1, 0x01);      // SITR: CONT raises the interrupt
    wr(DSH_CTRL_REG3, 0x48);  // INT1 on, active high, latched
    wr(DSH_CTRL_REG1, 0x01);  // state machine 1 on, routed to INT1
}

bool motion_init(void)
{
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_ACCEL_SDA_GPIO,
        .scl_io_num = CONFIG_ACCEL_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  // the breakout normally has its own too
    };
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;

    static const uint16_t addrs[] = { 0x19, 0x18, 0x1D, 0x1E };
    uint16_t addr = 0;
    for (int i = 0; i < 4 && !addr; i++) {
        if (i2c_master_probe(bus, addrs[i], 50) == ESP_OK) addr = addrs[i];
    }
    if (!addr) {
        ESP_LOGW(TAG, "no accelerometer on SDA %d / SCL %d, deep sleep disabled",
                 CONFIG_ACCEL_SDA_GPIO, CONFIG_ACCEL_SCL_GPIO);
        scan_bus(bus, "as wired");
        i2c_del_master_bus(bus);
        // wiring check: try again with SDA and SCL swapped
        i2c_master_bus_config_t sw = bc;
        sw.sda_io_num = CONFIG_ACCEL_SCL_GPIO;
        sw.scl_io_num = CONFIG_ACCEL_SDA_GPIO;
        if (i2c_new_master_bus(&sw, &bus) == ESP_OK) {
            if (scan_bus(bus, "with SDA/SCL swapped")) {
                ESP_LOGW(TAG, "-> the SDA and SCL wires are swapped, swap them over");
            }
            i2c_del_master_bus(bus);
        }
        return false;
    }
    const i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus, &dc, &s_dev);

    uint8_t who = 0;
    rd(WHO_AM_I, &who);
    const char *name;
    if (who == 0x33) {
        s_chip = CHIP_LIS3DH;
        name = "LIS3DH";
    } else if (who == 0x3F) {
        s_chip = CHIP_LIS3DSH;
        name = "LIS3DSH";
    } else {
        ESP_LOGW(TAG, "chip at 0x%02x has WHO_AM_I 0x%02x, not a LIS3DH/LIS3DSH, "
                      "deep sleep disabled", addr, who);
        s_dev = NULL;
        return false;
    }

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_ACCEL_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // stays low if the wire comes off
    };
    gpio_config(&io);

    if (s_chip == CHIP_LIS3DH) setup_lis3dh();
    else                       setup_lis3dsh();

    s_last_us = esp_timer_get_time();
    xTaskCreate(motion_task, "motion", 2560, NULL, 3, NULL);
    ESP_LOGI(TAG, "%s found at 0x%02x, wake threshold %d mg", name, addr,
             CONFIG_ACCEL_WAKE_THRESHOLD_MG);
    return true;
}

bool motion_present(void)
{
    return s_dev != NULL;
}

int64_t motion_last_us(void)
{
    return s_last_us;
}

bool motion_arm_for_sleep(void)
{
    if (s_chip == CHIP_LIS3DSH) arm_lis3dsh();
    clear_latch();
    vTaskDelay(pdMS_TO_TICKS(150));  // give it a few more samples
    return gpio_get_level(CONFIG_ACCEL_INT_GPIO) == 0;
}

#else  // no accelerometer: never sleeps

bool motion_init(void) { return false; }
bool motion_present(void) { return false; }
int64_t motion_last_us(void) { return 0; }
bool motion_arm_for_sleep(void) { return false; }

#endif
