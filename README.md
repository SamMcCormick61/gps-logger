# GPS ride logger (ESP32-S3)

Low-power GPS position logger for cycling. An ESP32-S3 records a point every 10 m to on-board flash, and a small web UI lets you download each ride as GPX and update the firmware over the air.

Firmware version: 0.6.4 (ESP-IDF v6.0)

## Features

- Logs a point every 10 m of movement (or 30 m regardless of speed), gated on ≥4 satellites and HDOP ≤ 5.
- Ride logs on LittleFS in on-board flash, one file per power-on or wake.
- GPX export of each ride from the web page.
- Wi-Fi only on demand: hold **BOOT** for 2 s. It joins your saved home Wi-Fi (`http://gpslogger.local`) or starts its own hotspot `GPS-Logger-XXXX` (password `ridelogger`, `http://192.168.4.1`). Logging pauses while Wi-Fi is on, and Wi-Fi turns itself off after 10 min idle.
- OTA firmware upload from the web page (`build/gps_logger.bin`) with rollback.
- Motion sleep: a LIS3DH or LIS3DSH accelerometer wakes the board; after a set time without movement (default 300 s, adjustable on the web page) it deep-sleeps with the GPS power cut.
- Supports a u-blox NEO-6M or an ATGM336H (menuconfig → GPS logger → GPS module), 2 Hz updates.
- RGB status LED (WS2812).

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 board | Test board: ESP32-S3 Super Mini / S3 Zero (default build). Final: bare ESP32-S3-WROOM-1-N16R8. |
| GPS | NEO-6M or ATGM336H on UART |
| Accelerometer | LIS3DH or LIS3DSH on I2C, INT1 for wake |
| GPS power switch | AO3401 P-MOSFET (or a PNP/NPN transistor) on GPIO7 |
| Battery | Li-ion (e.g. 2× 18650 in parallel) into a low-quiescent 3.3 V LDO (HT7333, ME6211, RT9080) |

### Default pinout (Super Mini build)

| Signal | GPIO |
|---|---|
| GPS TX → S3 RX | 13 |
| GPS RX ← S3 TX | 12 |
| Accelerometer SDA / SCL | 5 / 6 |
| Accelerometer INT1 | 4 |
| GPS power switch | 7 |
| Wi-Fi button | 0 (BOOT) |

The N16R8 build moves the GPS to GPIO18 (RX) / 17 (TX). Full wiring for both boards, the power switch options and the module support parts are in [WIRING.md](WIRING.md).

## Building

From an ESP-IDF v6.0 shell (`export.bat` / `export.ps1`) in this folder:

```
idf.py set-target esp32s3
idf.py -p COMx flash monitor
```

Bare N16R8 module (16 MB flash, PSRAM):

```
idf.py fullclean
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.n16r8" set-target esp32s3
idf.py -p COMx flash monitor
```

Sleep-current test build (deep-sleeps 10 s after boot, nothing else):

```
idf.py -B build_sleeptest -D SDKCONFIG=build_sleeptest/sdkconfig -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sleeptest" -p COMx flash monitor
```

Settings live under `idf.py menuconfig` → **GPS logger**. Commit changes you want to keep to `sdkconfig.defaults` (or the board-specific file); the generated `sdkconfig` is not tracked.

## Status LED

| Pattern | Meaning |
|---|---|
| Red, green, blue at boot | LED test |
| Red blink every 2 s | No usable fix yet |
| Green flash | Point logged |
| Green blink every 3 s | Recording, not moving |
| Blue flash | Saved to flash |
| Fast red | Flash full |
| Cyan blink | Wi-Fi on |

## Layout

```
main/
  main.c         start-up and task wiring
  gps.c          NMEA parsing, module setup (UBX / PCAS)
  logger.c       10 m point logic, LittleFS ride files
  motion.c       LIS3DH / LIS3DSH motion wake
  power.c        deep sleep and GPS power switch
  status_led.c   WS2812 status LED
  webui.c        Wi-Fi, web server, GPX export, OTA
  web/index.html the web page
partitions.csv        4 MB layout (Super Mini)
partitions_16mb.csv   16 MB layout (N16R8)
```
