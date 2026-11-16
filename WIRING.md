# Wiring

## Breadboard test: ESP32-S3 Super Mini + NEO-6M (the default build)

| NEO-6M | Super Mini pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| TX | 13, the S3's RX |
| RX | 12, the S3's TX |

That's all. The Super Mini already has the regulator, EN/BOOT parts and USB-C. Power it from USB for now.

Flash it from the ESP-IDF 6.0 command prompt in the `gps-logger` folder:
```
idf.py set-target esp32s3
idf.py -p COMx flash monitor
```
If flashing fails to connect, hold BOOT, tap RST, release BOOT, then retry.

### Motion sensor (LIS3DH breakout), for deep sleep

| LIS3DH board | Super Mini pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | 5 |
| SCL | 6 |
| INT1 | 4 |
| CS | 3V3 (selects I2C; most boards already pull it up) |
| SDO | leave open, or GND (either address works) |

The serial log shows `LIS3DH found at 0x19` at start-up. Without the sensor it just says deep sleep is disabled and carries on as before.

After 5 minutes with no movement and Wi-Fi off, it saves the ride, turns the LED off and deep-sleeps. A nudge to the board, or pressing BOOT, wakes it. Each wake starts a new ride file. The sensor can be a LIS3DH (`found at 0x19`) or a LIS3DSH (`found at 0x1d`); some boards sold as LIS3DH carry a LIS3DSH (chip marked DSH). Both work. Use CS to 3V3 and SDO to 3V3.

### GPS power switch (turns the GPS off during deep sleep)

| Part | Connection |
|---|---|
| AO3401 P-MOSFET, source (S) | 3V3 |
| AO3401 drain (D) | GPS VCC (instead of 3V3) |
| AO3401 gate (G) | 7 |
| 100 kΩ resistor | gate to 3V3 (keeps the GPS off while the S3 sleeps) |

**Or with a PNP transistor (2N4125, 2N3906, BC327, S8550) in the GPS VCC line** (the default build, pin low = on):

| Part | Connection |
|---|---|
| PNP emitter (E) | 3V3 |
| PNP collector (C) | GPS VCC (instead of 3V3) |
| PNP base (B) | 1 kΩ resistor to pin 7 |
| 10 kΩ resistor | base to 3V3 (keeps the GPS off while the S3 sleeps) |

2N4125 / 2N3906 (TO-92), flat face towards you, legs down: E, B, C from left to right. GPS GND stays on GND.

**Or with an NPN transistor (C945, 2N2222, BC547, S8050) in the GPS ground line** (set `CONFIG_GPS_POWER_ACTIVE_HIGH=y`):

| Part | Connection |
|---|---|
| NPN emitter (E) | GND |
| NPN collector (C) | GPS GND (instead of GND) |
| NPN base (B) | 1 kΩ resistor to pin 7 |
| 10 kΩ resistor | base to GND (keeps the GPS off while the S3 sleeps) |

C945 (TO-92), flat face towards you, legs down: E, C, B from left to right. GPS VCC stays on 3V3.

AO3401 (SOT-23, looking at the top with the single leg at the top): G bottom-left, S bottom-right, D top. GPS GND stays on GND. The log shows `GPS power on` at start-up. Any small logic-level P-MOSFET works (AO3401, Si2301, DMG2305); a 2N7000 or other N-channel won't.

---

# Final build: bare ESP32-S3-WROOM-1-N16R8 + NEO-6M

The firmware for this board is built with its own settings (16 MB flash, PSRAM, GPS on GPIO17/18):
```
idf.py fullclean
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.n16r8" set-target esp32s3
idf.py -p COMx flash monitor
```

Pin numbers below are the module's physical pads (WROOM-1 pinout), with the GPIO in brackets.

## Power (your 3.3 V supply)

| From | To | Notes |
|---|---|---|
| 3.3 V | Pad 2 (3V3) | Put 10 µF + 100 nF right next to the pad. Wi-Fi bursts draw ~500 mA. |
| GND | Pad 1, Pad 40 and the big centre pad (41) | Solder the centre pad if you can; it's also the main heat path. |

## Must-have support parts

| Part | Connection | Why |
|---|---|---|
| 10 kΩ resistor | EN (pad 3) to 3.3 V | Chip runs when EN is high |
| 1 µF capacitor | EN (pad 3) to GND | Delays start-up until 3.3 V is stable. Without it the module may not boot reliably. |
| Push button | GPIO0 (pad 27) to GND | Hold at power-up for download mode. Later this is the "start Wi-Fi" button. |
| 10 kΩ resistor | GPIO0 (pad 27) to 3.3 V | Keeps GPIO0 high for a normal boot |
| Push button (optional) | EN (pad 3) to GND | Reset button. Handy, otherwise power-cycle instead. |

## USB (first flash and serial monitor only)

Cut a USB cable or use a USB breakout. **Do not connect USB 5 V (VBUS) to anything.** The board runs from your 3.3 V supply.

| USB wire | Module |
|---|---|
| D− (white) | GPIO19 (pad 13) |
| D+ (green) | GPIO20 (pad 14) |
| GND (black) | GND |

## NEO-6M

| NEO-6M | Module |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| TX | GPIO18 (pad 11), the S3's RX |
| RX | GPIO17 (pad 10), the S3's TX |

Most NEO-6M breakouts (GY-NEO6MV2) have their own small regulator. Fed with 3.3 V they get about 3.1 V, which is still inside the NEO-6M's 2.7 to 3.6 V range. If yours behaves oddly, that regulator can be bypassed in step 4.

## Battery voltage sense (optional now, used in step 3)

| Part | Connection |
|---|---|
| 100 kΩ | Battery + to GPIO1 (pad 39) |
| 100 kΩ | GPIO1 (pad 39) to GND |
| 100 nF | GPIO1 to GND |

This halves 4.2 V to 2.1 V for the ADC. It costs about 20 µA all the time, which is negligible.

## Pins to leave alone

- **GPIO35, 36, 37**: used internally by the 8 MB PSRAM on the N16R8. Do not connect them.
- **GPIO45, GPIO46, GPIO3**: strapping pins. Leave GPIO45 unconnected. If it's pulled high at boot, the flash voltage is set wrong and the module won't start.
- **GPIO43/44 (pads 37/36)**: UART0. Free to leave unconnected since logs go over USB.

## First flash

1. Power up while holding the GPIO0 button (or hold it, tap reset, release).
2. Plug in USB. Windows should show a new COM port ("USB JTAG/serial debug unit").
3. Run the three N16R8 build commands at the top of this section.
   Replace COMx with your port. Later flashes usually don't need the button because the USB port can reset the chip into download mode by itself.

## Motion sensor (LIS3DH)

Same as the Super Mini: SDA to GPIO5 (pad 5), SCL to GPIO6 (pad 6), INT1 to GPIO4 (pad 4), CS to 3.3 V, VCC and GND.

## GPS power switch (so the GPS is off during deep sleep)

| Part | Connection |
|---|---|
| AO3401 P-MOSFET, source | 3.3 V |
| AO3401 drain | GPS VCC |
| AO3401 gate | GPIO7 (pad 7) |
| 100 kΩ resistor | gate to 3.3 V (keeps the GPS off while the S3 sleeps) |

If your GPS board has a V_BCKP / VBAT pin and no backup cell, wire it straight to 3.3 V (not through the MOSFET). It draws a few µA and lets the GPS get a fix again within seconds of waking.

## ATGM336H instead of the NEO-6M

Wire it the same way (TX to the S3's RX, RX to the S3's TX). Then in `idf.py menuconfig` set **GPS logger → GPS module → ATGM336H**, or add `CONFIG_GPS_MODEL_ATGM336H=y` to `sdkconfig.n16r8`.

## Power with 2x 18650

Put the two cells **in parallel** (about 3.0 to 4.2 V) into a low-quiescent 3.3 V LDO such as an HT7333, ME6211 or RT9080. Avoid the AMS1117: it wastes about 5 mA on its own, which would flatten the cells in a few weeks of sleep.

## Measuring sleep current (test build)

`sdkconfig.sleeptest` makes a build that does nothing except switch the GPS power and LED pin off and deep-sleep 10 s after power-on, so you can measure the S3 board on its own (GPS and accelerometer can be unplugged). Press BOOT to wake it; it sleeps again 10 s later. It builds into its own folder, so your normal build is untouched:

```
idf.py -B build_sleeptest -D SDKCONFIG=build_sleeptest/sdkconfig -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sleeptest" -p COM4 flash monitor
```

To go back: press BOOT (or reset), and during the 10 s countdown run `idf.py -p COM4 flash` for the normal build. If you miss the window, hold BOOT, tap RST, release BOOT, then flash.

Expected on battery with the meter in series: S3 + regulator about 0.05 to 0.2 mA, plus the red power LED (1 to 3 mA) and the WS2812 LED's idle draw (about 0.5 to 1 mA) if still fitted.
