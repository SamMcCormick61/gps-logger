#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"

// Pins are set per board in sdkconfig.defaults / sdkconfig.n16r8
// (menuconfig: "GPS logger"). S3 TX -> GPS RX, S3 RX <- GPS TX.
// Works with a NEO-6M or an ATGM336H (CONFIG_GPS_MODEL_*); both talk NMEA at 9600 baud.
#define GPS_UART_NUM   UART_NUM_1
#define GPS_TX_PIN     CONFIG_GPS_TX_GPIO
#define GPS_RX_PIN     CONFIG_GPS_RX_GPIO
#define GPS_BAUD       9600

typedef struct {
    bool     valid;        // RMC status 'A' (active fix)
    int      fix_quality;  // GGA: 0 none, 1 GPS, 2 DGPS
    int      sats;         // satellites used
    int      sats_in_view; // GSV: satellites the receiver knows are overhead
    int      sats_heard;   // GSV: of those, how many have a signal (SNR > 0)
    int      best_snr;     // GSV: strongest signal, dB-Hz (>30 is good)
    double   lat;          // degrees, +N
    double   lon;          // degrees, +E
    float    alt_m;        // metres above MSL
    float    speed_kmh;
    float    course_deg;
    float    hdop;
    int      year, month, day;
    int      hour, minute;
    float    second;       // UTC
    int64_t  updated_us;   // esp_timer time of last valid RMC
    uint32_t fix_count;    // increments on every valid RMC
} gps_fix_t;

void gps_init(void);
void gps_configure(void);           // 2 Hz updates, drop unused NMEA sentences
void gps_power(bool on);           // switches the GPS supply if CONFIG_GPS_POWER_GPIO is wired
void gps_get_fix(gps_fix_t *out);  // thread-safe snapshot
uint32_t gps_sentence_count(void); // valid NMEA sentences seen so far
