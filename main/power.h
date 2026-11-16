#pragma once
#include <stdint.h>
#include <stddef.h>

// Deep sleep: after a set number of seconds without movement (and Wi-Fi off)
// the ride is closed, the GPS is switched off and the S3 deep-sleeps. Movement
// (accelerometer INT1) or pressing BOOT wakes it, and it starts up as from power-on.
// The delay is set on the web page (saved in NVS); 0 = never sleep.
void power_init(void);        // notes why we woke; call early
void power_start(void);       // writes the start-up event, starts the idle watcher

uint32_t power_get_sleep_s(void);
void     power_set_sleep_s(uint32_t s);   // saved to flash, takes effect at once

// Event log on flash (/logs/events.txt): start-up / wake / sleep lines with UTC time.
void power_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define POWER_EVENTS_PATH     "/logs/events.txt"
#define POWER_EVENTS_OLD_PATH "/logs/events.old"
