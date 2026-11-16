#pragma once
#include <stdbool.h>

// Wi-Fi + web page for downloading rides (GPX), deleting rides and
// uploading new firmware. Hold the button (GPIO0 / BOOT by default) for 2 s to
// turn Wi-Fi on or off. Saved home Wi-Fi is tried first (open
// http://gpslogger.local), otherwise the board starts its own hotspot.
// Logging pauses while Wi-Fi is on.
void webui_init(void);
bool webui_is_active(void);
const char *webui_ssid(void);     // hotspot name, e.g. "GPS-Logger-1A2B"
const char *webui_address(void);  // IP address while Wi-Fi is on
bool webui_is_connecting(void);  // trying saved home Wi-Fi, logging already paused
bool webui_on_home_wifi(void);    // true = joined saved network, false = own hotspot
