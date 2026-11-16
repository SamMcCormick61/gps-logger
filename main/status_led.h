#pragma once

// Status LED patterns (WS2812 RGB LED, pin set by CONFIG_STATUS_LED_GPIO, -1 = none):
//   red blink every 2 s      no GPS fix yet / fix too weak to log
//   green flash              a point was logged
//   green blink every 3 s    recording, but standing still
//   blue flash               points saved to flash
//   fast red blink           flash full, not recording
//   cyan blink every 1 s     Wi-Fi hotspot on (logging paused)
void status_led_init(void);  // runs a red-green-blue test, then starts the LED task
void status_led_off(void);   // blanks the LED for deep sleep (a WS2812 keeps its colour otherwise)
