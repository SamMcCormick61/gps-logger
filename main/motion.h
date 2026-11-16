#pragma once
#include <stdbool.h>
#include <stdint.h>

// LIS3DH or LIS3DSH accelerometer on I2C, used to notice when the bike moves.
// Its INT1 pin goes high on movement; the S3 can wake from deep sleep on it.
bool    motion_init(void);          // false if no accelerometer found (sleep is then disabled)
bool    motion_present(void);
int64_t motion_last_us(void);       // esp_timer time movement was last felt
bool    motion_arm_for_sleep(void); // clears the latched interrupt; false if moving right now
