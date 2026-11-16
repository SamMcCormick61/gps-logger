#pragma once
#include <stdbool.h>
#include <stdint.h>

#define LOGGER_BASE_PATH   "/logs"
#define LOGGER_MIN_DIST_M  10.0    // log a point every 10 m of travel
#define LOGGER_MIN_SATS    4       // need a 3D fix
#define LOGGER_MAX_HDOP    5.0f
#define LOGGER_FLUSH_MS    10000   // worst case data loss on power cut

// One logged point, stored as-is in the ride file (16 bytes, little endian).
// GPX is generated from these when a ride is downloaded.
typedef struct __attribute__((packed)) {
    uint32_t time;        // UTC, seconds since 1970
    int32_t  lat_e7;      // degrees * 1e7
    int32_t  lon_e7;      // degrees * 1e7
    int16_t  ele_m;       // metres above sea level
    uint16_t speed_cmps;  // cm/s
} log_point_t;

// Ride file layout: 8-byte header, then log_point_t records.
#define LOGGER_FILE_MAGIC  "GPSL"
#define LOGGER_FILE_VER    1

typedef struct {
    bool     mounted;
    bool     recording;     // a ride file is open
    bool     full;          // stopped because flash is full
    char     ride_name[32]; // e.g. "20261007-153253.trk"
    uint32_t points;        // points in the current ride
    uint32_t flushes;       // times points were saved to flash
    double   distance_m;    // distance covered in the current ride
    uint32_t fs_total_kb;
    uint32_t fs_used_kb;
} logger_status_t;

void logger_init(void);    // mount flash filesystem, list rides, start logging task
void logger_get_status(logger_status_t *out);

// Pausing ends the current ride (it's saved and closed) so its file can be
// downloaded safely. After resuming, the next good fix starts a new ride.
void logger_set_paused(bool paused);
bool logger_is_paused(void);
