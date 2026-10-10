// High-rate flight log: a RAM ring buffer (never blocks the control loop)
// written to the Teensy 4.1 built-in microSD once the vehicle is safe.
#pragma once
#include <Arduino.h>

struct LogRecord {           // 64 bytes
    float t;
    float h, vz, vx, vy;
    float qw, qx, qy, qz;
    float f_axial;
    float gimbal_dy, gimbal_dz, throttle;
    float baro_h, lidar_m;
    uint8_t phase, flags;
    uint16_t pad;
};
static_assert(sizeof(LogRecord) == 64, "keep the log record compact");

class Logger {
public:
    void push(const LogRecord& r);
    void event(float t, const char* msg);   // also echoed to USB serial
    bool flush_to_sd();                      // call only when the vehicle is safe
    uint32_t count() const { return count_; }

private:
    uint32_t head_ = 0, count_ = 0;
    char events_[2048];
    uint32_t ev_len_ = 0;
};
