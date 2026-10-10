#include "logger.h"

#include <SD.h>

// 4500 records x 64 B = 288 KB in DMAMEM (RAM2): last 45 s at 100 Hz.
static constexpr uint32_t N_REC = 4500;
DMAMEM static LogRecord g_ring[N_REC];

void Logger::push(const LogRecord& r) {
    g_ring[head_] = r;
    head_ = (head_ + 1) % N_REC;
    if (count_ < N_REC) ++count_;
}

void Logger::event(float t, const char* msg) {
    int n = snprintf(events_ + ev_len_, sizeof(events_) - ev_len_, "%.3f,%s\n", t, msg);
    if (n > 0 && ev_len_ + n < sizeof(events_)) ev_len_ += n;
    Serial.printf("[%8.3f] %s\n", t, msg);
}

bool Logger::flush_to_sd() {
    if (!SD.begin(BUILTIN_SDCARD)) return false;
    char name[16];
    for (int i = 0; i < 1000; ++i) {
        snprintf(name, sizeof name, "FLT%03d.CSV", i);
        if (!SD.exists(name)) break;
    }
    File f = SD.open(name, FILE_WRITE);
    if (!f) return false;
    f.println("t,h,vz,vx,vy,qw,qx,qy,qz,f_axial,gimbal_dy,gimbal_dz,throttle,baro_h,lidar_m,phase,flags");
    uint32_t start = (head_ + N_REC - count_) % N_REC;
    for (uint32_t i = 0; i < count_; ++i) {
        const LogRecord& r = g_ring[(start + i) % N_REC];
        f.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.3f,%.5f,%.5f,%.3f,%.3f,%.3f,%u,%u\n",
                 r.t, r.h, r.vz, r.vx, r.vy, r.qw, r.qx, r.qy, r.qz, r.f_axial, r.gimbal_dy,
                 r.gimbal_dz, r.throttle, r.baro_h, r.lidar_m, r.phase, r.flags);
    }
    f.println("# events");
    f.write((const uint8_t*)events_, ev_len_);
    f.close();
    return true;
}
