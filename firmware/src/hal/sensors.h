// Minimal register-level drivers (I2C/UART) for the flight sensors.
// Register addresses are from the Bosch BMI088 / BMP390 datasheets and the
// Benewake TFmini-S manual; verify against the datasheet revision you have.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "../core/vec.h"

class Bmi088 {
public:
    bool begin(TwoWire& bus, uint8_t acc_addr, uint8_t gyr_addr);
    // Reads both sensors; returns false on an I2C error.
    bool read(Vec3& accel_mss, Vec3& gyro_rads);  // SENSOR axes

private:
    bool write(uint8_t addr, uint8_t reg, uint8_t val);
    bool read_bytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);
    TwoWire* bus_ = nullptr;
    uint8_t acc_ = 0, gyr_ = 0;
};

class Bmp390 {
public:
    bool begin(TwoWire& bus, uint8_t addr);  // normal mode, 50 Hz, IIR 3
    bool read(float& pressure_pa, float& temp_c);  // non-blocking: latest sample

private:
    bool write(uint8_t reg, uint8_t val);
    bool read_bytes(uint8_t reg, uint8_t* buf, uint8_t n);
    TwoWire* bus_ = nullptr;
    uint8_t addr_ = 0;
    // Calibration coefficients (datasheet section 9.1, floating point).
    double t1_, t2_, t3_, p1_, p2_, p3_, p4_, p5_, p6_, p7_, p8_, p9_, p10_, p11_;
};

class TfMini {
public:
    void begin(HardwareSerial& port, uint32_t baud);
    // Parse any pending bytes; returns true when a new valid frame arrived.
    bool poll();
    float distance_m() const { return dist_m_; }
    uint16_t strength() const { return strength_; }

private:
    HardwareSerial* port_ = nullptr;
    uint8_t frame_[9];
    uint8_t idx_ = 0;
    float dist_m_ = -1;
    uint16_t strength_ = 0;
};

// Map a sensor-frame vector into the body frame using board::IMU_MAP.
Vec3 sensor_to_body(const Vec3& s);
