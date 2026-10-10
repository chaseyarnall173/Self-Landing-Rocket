#include "sensors.h"

#include "board.h"

// ================================================================ BMI088
// Accelerometer registers
static constexpr uint8_t ACC_CHIP_ID = 0x00;   // expect 0x1E
static constexpr uint8_t ACC_DATA = 0x12;      // X_LSB..Z_MSB (6 bytes)
static constexpr uint8_t ACC_CONF = 0x40;
static constexpr uint8_t ACC_RANGE = 0x41;
static constexpr uint8_t ACC_PWR_CONF = 0x7C;
static constexpr uint8_t ACC_PWR_CTRL = 0x7D;
static constexpr uint8_t ACC_SOFTRESET = 0x7E;
// Gyroscope registers
static constexpr uint8_t GYR_CHIP_ID = 0x00;   // expect 0x0F
static constexpr uint8_t GYR_DATA = 0x02;      // X_LSB..Z_MSB (6 bytes)
static constexpr uint8_t GYR_RANGE = 0x0F;
static constexpr uint8_t GYR_BANDWIDTH = 0x10;
static constexpr uint8_t GYR_LPM1 = 0x11;
static constexpr uint8_t GYR_SOFTRESET = 0x14;

bool Bmi088::write(uint8_t addr, uint8_t reg, uint8_t val) {
    bus_->beginTransmission(addr);
    bus_->write(reg);
    bus_->write(val);
    return bus_->endTransmission() == 0;
}

bool Bmi088::read_bytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_->beginTransmission(addr);
    bus_->write(reg);
    if (bus_->endTransmission(false) != 0) return false;
    if (bus_->requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_->read();
    return true;
}

bool Bmi088::begin(TwoWire& bus, uint8_t acc_addr, uint8_t gyr_addr) {
    bus_ = &bus;
    acc_ = acc_addr;
    gyr_ = gyr_addr;
    uint8_t id = 0;
    // Gyro
    write(gyr_, GYR_SOFTRESET, 0xB6);
    delay(30);
    if (!read_bytes(gyr_, GYR_CHIP_ID, &id, 1) || id != 0x0F) return false;
    write(gyr_, GYR_RANGE, 0x00);       // +-2000 dps
    write(gyr_, GYR_BANDWIDTH, 0x02);   // ODR 1000 Hz, filter 116 Hz
    write(gyr_, GYR_LPM1, 0x00);        // normal power
    // Accel
    write(acc_, ACC_SOFTRESET, 0xB6);
    delay(2);
    if (!read_bytes(acc_, ACC_CHIP_ID, &id, 1) || id != 0x1E) return false;
    write(acc_, ACC_PWR_CONF, 0x00);    // active
    delay(2);
    write(acc_, ACC_PWR_CTRL, 0x04);    // accelerometer on
    delay(50);
    write(acc_, ACC_CONF, 0xAB);        // normal filter, ODR 800 Hz
    write(acc_, ACC_RANGE, 0x03);       // +-24 g (touchdown shocks)
    return true;
}

bool Bmi088::read(Vec3& a, Vec3& g) {
    uint8_t b[6];
    if (!read_bytes(acc_, ACC_DATA, b, 6)) return false;
    const float ka = 24.0f * G0 / 32768.0f;  // range 0x03 -> +-24 g
    a = Vec3((int16_t)(b[1] << 8 | b[0]) * ka, (int16_t)(b[3] << 8 | b[2]) * ka,
             (int16_t)(b[5] << 8 | b[4]) * ka);
    if (!read_bytes(gyr_, GYR_DATA, b, 6)) return false;
    const float kg = 2000.0f * DEG / 32768.0f;
    g = Vec3((int16_t)(b[1] << 8 | b[0]) * kg, (int16_t)(b[3] << 8 | b[2]) * kg,
             (int16_t)(b[5] << 8 | b[4]) * kg);
    return true;
}

// ================================================================ BMP390
static constexpr uint8_t BMP_CHIP_ID = 0x00;   // 0x60 (BMP390) or 0x50 (BMP388)
static constexpr uint8_t BMP_DATA = 0x04;      // press xlsb..temp msb (6 bytes)
static constexpr uint8_t BMP_PWR_CTRL = 0x1B;
static constexpr uint8_t BMP_OSR = 0x1C;
static constexpr uint8_t BMP_ODR = 0x1D;
static constexpr uint8_t BMP_CONFIG = 0x1F;
static constexpr uint8_t BMP_CALIB = 0x31;     // 21 bytes
static constexpr uint8_t BMP_CMD = 0x7E;

bool Bmp390::write(uint8_t reg, uint8_t val) {
    bus_->beginTransmission(addr_);
    bus_->write(reg);
    bus_->write(val);
    return bus_->endTransmission() == 0;
}

bool Bmp390::read_bytes(uint8_t reg, uint8_t* buf, uint8_t n) {
    bus_->beginTransmission(addr_);
    bus_->write(reg);
    if (bus_->endTransmission(false) != 0) return false;
    if (bus_->requestFrom(addr_, n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = bus_->read();
    return true;
}

bool Bmp390::begin(TwoWire& bus, uint8_t addr) {
    bus_ = &bus;
    addr_ = addr;
    write(BMP_CMD, 0xB6);  // soft reset
    delay(10);
    uint8_t id = 0;
    if (!read_bytes(BMP_CHIP_ID, &id, 1) || (id != 0x60 && id != 0x50)) return false;
    uint8_t c[21];
    if (!read_bytes(BMP_CALIB, c, 21)) return false;
    auto u16 = [&](int i) { return (uint16_t)(c[i + 1] << 8 | c[i]); };
    auto s16 = [&](int i) { return (int16_t)(c[i + 1] << 8 | c[i]); };
    auto s8 = [&](int i) { return (int8_t)c[i]; };
    t1_ = u16(0) / pow(2, -8);
    t2_ = u16(2) / pow(2, 30);
    t3_ = s8(4) / pow(2, 48);
    p1_ = (s16(5) - pow(2, 14)) / pow(2, 20);
    p2_ = (s16(7) - pow(2, 14)) / pow(2, 29);
    p3_ = s8(9) / pow(2, 32);
    p4_ = s8(10) / pow(2, 37);
    p5_ = u16(11) / pow(2, -3);
    p6_ = u16(13) / pow(2, 6);
    p7_ = s8(15) / pow(2, 8);
    p8_ = s8(16) / pow(2, 15);
    p9_ = s16(17) / pow(2, 48);
    p10_ = s8(19) / pow(2, 48);
    p11_ = s8(20) / pow(2, 65);
    write(BMP_OSR, 0x03);                // pressure x8, temperature x1 (fits 50 Hz)
    write(BMP_ODR, 0x02);                // 50 Hz
    write(BMP_CONFIG, 0x02 << 1);        // IIR coefficient 3
    write(BMP_PWR_CTRL, 0x33);           // press + temp enabled, normal mode
    return true;
}

bool Bmp390::read(float& pressure_pa, float& temp_c) {
    uint8_t b[6];
    if (!read_bytes(BMP_DATA, b, 6)) return false;
    double up = (uint32_t)b[2] << 16 | (uint32_t)b[1] << 8 | b[0];
    double ut = (uint32_t)b[5] << 16 | (uint32_t)b[4] << 8 | b[3];
    // Temperature compensation
    double pd1 = ut - t1_;
    double pd2 = pd1 * t2_;
    double t = pd2 + pd1 * pd1 * t3_;
    // Pressure compensation
    double o1 = p5_ + p6_ * t + p7_ * t * t + p8_ * t * t * t;
    double o2 = up * (p1_ + p2_ * t + p3_ * t * t + p4_ * t * t * t);
    double o3 = up * up * (p9_ + p10_ * t) + up * up * up * p11_;
    pressure_pa = (float)(o1 + o2 + o3);
    temp_c = (float)t;
    return pressure_pa > 30000.0f && pressure_pa < 110000.0f;
}

// ================================================================ TFmini-S
void TfMini::begin(HardwareSerial& port, uint32_t baud) {
    port_ = &port;
    port_->begin(baud);
}

bool TfMini::poll() {
    bool got = false;
    while (port_->available()) {
        uint8_t c = port_->read();
        if (idx_ < 2) {
            if (c == 0x59) frame_[idx_++] = c; else idx_ = 0;
            continue;
        }
        frame_[idx_++] = c;
        if (idx_ == 9) {
            idx_ = 0;
            uint8_t sum = 0;
            for (int i = 0; i < 8; ++i) sum += frame_[i];
            if (sum != frame_[8]) continue;
            uint16_t d = frame_[3] << 8 | frame_[2];
            uint16_t s = frame_[5] << 8 | frame_[4];
            strength_ = s;
            // Manual: strength < 100 or == 65535 means the reading is unreliable.
            if (s < 100 || s == 65535 || d == 0) {
                dist_m_ = -1;
            } else {
                dist_m_ = d * 0.01f;
                got = true;
            }
        }
    }
    return got;
}

Vec3 sensor_to_body(const Vec3& s) {
    const float v[3] = {s.x, s.y, s.z};
    float o[3];
    for (int r = 0; r < 3; ++r) {
        o[r] = 0;
        for (int c = 0; c < 3; ++c) o[r] += board::IMU_MAP[r][c] * v[c];
    }
    return {o[0], o[1], o[2]};
}
