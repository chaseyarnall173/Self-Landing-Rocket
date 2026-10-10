// State estimation.
//
// AttitudeEstimator: gyro-integrated quaternion. Accelerometers are NOT a
// gravity reference in flight (they read thrust/drag, and 0 g in free fall),
// so the accelerometer is only used on the pad to level the initial attitude
// and to estimate the gyro bias.
//
// AltitudeKF: 3-state Kalman filter [h, v, accel_bias] driven by the
// world-frame vertical acceleration and corrected by the barometer and,
// close to the ground, the downward lidar.
#pragma once
#include "vec.h"

class AttitudeEstimator {
public:
    // Accumulate pad samples (vehicle still). Call ~1000 times before arming.
    void pad_sample(const Vec3& accel_mss, const Vec3& gyro_rads);
    // Level the attitude from the averaged accel and latch the gyro bias.
    bool pad_finalize();
    void update(const Vec3& gyro_rads, float dt);

    const Quat& q() const { return q_; }
    Vec3 rate() const { return rate_; }
    Vec3 axis_world() const { return q_.rotate(Vec3(1, 0, 0)); }  // body +x in world
    float tilt_rad() const;                                         // from vertical
    Vec3 bias() const { return bias_; }

private:
    Quat q_;
    Vec3 bias_;
    Vec3 rate_;
    Vec3 acc_sum_, gyro_sum_;
    int n_ = 0;
};

class AltitudeKF {
public:
    void reset(float h0 = 0.0f);
    // a_up: world-frame vertical specific force minus g (m/s^2).
    void predict(float a_up, float dt);
    void update_baro(float h_meas, float sigma_m);
    void update_range(float h_meas, float sigma_m);  // lidar (already tilt-corrected)
    void zero_velocity(float sigma_mps = 0.05f);     // pad / landed updates

    float h() const { return x_[0]; }
    float v() const { return x_[1]; }
    float bias() const { return x_[2]; }
    float sigma_h() const;

private:
    void update_pos(float z, float r);
    float x_[3] = {0, 0, 0};
    float P_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 0.1f}};
    float q_acc_ = 0.5f;     // accel noise (m/s^2)^2 * s
    float q_bias_ = 0.002f;  // bias random walk
};

// Barometric altitude above the pad from pressure (hypsometric, ISA).
float baro_altitude_m(float pressure_pa, float ground_pressure_pa, float ground_temp_c);
