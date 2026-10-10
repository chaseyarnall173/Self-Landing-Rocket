#include "estimator.h"

#include <math.h>

// ------------------------------------------------------------- attitude
void AttitudeEstimator::pad_sample(const Vec3& a, const Vec3& g) {
    acc_sum_ = acc_sum_ + a;
    gyro_sum_ = gyro_sum_ + g;
    ++n_;
}

bool AttitudeEstimator::pad_finalize() {
    if (n_ < 100) return false;
    Vec3 a = acc_sum_ * (1.0f / n_);
    bias_ = gyro_sum_ * (1.0f / n_);
    // On the pad the accelerometer reads +1 g along "up". Rotate body so that
    // the measured up direction maps onto world +z.
    Vec3 up_body = a.normalized();
    q_ = Quat::between(up_body, Vec3(0, 0, 1));
    acc_sum_ = gyro_sum_ = Vec3();
    n_ = 0;
    return true;
}

void AttitudeEstimator::update(const Vec3& gyro, float dt) {
    rate_ = gyro - bias_;
    // q_{k+1} = q_k (x) exp(0.5 * w * dt): body-frame rates.
    q_ = q_ * Quat::from_rotvec(rate_ * dt);
    q_.normalize();
}

float AttitudeEstimator::tilt_rad() const {
    float c = clampf(axis_world().z, -1.0f, 1.0f);
    return acosf(c);
}

// ---------------------------------------------------------- altitude KF
void AltitudeKF::reset(float h0) {
    x_[0] = h0;
    x_[1] = 0;
    x_[2] = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] = 0;
    P_[0][0] = 1.0f;
    P_[1][1] = 0.1f;
    P_[2][2] = 0.05f;
}

void AltitudeKF::predict(float a_up, float dt) {
    // x = F x + B u with F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
    const float a = a_up - x_[2];
    x_[0] += x_[1] * dt + 0.5f * a * dt * dt;
    x_[1] += a * dt;
    const float F[3][3] = {{1, dt, -0.5f * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
    float FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            FP[i][j] = 0;
            for (int k = 0; k < 3; ++k) FP[i][j] += F[i][k] * P_[k][j];
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            Pn[i][j] = 0;
            for (int k = 0; k < 3; ++k) Pn[i][j] += FP[i][k] * F[j][k];
        }
    // Process noise from white acceleration noise + bias random walk.
    const float q = q_acc_, dt2 = dt * dt, dt3 = dt2 * dt;
    Pn[0][0] += q * dt3 / 3.0f;
    Pn[0][1] += q * dt2 / 2.0f;
    Pn[1][0] += q * dt2 / 2.0f;
    Pn[1][1] += q * dt;
    Pn[2][2] += q_bias_ * dt;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] = Pn[i][j];
}

void AltitudeKF::update_pos(float z, float r) {
    // H = [1 0 0]
    const float y = z - x_[0];
    const float s = P_[0][0] + r;
    if (s <= 0) return;
    // Reject gross outliers (e.g. baro pressure spike from motor plume).
    if (y * y > 25.0f * s) return;
    float K[3] = {P_[0][0] / s, P_[1][0] / s, P_[2][0] / s};
    for (int i = 0; i < 3; ++i) x_[i] += K[i] * y;
    float P0[3] = {P_[0][0], P_[0][1], P_[0][2]};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] -= K[i] * P0[j];
}

void AltitudeKF::update_baro(float h, float sigma) { update_pos(h, sigma * sigma); }
void AltitudeKF::update_range(float h, float sigma) { update_pos(h, sigma * sigma); }

void AltitudeKF::zero_velocity(float sigma) {
    // H = [0 1 0]
    const float r = sigma * sigma;
    const float y = -x_[1];
    const float s = P_[1][1] + r;
    float K[3] = {P_[0][1] / s, P_[1][1] / s, P_[2][1] / s};
    for (int i = 0; i < 3; ++i) x_[i] += K[i] * y;
    float P1[3] = {P_[1][0], P_[1][1], P_[1][2]};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] -= K[i] * P1[j];
}

float AltitudeKF::sigma_h() const { return sqrtf(P_[0][0] > 0 ? P_[0][0] : 0); }

float baro_altitude_m(float p, float p0, float t0_c) {
    // Hypsometric equation with the ISA lapse rate.
    const float T0 = t0_c + 273.15f;
    return (T0 / 0.0065f) * (1.0f - powf(p / p0, 0.190263f));
}
