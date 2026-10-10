#include "motor_curve.h"

// The curve implicitly starts at (0, 0) like a RASP file.
static inline void seg(const MotorCurve& m, int i, float& t0, float& f0, float& t1, float& f1) {
    t0 = (i == 0) ? 0.0f : m.t[i - 1];
    f0 = (i == 0) ? 0.0f : m.f[i - 1];
    t1 = m.t[i];
    f1 = m.f[i];
}

float MotorCurve::thrust(float tb) const {
    if (tb <= 0.0f || tb >= burn_time()) return 0.0f;
    for (int i = 0; i < n; ++i) {
        float t0, f0, t1, f1;
        seg(*this, i, t0, f0, t1, f1);
        if (tb <= t1) {
            float u = (t1 > t0) ? (tb - t0) / (t1 - t0) : 1.0f;
            return f0 + u * (f1 - f0);
        }
    }
    return 0.0f;
}

float MotorCurve::impulse(float tb) const {
    if (tb <= 0.0f) return 0.0f;
    float acc = 0.0f;
    for (int i = 0; i < n; ++i) {
        float t0, f0, t1, f1;
        seg(*this, i, t0, f0, t1, f1);
        if (tb >= t1) {
            acc += 0.5f * (f0 + f1) * (t1 - t0);
        } else {
            float ft = thrust(tb);
            acc += 0.5f * (f0 + ft) * (tb - t0);
            return acc;
        }
    }
    return acc;
}

float MotorCurve::mass(float tb) const {
    float total = impulse(burn_time());
    float burnt = total > 0.0f ? impulse(tb) / total : 0.0f;
    if (burnt > 1.0f) burnt = 1.0f;
    return total_kg - burnt * propellant_kg;
}
