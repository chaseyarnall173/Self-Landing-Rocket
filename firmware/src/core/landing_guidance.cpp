#include "landing_guidance.h"

#include <math.h>

#include "vec.h"

static constexpr int PLAN_SAMPLES = 40;
static constexpr int PLAN_CANDIDATES = 9;

float LandingGuidance::mass(float tb) const {
    float motor = (tb < 0.0f) ? v_.landing->total_kg : v_.landing->mass(tb);
    return v_.dry_mass_kg + v_.spent_ascent_kg + motor;
}

void LandingGuidance::window(float& t1, float& t2) {
    if (!have_window_) {
        // Same rule as the simulator, scanned on 400 points.
        const int N = 400;
        float tb_end = v_.landing->burn_time();
        bool found1 = false, found2 = false;
        t1_ = tb_end;
        t2_ = 0.0f;
        for (int i = 0; i < N; ++i) {
            float tb = 0.05f + (tb_end - 0.05f) * i / (N - 1);
            float thrust = v_.landing->thrust(tb);
            float w = mass(tb) * G0;
            if (!found1 && v_.throttle_min * thrust < v_.window_hop_margin * w) {
                t1_ = tb;
                found1 = true;
            }
            if (thrust > v_.window_end_thrust_ratio * w) {
                t2_ = tb;
                found2 = true;
            }
        }
        if (!found2 || t2_ < t1_) t2_ = t1_;
        have_window_ = true;
    }
    t1 = t1_;
    t2 = t2_;
}

bool LandingGuidance::plan(float h, float v, float rho, float tb_now, float t_land,
                           float cos_tilt, float cda, float thrust_scale, float& k0,
                           float& k1) const {
    const float tau = t_land - tb_now;
    if (tau < 0.05f) return false;
    const float vtd = -v_.v_td;
    float a0 = 0, a1 = 0, b0 = 0, b1 = 0, dv = 0, dh = 0;
    float prev[6] = {0, 0, 0, 0, 0, 0};
    const float ds = tau / (PLAN_SAMPLES - 1);
    for (int i = 0; i < PLAN_SAMPLES; ++i) {
        float s = ds * i;
        float tb = tb_now + s;
        float m = mass(tb);
        float a = thrust_scale * v_.landing->thrust(tb) * cos_tilt / m;
        float vl = v + (vtd - v) * s / tau;
        float d = 0.5f * rho * cda * vl * vl / m;
        float cur[6] = {a, s * a, (tau - s) * a, s * (tau - s) * a, d, (tau - s) * d};
        if (i > 0) {  // trapezoid
            a0 += 0.5f * ds * (cur[0] + prev[0]);
            a1 += 0.5f * ds * (cur[1] + prev[1]);
            b0 += 0.5f * ds * (cur[2] + prev[2]);
            b1 += 0.5f * ds * (cur[3] + prev[3]);
            dv += 0.5f * ds * (cur[4] + prev[4]);
            dh += 0.5f * ds * (cur[5] + prev[5]);
        }
        for (int j = 0; j < 6; ++j) prev[j] = cur[j];
    }
    const float rhs_v = vtd - v + G0 * tau - dv;
    const float rhs_h = -h - v * tau + 0.5f * G0 * tau * tau - dh;
    const float det = a0 * b1 - a1 * b0;
    if (fabsf(det) < 1e-9f) return false;
    k0 = (rhs_v * b1 - a1 * rhs_h) / det;
    k1 = (a0 * rhs_h - b0 * rhs_v) / det;
    return true;
}

ThrottlePlan LandingGuidance::best_plan(float h, float v, float rho, float tb_now,
                                        float cos_tilt, float cda, float thrust_scale) {
    float t1, t2;
    window(t1, t2);
    float lo = fmaxf(t1, tb_now + 0.15f);
    float hi = fmaxf(lo, t2);
    float pref = have_t_land_ ? t_land_pref_ : 0.5f * (t1 + t2);
    ThrottlePlan best;
    float best_cost = 1e30f;
    for (int i = 0; i < PLAN_CANDIDATES; ++i) {
        float tl = lo + (hi - lo) * i / (PLAN_CANDIDATES - 1);
        float k0, k1;
        if (!plan(h, v, rho, tb_now, tl, cos_tilt, cda, thrust_scale, k0, k1)) continue;
        float ke = k0 + k1 * (tl - tb_now);
        float viol = fmaxf(0.0f, v_.throttle_min - fminf(k0, ke)) +
                     fmaxf(0.0f, fmaxf(k0, ke) - 1.0f);
        float cost = 10.0f * viol + 0.05f * fabsf(tl - pref);
        if (cost < best_cost) {
            best_cost = cost;
            best.valid = true;
            best.t_land = tl;
            best.k0 = k0;
            best.k1 = k1;
            best.violation = viol;
        }
    }
    if (best.valid) {
        have_t_land_ = true;
        t_land_pref_ = best.t_land;
    }
    return best;
}

bool LandingGuidance::fire_now(float h, float v, float rho, float delay_s, float fire_k1,
                               bool keep_drogue) {
    // Coast through the expected ignition delay under the drogue.
    const float dt = 0.01f;
    const float m = mass(-1.0f);
    int steps = (int)lroundf(delay_s / dt);
    for (int i = 0; i < steps; ++i) {
        float a = -0.5f * rho * v_.cda_pre_m2 * v * fabsf(v) / m - G0;
        v += a * dt;
        h += v * dt;
    }
    if (h <= 0.0f) return true;
    float t1, t2;
    window(t1, t2);
    float k0, k1;
    float cda = keep_drogue ? v_.cda_pre_m2 : v_.cda_burn_m2;
    if (!plan(h, v, rho, 0.0f, 0.5f * (t1 + t2), 1.0f, cda, 1.0f, k0, k1)) return true;
    return k1 <= fire_k1;
}

float LandingGuidance::terminal_throttle(float h, float v, float rho, float tb,
                                         float cos_tilt, float cda, float thrust_scale,
                                         float kv) const {
    const float vtd = v_.v_td;
    const float m = mass(tb);
    float h_eff = fmaxf(h, 0.05f);
    float brake = (v < -vtd) ? fmaxf(0.0f, (v * v - vtd * vtd) / (2.0f * h_eff)) : 0.0f;
    float a_req = G0 + brake + ((brake > 0.0f) ? 0.0f : kv * (-vtd - v));
    a_req -= -0.5f * rho * cda * v * fabsf(v) / m;  // drag already helps
    float avail = thrust_scale * v_.landing->thrust(tb) * cos_tilt;
    float k = (avail > 1.0f) ? a_req * m / avail : 1.0f;
    return clampf(k, v_.throttle_min, 1.0f);
}
