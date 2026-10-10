// Landing-burn guidance: ignition decision + exhaust-paddle throttle plan.
// C++ port of sim/rocketsim/guidance.py (FlightComputer.landing_window,
// plan, best_plan, fire_now and the terminal braking law). See
// docs/06-gnc.md for the derivation.
#pragma once
#include "motor_curve.h"

struct LandingVehicle {
    const MotorCurve* landing;   // landing motor curve
    float dry_mass_kg;           // dry vehicle (no motors)
    float spent_ascent_kg;       // casing of the burnt-out ascent motor
    float cda_pre_m2;            // CdA with the drogue out
    float cda_burn_m2;           // CdA without drogue
    float throttle_min;          // paddle authority (fraction of thrust left when closed)
    float v_td;                  // target touchdown speed (positive number, m/s)
    float window_hop_margin;
    float window_end_thrust_ratio;
};

struct ThrottlePlan {
    bool valid = false;
    float t_land = 0;  // burn clock at touchdown
    float k0 = 0;      // throttle now
    float k1 = 0;      // throttle slope (1/s)
    float violation = 0;
};

class LandingGuidance {
public:
    explicit LandingGuidance(const LandingVehicle& v) : v_(v) {}

    float mass(float tb) const;            // vehicle mass at burn clock tb (<0 = unlit)
    void window(float& t1, float& t2);     // allowed touchdown burn-clock window

    // Throttle ramp k(s)=k0+k1*s reaching h=0, v=-v_td at burn clock t_land.
    bool plan(float h, float v, float rho, float tb_now, float t_land, float cos_tilt,
              float cda, float thrust_scale, float& k0, float& k1) const;

    // Search the window for the most feasible plan (hysteresis on t_land).
    ThrottlePlan best_plan(float h, float v, float rho, float tb_now, float cos_tilt,
                           float cda, float thrust_scale);

    // True when the ignition command should be sent now (motor unlit).
    bool fire_now(float h, float v, float rho, float delay_s, float fire_k1, bool keep_drogue);

    // Terminal braking law (below TERMINAL_ALT): throttle for constant decel
    // to v_td at h=0.
    float terminal_throttle(float h, float v, float rho, float tb, float cos_tilt, float cda,
                            float thrust_scale, float kv) const;

    void reset() { have_t_land_ = false; }

private:
    LandingVehicle v_;
    bool have_window_ = false;
    float t1_ = 0, t2_ = 0;
    bool have_t_land_ = false;
    float t_land_pref_ = 0;
};
