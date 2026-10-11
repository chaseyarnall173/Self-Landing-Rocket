// Flight parameters. Every value here mirrors sim/rocketsim/guidance.py
// (GuidanceConfig) or sim/rocketsim/designs.py. Change both together and
// re-run the Monte Carlo before flying a change.
#pragma once

namespace params {

// ---------------------------------------------------------------- vehicle
constexpr float DRY_MASS_KG = 1.294f;          // WEIGH IT (everything but motors); CAD rev C estimate
constexpr float INERTIA_KGM2 = 0.241f;         // pitch/yaw, measure with a bifilar pendulum (CAD 0.24)
constexpr float GIMBAL_LEVER_M = 0.499f;       // gimbal pivot -> CG at landing
constexpr float ASCENT_OFFSET_M = +0.0165f;    // ascent motor axis offset (body +y)
constexpr float LANDING_OFFSET_M = -0.0165f;   // landing motor axis offset (body -y)
constexpr float GIMBAL_LIMIT_DEG = 7.0f;
constexpr float LEG_DROP_M = 0.22f;            // feet below the aft end, legs out (55 deg)
constexpr float CDA_PRE_M2 = 0.06627f;         // tail-first CdA, top airbrakes open (gen_tables)
constexpr float CDA_BURN_M2 = 0.00867f;        // tail-first CdA, airbrakes stowed
constexpr float CDA_LEGS_M2 = 0.00486f;        // extra CdA with legs deployed

// ---------------------------------------------------- landing-burn logic
constexpr float IGNITION_DELAY_NOMINAL_S = 0.30f;   // MEASURE on the static-fire stand
constexpr float THROTTLE_MIN = 0.25f;               // paddles closed: MEASURE on the stand
constexpr float TARGET_TOUCHDOWN_SPEED = 0.5f;      // m/s down
constexpr float FIRE_K1 = 0.0f;                     // fire when plan ramp slope <= this
constexpr float FIRE_WINDOW_FRAC = 0.80f;           // fire plan aims this far into the window
// Burn shape: strong braking that decays to ~weight at touchdown (the
// 'strong then simmer' profile). false = the older linear throttle ramp.
constexpr bool EXP_BURN_PROFILE = true;
constexpr float EXP_END_WEIGHT = 2.0f;              // cost on net accel at touchdown (wants ~hover)
constexpr float EXP_TERMINAL_ALT_M = 0.8f;
constexpr float PREDICTOR_GATE_ALT_M = 120.0f;
constexpr float WINDOW_HOP_MARGIN = 1.0f;
constexpr float WINDOW_END_THRUST_RATIO = 1.25f;
constexpr float REPLAN_DT_S = 0.02f;
constexpr float TERMINAL_ALT_M = 2.0f;
constexpr float TERMINAL_KV = 3.0f;
constexpr float LATERAL_KV = 1.5f;
constexpr float LATERAL_TILT_MAX_DEG = 12.0f;
constexpr float LATERAL_FADE_ALT_M = 2.5f;
constexpr float LEG_DEPLOY_ALT_M = 12.0f;
// Test flight F2 ("virtual ground", docs/09): the whole landing burn is
// flown against a ground this many metres above the real one, then the main
// chute deploys at the virtual touchdown. 0 = normal flight.
constexpr float VIRTUAL_GROUND_M = 0.0f;

// ------------------------------------------------------ safety envelope
constexpr float IGNITION_DETECT_TIMEOUT_S = 0.90f;
constexpr float ABORT_TILT_DEG = 35.0f;
constexpr float FIRE_MAX_TILT_DEG = 20.0f;         // never light the motor beyond this
constexpr float FIRE_MIN_ALT_M = 5.0f;
constexpr float FIRE_VZ_MIN = -35.0f;              // faster than this = airbrakes failed
constexpr float FIRE_VZ_MAX = -8.0f;
constexpr float ARM_MAX_TILT_DEG = 5.0f;           // must be vertical on the pad to arm
constexpr float LIFTOFF_ACCEL_G = 2.0f;
constexpr float LIFTOFF_HOLD_S = 0.05f;

// ------------------------------------------------- attitude controller
constexpr float ATT_WN = 12.0f;
constexpr float ATT_ZETA = 0.85f;
constexpr float ATT_KI = 4.0f;
constexpr float ASCENT_LATERAL_KV = 0.8f;
constexpr float ASCENT_TILT_MAX_DEG = 4.0f;

// ------------------------------------------------------------- loop rates
constexpr float IMU_DT_S = 0.001f;     // 1 kHz
constexpr float CONTROL_DT_S = 0.005f; // 200 Hz
constexpr float BARO_DT_S = 0.02f;     // 50 Hz
constexpr float LOG_DT_S = 0.01f;      // 100 Hz
constexpr float TELEM_DT_S = 0.2f;     // 5 Hz

}  // namespace params
