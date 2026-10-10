// Flight state machine + mission logic (hardware independent).
//
//   PAD_SAFE -> PAD_ARMED -> BOOST -> COAST -> DESCENT -> LANDING_BURN
//                                                  |            |
//                                                  v            v
//                                             ABORT_CHUTE   TOUCHDOWN -> LANDED_SAFE
//
// The top airbrakes open at apogee (COAST -> DESCENT); the engine does the
// rest. The main chute only deploys on an abort.
// The HAL feeds FsmInputs every control cycle (200 Hz) and applies
// FsmOutputs. Nothing in here touches hardware, so the same code runs in the
// native unit tests.
#pragma once
#include "estimator.h"
#include "landing_guidance.h"
#include "motor_curve.h"
#include "tvc_controller.h"
#include "vec.h"

enum class Phase : uint8_t {
    PAD_SAFE = 0,
    PAD_ARMED,
    BOOST,
    COAST,
    DESCENT,
    LANDING_BURN,
    TOUCHDOWN,
    LANDED_SAFE,
    ABORT_CHUTE,
};

const char* phase_name(Phase p);

struct FsmInputs {
    float t = 0;               // s since boot
    Quat q;                    // body -> world
    Vec3 rate;                 // body rates, rad/s (bias removed)
    float tilt = 0;            // rad from vertical
    float h_aft = 0;           // KF altitude of the aft end above the pad (m)
    float vz = 0;              // KF vertical velocity (m/s, up +)
    float vx = 0, vy = 0;      // horizontal velocity (GPS-aided), world frame
    float f_axial = 0;         // body +x specific force (m/s^2)
    float rho = 1.225f;        // air density
    bool pad_calibrated = false;
    bool arm_switch = false;   // physical key / remove-before-flight in the arm position
    bool continuity_a = false; // landing igniter A
    bool continuity_b = false; // landing igniter B
    float vbat = 0;
};

struct FsmOutputs {
    Phase phase = Phase::PAD_SAFE;
    float gimbal_dy = 0, gimbal_dz = 0;  // thrust deflection (rad) toward body +y/+z
    float throttle = 1.0f;               // exhaust-paddle throttle command 0..1
    bool pyro_enable = false;            // master software enable (hardware key is separate)
    bool fire_landing = false;           // both landing igniters
    bool airbrakes_deploy = false;       // top airbrake petals (latch released at apogee)
    bool legs_deploy = false;
    bool chute_deploy = false;
    float h_feet = 0;
    ThrottlePlan plan;
};

class FlightFsm {
public:
    FlightFsm(const MotorCurve& ascent, const MotorCurve& landing);
    FsmOutputs step(const FsmInputs& in, float dt);
    Phase phase() const { return out_.phase; }
    float t_liftoff() const { return t_liftoff_; }
    float thrust_scale() const { return thrust_scale_; }
    // Event log hook: called with (time, message) on every transition.
    void (*on_event)(float t, const char* msg) = nullptr;

private:
    void set_phase(float t, Phase p);
    void event(float t, const char* msg);
    float landing_burn_clock(float t) const { return t - t_ign_ + 0.02f; }
    void do_landing_guidance(const FsmInputs& in, float dt);
    Vec3 up_command(float phi_x, float phi_y) const;

    const MotorCurve& ascent_;
    const MotorCurve& landing_;
    LandingGuidance guidance_;
    TvcController tvc_;
    FsmOutputs out_;
    float t_phase_ = 0, t_liftoff_ = -1, t_fire_ = -1, t_ign_ = -1, t_touch_ = -1;
    float f_at_fire_ = 0, max_h_ = 0, thrust_scale_ = 1.0f, next_replan_ = 0;
    float throttle_ = 1.0f, cda_ = 0;
    int hold_count_ = 0;
    bool airbrakes_out_ = false, legs_out_ = false, terminal_ = false;
};
