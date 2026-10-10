#include "flight_fsm.h"

#include <math.h>
#include <stdio.h>

#include "params.h"

using namespace params;

const char* phase_name(Phase p) {
    switch (p) {
        case Phase::PAD_SAFE: return "PAD_SAFE";
        case Phase::PAD_ARMED: return "PAD_ARMED";
        case Phase::BOOST: return "BOOST";
        case Phase::COAST: return "COAST";
        case Phase::DESCENT: return "DESCENT";
        case Phase::LANDING_BURN: return "LANDING_BURN";
        case Phase::TOUCHDOWN: return "TOUCHDOWN";
        case Phase::LANDED_SAFE: return "LANDED_SAFE";
        case Phase::ABORT_CHUTE: return "ABORT_CHUTE";
    }
    return "?";
}

static LandingVehicle make_vehicle(const MotorCurve& ascent, const MotorCurve& landing) {
    LandingVehicle v;
    v.landing = &landing;
    v.dry_mass_kg = DRY_MASS_KG;
    v.spent_ascent_kg = ascent.casing_kg();
    v.cda_pre_m2 = CDA_PRE_M2;
    v.cda_burn_m2 = CDA_BURN_M2;
    v.throttle_min = THROTTLE_MIN;
    v.v_td = TARGET_TOUCHDOWN_SPEED;
    v.window_hop_margin = WINDOW_HOP_MARGIN;
    v.window_end_thrust_ratio = WINDOW_END_THRUST_RATIO;
    return v;
}

FlightFsm::FlightFsm(const MotorCurve& ascent, const MotorCurve& landing)
    : ascent_(ascent),
      landing_(landing),
      guidance_(make_vehicle(ascent, landing)),
      tvc_(TvcGains{ATT_WN, ATT_ZETA, ATT_KI, GIMBAL_LIMIT_DEG * DEG, INERTIA_KGM2,
                    GIMBAL_LEVER_M}) {}

void FlightFsm::event(float t, const char* msg) {
    if (on_event) on_event(t, msg);
}

void FlightFsm::set_phase(float t, Phase p) {
    out_.phase = p;
    t_phase_ = t;
    hold_count_ = 0;
    event(t, phase_name(p));
}

// Desired thrust direction in the world frame from small lean angles about
// the world axes (phi_x leans the thrust toward +x, phi_y toward +y).
Vec3 FlightFsm::up_command(float phi_x, float phi_y) const {
    return Vec3(tanf(phi_x), tanf(phi_y), 1.0f).normalized();
}

FsmOutputs FlightFsm::step(const FsmInputs& in, float dt) {
    out_.fire_landing = false;
    out_.gimbal_dy = out_.gimbal_dz = 0.0f;
    out_.h_feet = in.h_aft - (legs_out_ ? LEG_DROP_M : 0.0f) - VIRTUAL_GROUND_M;
    if (in.h_aft > max_h_) max_h_ = in.h_aft;
    const float h_feet = out_.h_feet;

    switch (out_.phase) {
        case Phase::PAD_SAFE: {
            out_.pyro_enable = false;
            if (in.pad_calibrated && in.arm_switch && in.tilt < ARM_MAX_TILT_DEG * DEG &&
                in.continuity_a && in.continuity_b) {
                set_phase(in.t, Phase::PAD_ARMED);
            }
            break;
        }
        case Phase::PAD_ARMED: {
            out_.pyro_enable = true;
            if (!in.arm_switch) {
                set_phase(in.t, Phase::PAD_SAFE);
                break;
            }
            if (in.f_axial > LIFTOFF_ACCEL_G * G0) {
                if (++hold_count_ * CONTROL_DT_S >= LIFTOFF_HOLD_S) {
                    t_liftoff_ = in.t - LIFTOFF_HOLD_S - 0.05f;  // motor lit a bit earlier
                    tvc_.reset();
                    set_phase(in.t, Phase::BOOST);
                }
            } else {
                hold_count_ = 0;
            }
            break;
        }
        case Phase::BOOST: {
            float thrust = ascent_.thrust(in.t - t_liftoff_);
            float lim = ASCENT_TILT_MAX_DEG * DEG;
            float px = clampf(-ASCENT_LATERAL_KV * in.vx / G0, -lim, lim);
            float py = clampf(-ASCENT_LATERAL_KV * in.vy / G0, -lim, lim);
            Vec3 trim_w = in.q.rotate(Vec3(0, sinf(tvc_.trim(ASCENT_OFFSET_M)), 0));
            px -= trim_w.x;
            py -= trim_w.y;
            GimbalCmd g = tvc_.update(in.q, in.rate, up_command(px, py), thrust,
                                      ASCENT_OFFSET_M, dt);
            out_.gimbal_dy = g.dy;
            out_.gimbal_dz = g.dz;
            if (in.t - t_phase_ > 0.3f && in.f_axial < 0.0f) {
                if (++hold_count_ >= 5) {
                    tvc_.reset();
                    set_phase(in.t, Phase::COAST);
                }
            } else {
                hold_count_ = 0;
            }
            break;
        }
        case Phase::COAST: {
            if (in.vz < -0.5f) {
                if (++hold_count_ >= 10) {
                    drogue_out_ = true;
                    out_.drogue_deploy = true;
                    event(in.t, "DROGUE");
                    set_phase(in.t, Phase::DESCENT);
                }
            } else {
                hold_count_ = 0;
            }
            break;
        }
        case Phase::DESCENT: {
            if (in.tilt > ABORT_TILT_DEG * DEG && in.h_aft < 0.7f * max_h_) {
                event(in.t, "ABORT: tilt");
                set_phase(in.t, Phase::ABORT_CHUTE);
                break;
            }
            bool envelope = in.tilt < FIRE_MAX_TILT_DEG * DEG && h_feet > FIRE_MIN_ALT_M &&
                            in.vz > FIRE_VZ_MIN && in.vz < FIRE_VZ_MAX;
            if (h_feet < PREDICTOR_GATE_ALT_M && envelope &&
                guidance_.fire_now(h_feet, in.vz, in.rho, IGNITION_DELAY_NOMINAL_S, FIRE_K1,
                                   true)) {
                t_fire_ = in.t;
                f_at_fire_ = in.f_axial;
                out_.fire_landing = true;
                char buf[64];
                snprintf(buf, sizeof buf, "FIRE h=%.1f vz=%.1f", h_feet, in.vz);
                event(in.t, buf);
                set_phase(in.t, Phase::LANDING_BURN);
            } else if (h_feet < 15.0f && !envelope) {
                // Below the last useful ignition altitude and outside the safe
                // envelope (e.g. drogue failed, too fast, tilted): don't light it.
                event(in.t, "ABORT: envelope");
                set_phase(in.t, Phase::ABORT_CHUTE);
            }
            break;
        }
        case Phase::LANDING_BURN: {
            out_.fire_landing = true;  // HAL limits the igniter pulse length
            if (t_ign_ < 0.0f) {
                throttle_ = 1.0f;
                if (in.f_axial > f_at_fire_ + 0.8f * G0) {
                    t_ign_ = in.t;
                    tvc_.reset();
                    guidance_.reset();
                    event(in.t, "IGNITION");
                } else if (in.t - t_fire_ > IGNITION_DETECT_TIMEOUT_S) {
                    event(in.t, "ABORT: no ignition");
                    set_phase(in.t, Phase::ABORT_CHUTE);
                    break;
                }
            }
            if (t_ign_ >= 0.0f) do_landing_guidance(in, dt);
            if (h_feet <= LEG_DEPLOY_ALT_M && !legs_out_) {
                legs_out_ = true;
                event(in.t, "LEGS");
            }
            break;
        }
        case Phase::TOUCHDOWN: {
            throttle_ = THROTTLE_MIN;
            float tb = landing_burn_clock(in.t);
            float thrust = THROTTLE_MIN * thrust_scale_ * landing_.thrust(tb);
            GimbalCmd g = tvc_.update(in.q, in.rate, Vec3(0, 0, 1), thrust, LANDING_OFFSET_M, dt);
            out_.gimbal_dy = g.dy;
            out_.gimbal_dz = g.dz;
            if (tb > landing_.burn_time() + 2.0f) set_phase(in.t, Phase::LANDED_SAFE);
            break;
        }
        case Phase::LANDED_SAFE:
            out_.pyro_enable = false;
            break;
        case Phase::ABORT_CHUTE: {
            out_.chute_deploy = true;
            // A late ("hang fire") landing motor under the chute: keep it
            // pointing nozzle-down so it cannot fly sideways into the crowd.
            if (t_fire_ >= 0.0f && in.f_axial > f_at_fire_ + 0.8f * G0) {
                GimbalCmd g = tvc_.update(in.q, in.rate, Vec3(0, 0, 1),
                                          0.5f * landing_.total_impulse() / landing_.burn_time(),
                                          LANDING_OFFSET_M, dt);
                out_.gimbal_dy = g.dy;
                out_.gimbal_dz = g.dz;
            }
            if (in.t - t_phase_ > 30.0f) out_.pyro_enable = false;
            break;
        }
    }
    out_.throttle = throttle_;
    out_.legs_deploy = legs_out_;
    return out_;
}

void FlightFsm::do_landing_guidance(const FsmInputs& in, float dt) {
    const float tb = landing_burn_clock(in.t);
    const float m = guidance_.mass(tb);
    cda_ = (drogue_out_ ? CDA_PRE_M2 : CDA_BURN_M2) + (legs_out_ ? CDA_LEGS_M2 : 0.0f);
    const float nominal = landing_.thrust(tb);

    // Learn the real thrust level from the accelerometer (drag removed).
    float measured = in.f_axial * m - 0.5f * in.rho * cda_ * in.vz * in.vz;
    if (measured < 0) measured = 0;
    if (nominal > 5.0f && tb > 0.15f) {
        float ratio = measured / (fmaxf(throttle_, THROTTLE_MIN) * nominal);
        ratio = clampf(ratio, 0.7f, 1.3f);
        thrust_scale_ += 0.05f * (ratio - thrust_scale_);
    }

    if (out_.h_feet <= 0.05f) {
        throttle_ = THROTTLE_MIN;
        t_touch_ = in.t;
        if (VIRTUAL_GROUND_M > 0.0f) {
            // Test flight: "landed" on the virtual ground - now recover on the main.
            char buf[48];
            snprintf(buf, sizeof buf, "VIRTUAL TOUCHDOWN vz=%.2f", in.vz);
            event(in.t, buf);
            set_phase(in.t, Phase::ABORT_CHUTE);
            return;
        }
        if (drogue_out_) {
            out_.drogue_release = true;  // cut it so ground wind can't drag us over
            drogue_out_ = false;
            event(in.t, "DROGUE RELEASE");
        }
        set_phase(in.t, Phase::TOUCHDOWN);
        return;
    }

    const float cos_t = fmaxf(0.7f, cosf(in.tilt));
    if (out_.h_feet > TERMINAL_ALT_M && !terminal_) {
        if (in.t >= next_replan_) {
            next_replan_ = in.t + REPLAN_DT_S;
            ThrottlePlan p = guidance_.best_plan(out_.h_feet, in.vz, in.rho, tb, cos_t, cda_,
                                                 thrust_scale_);
            if (!p.valid) {
                terminal_ = true;
            } else {
                out_.plan = p;
                throttle_ = clampf(p.k0, THROTTLE_MIN, 1.0f);
            }
        }
    } else {
        throttle_ = guidance_.terminal_throttle(out_.h_feet, in.vz, in.rho, tb, cos_t, cda_,
                                                thrust_scale_, TERMINAL_KV);
    }

    // Lateral: lean the THRUST to null horizontal velocity, fading to upright
    // near the ground. The body leans opposite to the offset trim so the trim
    // deflection doesn't itself push us sideways.
    const float lim = LATERAL_TILT_MAX_DEG * DEG;
    const float fade = clampf((out_.h_feet - 0.3f) / LATERAL_FADE_ALT_M, 0.0f, 1.0f);
    float px = clampf(-LATERAL_KV * in.vx / G0, -lim, lim) * fade;
    float py = clampf(-LATERAL_KV * in.vy / G0, -lim, lim) * fade;
    // Trim acts along body y; map its effect into the world frame via the attitude.
    float trim = tvc_.trim(LANDING_OFFSET_M);
    Vec3 trim_w = in.q.rotate(Vec3(0, sinf(trim), 0));
    px -= trim_w.x;
    py -= trim_w.y;
    const float thrust_now = throttle_ * thrust_scale_ * nominal;
    GimbalCmd g = tvc_.update(in.q, in.rate, up_command(px, py), thrust_now, LANDING_OFFSET_M, dt);
    out_.gimbal_dy = g.dy;
    out_.gimbal_dz = g.dz;
}
