// Native (desktop) unit + software-in-the-loop tests for the flight core.
// Build & run:   make -C firmware test
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "estimator.h"
#include "flight_fsm.h"
#include "golden.h"
#include "landing_guidance.h"
#include "motor_tables.h"
#include "params.h"
#include "tvc_controller.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                       \
    do {                                                       \
        if (cond) {                                            \
            ++g_pass;                                          \
        } else {                                               \
            ++g_fail;                                          \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);        \
            printf(__VA_ARGS__);                               \
            printf("\n");                                      \
        }                                                      \
    } while (0)
#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static LandingVehicle golden_vehicle() {
    LandingVehicle v;
    v.landing = &MOTOR_G40W;
    v.dry_mass_kg = G_DRY_MASS;
    v.spent_ascent_kg = G_ASCENT_CASING;
    v.cda_pre_m2 = G_CDA_PRE;
    v.cda_burn_m2 = G_CDA_BURN;
    v.throttle_min = G_KMIN;
    v.v_td = 0.5f;
    v.window_hop_margin = 1.0f;
    v.window_end_thrust_ratio = 1.25f;
    return v;
}

static void test_motor() {
    const MotorCurve& m = MOTOR_G40W;
    CHECK(NEAR(m.total_impulse(), 99.0f, 0.5f), "G40W impulse %.2f", m.total_impulse());
    CHECK(NEAR(m.thrust(0.5f), 63.063f, 0.01f), "thrust(0.5)=%.3f", m.thrust(0.5f));
    CHECK(m.thrust(-0.1f) == 0.0f && m.thrust(3.0f) == 0.0f, "thrust outside burn");
    CHECK(NEAR(m.mass(10.0f), m.casing_kg(), 1e-5f), "burnt-out mass");
    CHECK(NEAR(m.mass(0.0f), m.total_kg, 1e-6f), "unlit mass");
}

static void test_guidance_matches_python() {
    LandingGuidance g(golden_vehicle());
    float t1, t2;
    g.window(t1, t2);
    CHECK(NEAR(t1, G_WINDOW_T1, 0.01f) && NEAR(t2, G_WINDOW_T2, 0.01f),
          "window [%.3f %.3f] vs python [%.3f %.3f]", t1, t2, G_WINDOW_T1, G_WINDOW_T2);
    for (const PlanCase& c : PLAN_CASES) {
        float k0, k1;
        bool ok = g.plan(c.h, c.v, c.rho, c.tb, c.tl, 1.0f, G_CDA_BURN, 1.0f, k0, k1);
        CHECK(ok && NEAR(k0, c.k0, 2e-3f) && NEAR(k1, c.k1, 5e-3f),
              "plan h=%.1f v=%.1f: C++ (%.4f, %.4f) python (%.4f, %.4f)", c.h, c.v, k0, k1, c.k0,
              c.k1);
    }
    // Ignition altitude at a 22.7 m/s drogue descent.
    float h_fire = -1;
    for (int i = 1200; i > 0; --i) {
        float h = i * 0.05f;
        if (g.fire_now(h, -22.7f, 1.2f, 0.30f, 0.0f, true)) {
            h_fire = h;
            break;
        }
    }
    CHECK(NEAR(h_fire, G_FIRE_ALT_AT_22_7, 0.15f), "fire altitude %.2f vs python %.2f", h_fire,
          G_FIRE_ALT_AT_22_7);
}

static void test_tvc_closed_loop() {
    // Single-axis rigid body: lean 8 deg, constant 30 N thrust, offset motor.
    TvcController c(TvcGains{params::ATT_WN, params::ATT_ZETA, params::ATT_KI,
                             params::GIMBAL_LIMIT_DEG * DEG, params::INERTIA_KGM2,
                             params::GIMBAL_LEVER_M});
    // -90 deg about y maps body +x onto world +z (upright); a further +8 deg
    // about world y leans the nose toward world +x.
    Quat q = Quat::from_rotvec(Vec3(0, 8 * DEG, 0)) * Quat::from_rotvec(Vec3(0, -90 * DEG, 0));
    // Sanity: nose leans toward +x.
    Vec3 ax = q.rotate(Vec3(1, 0, 0));
    CHECK(ax.z > 0.98f && ax.x > 0.1f, "test setup lean (%.3f %.3f %.3f)", ax.x, ax.y, ax.z);
    Vec3 w(0, 0, 0);
    const float T = 30.0f, L = params::GIMBAL_LEVER_M, I = params::INERTIA_KGM2;
    const float e = params::LANDING_OFFSET_M;
    for (int i = 0; i < 800; ++i) {  // 4 s at 200 Hz
        GimbalCmd g = c.update(q, w, Vec3(0, 0, 1), T, e, 0.005f);
        // Body torques (same model as the controller documents) -> rates.
        Vec3 tau(0, L * T * sinf(g.dz), -L * T * sinf(g.dy) - e * T);
        w = w + tau * (0.005f / I);
        q = q * Quat::from_rotvec(w * 0.005f);
        q.normalize();
    }
    ax = q.rotate(Vec3(1, 0, 0));
    float tilt = acosf(clampf(ax.z, -1, 1)) / DEG;
    CHECK(tilt < 0.3f, "TVC did not converge: tilt %.2f deg", tilt);
}

static void test_altitude_kf() {
    AltitudeKF kf;
    kf.reset(0);
    srand(1);
    auto noise = [](float s) {
        float u = 0;
        for (int i = 0; i < 12; ++i) u += rand() / (float)RAND_MAX;
        return (u - 6.0f) * s;
    };
    float h = 0, v = 0;
    const float bias = 0.15f;  // accelerometer bias the filter must learn
    for (int i = 0; i < 8000; ++i) {  // 8 s, 1 kHz
        float a = (i < 2000) ? 15.0f : -9.80665f;
        v += a * 0.001f;
        h += v * 0.001f;
        kf.predict(a + bias + noise(0.3f), 0.001f);
        if (i % 20 == 0) kf.update_baro(h + noise(0.4f), 0.5f);
    }
    CHECK(NEAR(kf.h(), h, 1.0f), "KF h %.2f truth %.2f", kf.h(), h);
    CHECK(NEAR(kf.v(), v, 0.5f), "KF v %.2f truth %.2f", kf.v(), v);
    CHECK(NEAR(kf.bias(), bias, 0.1f), "KF bias %.3f truth %.3f", kf.bias(), bias);
}

static void test_attitude_integration() {
    AttitudeEstimator est;
    for (int i = 0; i < 500; ++i) est.pad_sample(Vec3(9.80665f, 0, 0), Vec3(0.01f, 0, 0));
    CHECK(est.pad_finalize(), "pad finalize");
    CHECK(est.tilt_rad() < 1e-3f, "pad tilt %.4f", est.tilt_rad());
    // Rotate 30 deg/s about body y for 1 s (bias 0.01 rad/s on x must be removed).
    for (int i = 0; i < 1000; ++i) est.update(Vec3(0.01f, 30 * DEG, 0), 0.001f);
    CHECK(NEAR(est.tilt_rad() / DEG, 30.0f, 0.2f), "integrated tilt %.2f", est.tilt_rad() / DEG);
}

// --------------------------------------------------------------- SIL test
// A vertical-only truth model drives the real FlightFsm through a whole
// mission: boost, coast, drogue descent, landing burn, touchdown.
static float g_events_t[32];
static const char* g_events[32];
static int g_nev = 0;
static char g_evbuf[32][64];
static void on_event(float t, const char* msg) {
    if (g_nev < 32) {
        snprintf(g_evbuf[g_nev], 64, "%s", msg);
        g_events[g_nev] = g_evbuf[g_nev];
        g_events_t[g_nev] = t;
        ++g_nev;
    }
}

static float sil_flight(float true_delay, float true_kmin, bool print) {
    FlightFsm fsm(MOTOR_G40W, MOTOR_G40W);
    g_nev = 0;
    fsm.on_event = on_event;
    const float dt = 0.005f;
    float t = 0, h = 0, v = 0;
    float t_cmd = -1, k_act = 1.0f;
    bool drogue = false, legs = false, landed = false;
    float vz_td = 0, max_h = 0;
    const float A = 3.14159f * 0.1016f * 0.1016f / 4;
    Quat up = Quat::from_rotvec(Vec3(0, -90 * DEG, 0));  // body +x = world up
    FsmInputs in;
    in.q = up;
    in.pad_calibrated = in.arm_switch = in.continuity_a = in.continuity_b = true;
    float t_launch = 1.0f;
    for (int i = 0; i < 6000 && !landed; ++i) {
        float ta = t - t_launch;
        float tl = (t_cmd >= 0) ? t - t_cmd - true_delay : -1;
        float m = params::DRY_MASS_KG + MOTOR_G40W.mass(ta > 0 ? ta : 0) +
                  (tl > 0 ? MOTOR_G40W.mass(tl) : MOTOR_G40W.total_kg);
        float thrust = (ta > 0 ? MOTOR_G40W.thrust(ta) : 0) +
                       (tl > 0 ? fmaxf(k_act, true_kmin) * MOTOR_G40W.thrust(tl) : 0);
        float cda = (v >= 0) ? 0.55f * A : (drogue ? params::CDA_PRE_M2 : params::CDA_BURN_M2) +
                                                 (legs ? params::CDA_LEGS_M2 : 0);
        float drag = -0.5f * 1.2f * cda * v * fabsf(v);
        float a = (thrust + drag) / m - G0;
        bool on_pad = (h <= 0 && a < 0 && t < t_launch + 0.5f);
        if (on_pad) a = 0;
        v += a * dt;
        h += v * dt;
        if (h < 0 && !on_pad && t > t_launch + 1) {
            landed = true;
            vz_td = v;
        }
        if (h < 0) h = 0;
        if (h > max_h) max_h = h;
        in.t = t;
        in.h_aft = h;
        in.vz = v;
        in.f_axial = on_pad ? G0 : (thrust + drag) / m;
        in.rho = 1.2f;
        FsmOutputs o = fsm.step(in, dt);
        if (o.fire_landing && t_cmd < 0) t_cmd = t;
        k_act += clampf((o.throttle - k_act) / 0.04f, -6, 6) * dt;
        drogue = drogue || o.drogue_deploy;
        if (o.drogue_release) drogue = false;
        legs = o.legs_deploy;
        t += dt;
    }
    if (print) {
        printf("  SIL delay=%.2f kmin=%.2f apogee=%.1f m touchdown vz=%.2f m/s, events:", true_delay,
               true_kmin, max_h, vz_td);
        for (int i = 0; i < g_nev; ++i) printf(" [%.2f %s]", g_events_t[i], g_events[i]);
        printf("\n");
    }
    return landed ? vz_td : -99.0f;
}

static void test_sil() {
    const float delays[] = {0.20f, 0.30f, 0.40f};
    for (float d : delays) {
        float vz = sil_flight(d, 0.25f, true);
        CHECK(vz > -2.5f && vz < 0.5f, "SIL delay %.2f: touchdown %.2f m/s", d, vz);
    }
}

int main() {
    test_motor();
    test_guidance_matches_python();
    test_tvc_closed_loop();
    test_altitude_kf();
    test_attitude_integration();
    test_sil();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
