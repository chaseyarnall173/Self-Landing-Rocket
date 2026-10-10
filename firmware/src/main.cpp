// SLR-1 flight computer - Teensy 4.1
//
//   1 kHz : IMU read, attitude integration, altitude-KF predict
//   50 Hz : barometer update, GPS velocity (when available)
//   200 Hz: flight state machine -> TVC gimbal, paddles, mechanisms, pyro
//   100 Hz: RAM log;   5 Hz: telemetry / USB status
//
// SAFETY: the landing-motor igniters can only fire when (1) the physical arm
// key is in, (2) the state machine has armed on the pad (vertical, both
// igniters show continuity), and (3) the in-flight envelope checks in
// FlightFsm pass. See docs/10-flight-ops.md before connecting any igniter.
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>

#include "core/estimator.h"
#include "core/flight_fsm.h"
#include "core/motor_tables.h"
#include "core/params.h"
#include "hal/actuators.h"
#include "hal/board.h"
#include "hal/logger.h"
#include "hal/sensors.h"

static Bmi088 imu;
static Bmp390 baro;
static TfMini lidar;
static SFE_UBLOX_GNSS gps;
static Actuators act;
static Logger logger;
static AttitudeEstimator att;
static AltitudeKF kf;
static FlightFsm fsm(MOTOR_F42T, MOTOR_F25W);  // ascent, landing

static bool have_gps = false;
static float ground_p = 101325.0f, ground_t = 15.0f;
static float baro_h = 0, lidar_h = -1, f_axial = 0;
static float vel_x = 0, vel_y = 0;     // world horizontal velocity (GPS-aided)
static Vec3 accel_b, gyro_b;
static bool pad_calibrated = false;
static uint32_t pad_still_since = 0;
static FsmOutputs out;

static void on_event(float t, const char* msg) { logger.event(t, msg); }

static float now_s() { return micros() * 1e-6f; }

static void fail(const char* what) {
    // Refuse to run without a working sensor: beep forever.
    while (true) {
        Serial.printf("SENSOR FAIL: %s\n", what);
        tone(board::BUZZER, 400, 500);
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    act.begin();  // pyro outputs LOW first thing
    act.pyro_enable(false);
    Wire.begin();
    Wire.setClock(board::I2C0_HZ);
    Wire1.begin();
    Wire1.setClock(board::I2C1_HZ);
    if (!imu.begin(Wire1, board::BMI088_ACC_ADDR, board::BMI088_GYR_ADDR)) fail("BMI088");
    if (!baro.begin(Wire, board::BMP390_ADDR)) fail("BMP390");
    lidar.begin(Serial1, board::LIDAR_BAUD);
    if (gps.begin(Wire)) {
        gps.setI2COutput(COM_TYPE_UBX);
        gps.setNavigationFrequency(10);
        gps.setAutoPVT(true);
        have_gps = true;
    }
    fsm.on_event = on_event;
    kf.reset(0);
    logger.event(now_s(), "BOOT");
}

// Ground reference + attitude levelling while the vehicle sits still on the pad.
static void pad_calibration(float t) {
    bool still = fabsf(accel_b.norm() - G0) < 0.3f && gyro_b.norm() < 0.05f;
    if (!still) {
        pad_still_since = millis();
        return;
    }
    att.pad_sample(accel_b, gyro_b);
    if (millis() - pad_still_since > 3000 && att.pad_finalize()) {
        pad_calibrated = true;
        pad_still_since = millis();
    }
}

void loop() {
    static uint32_t next_imu = 0, next_ctrl = 0, next_baro = 0, next_log = 0, next_tel = 0;
    const uint32_t us = micros();

    // ------------------------------------------------------------ 1 kHz
    if ((int32_t)(us - next_imu) >= 0) {
        next_imu = us + 1000;
        Vec3 a_s, g_s;
        if (imu.read(a_s, g_s)) {
            accel_b = sensor_to_body(a_s);
            gyro_b = sensor_to_body(g_s);
            f_axial = accel_b.x;
            const Phase ph = fsm.phase();
            if (ph == Phase::PAD_SAFE) pad_calibration(now_s());
            att.update(gyro_b, params::IMU_DT_S);
            Vec3 a_w = att.q().rotate(accel_b);
            kf.predict(a_w.z - G0, params::IMU_DT_S);
            vel_x += a_w.x * params::IMU_DT_S;
            vel_y += a_w.y * params::IMU_DT_S;
            if (ph == Phase::PAD_SAFE || ph == Phase::PAD_ARMED || ph == Phase::LANDED_SAFE) {
                kf.zero_velocity();
                vel_x = vel_y = 0;
            }
        }
    }

    // ------------------------------------------------------------ 50 Hz
    if ((int32_t)(us - next_baro) >= 0) {
        next_baro = us + 20000;
        float p, tc;
        if (baro.read(p, tc)) {
            if (fsm.phase() == Phase::PAD_SAFE) {  // track the ground reference
                ground_p += 0.02f * (p - ground_p);
                ground_t = tc;
            }
            baro_h = baro_altitude_m(p, ground_p, ground_t);
            // Inflate the baro noise during motor burns (plume / ram pressure).
            const bool burning = fsm.phase() == Phase::BOOST || fsm.phase() == Phase::LANDING_BURN;
            kf.update_baro(baro_h, burning ? 3.0f : 0.5f);
        }
        if (have_gps && gps.getPVT()) {
            // NED mm/s -> attitude world frame (y = fin-1 direction on the pad,
            // x = y cross up). Complementary correction of the IMU-integrated value.
            const float ve = gps.getNedEastVel() * 1e-3f, vn = gps.getNedNorthVel() * 1e-3f;
            const float psi = board::PAD_HEADING_DEG * DEG;
            const float wx = ve * cosf(psi) - vn * sinf(psi);
            const float wy = ve * sinf(psi) + vn * cosf(psi);
            vel_x += 0.2f * (wx - vel_x);
            vel_y += 0.2f * (wy - vel_y);
        }
    }

    if (lidar.poll()) {
        const float tilt = att.tilt_rad();
        float d = lidar.distance_m();
        if (d > 0 && d < 10.0f && tilt < 30 * DEG && params::VIRTUAL_GROUND_M == 0.0f) {
            lidar_h = d * cosf(tilt) - board::LIDAR_OFFSET_M;
            kf.update_range(lidar_h, 0.06f);
        } else {
            lidar_h = -1;
        }
    }

    // ------------------------------------------------------------ 200 Hz
    if ((int32_t)(us - next_ctrl) >= 0) {
        next_ctrl = us + 5000;
        FsmInputs in;
        in.t = now_s();
        in.q = att.q();
        in.rate = att.rate();
        in.tilt = att.tilt_rad();
        in.h_aft = kf.h();
        in.vz = kf.v();
        in.vx = vel_x;
        in.vy = vel_y;
        in.f_axial = f_axial;
        in.rho = 1.225f * expf(-kf.h() / 8500.0f);
        in.pad_calibrated = pad_calibrated;
        in.arm_switch = act.arm_key() && act.vbat() > board::VBAT_MIN;
        in.continuity_a = act.continuity_a();
        in.continuity_b = act.continuity_b();
        in.vbat = act.vbat();
        out = fsm.step(in, params::CONTROL_DT_S);

        act.pyro_enable(out.pyro_enable);
        if (out.fire_landing) act.fire_landing_igniters();
        act.gimbal(out.gimbal_dy, out.gimbal_dz);
        act.throttle(out.throttle);
        act.airbrake_latch(out.airbrakes_deploy);
        act.leg_pin(out.legs_deploy);
        act.chute_latch(out.chute_deploy);
        act.service();

        static bool flushed = false;
        if (!flushed && (out.phase == Phase::LANDED_SAFE ||
                         (out.phase == Phase::ABORT_CHUTE && kf.h() < 1.0f &&
                          fabsf(kf.v()) < 0.3f))) {
            flushed = true;
            act.pyro_enable(false);
            logger.event(now_s(), logger.flush_to_sd() ? "LOG SAVED" : "LOG SAVE FAILED");
        }
    }

    // ------------------------------------------------------------ 100 Hz log
    if ((int32_t)(us - next_log) >= 0) {
        next_log = us + 10000;
        if (fsm.phase() != Phase::PAD_SAFE) {
            LogRecord r{};
            r.t = now_s();
            r.h = kf.h();
            r.vz = kf.v();
            r.vx = vel_x;
            r.vy = vel_y;
            r.qw = att.q().w; r.qx = att.q().x; r.qy = att.q().y; r.qz = att.q().z;
            r.f_axial = f_axial;
            r.gimbal_dy = out.gimbal_dy;
            r.gimbal_dz = out.gimbal_dz;
            r.throttle = out.throttle;
            r.baro_h = baro_h;
            r.lidar_m = lidar_h;
            r.phase = (uint8_t)out.phase;
            r.flags = (out.airbrakes_deploy << 0) | (out.legs_deploy << 1) |
                      (out.chute_deploy << 2) | (act.arm_key() << 4);
            logger.push(r);
        }
    }

    // ------------------------------------------------------------ 5 Hz status
    if ((int32_t)(us - next_tel) >= 0) {
        next_tel = us + 200000;
        Serial.printf("%s cal=%d key=%d contA=%d contB=%d vbat=%.2f tilt=%.1f h=%.2f vz=%.2f lidar=%.2f\n",
                      phase_name(fsm.phase()), pad_calibrated, act.arm_key(), act.continuity_a(),
                      act.continuity_b(), act.vbat(), att.tilt_rad() / DEG, kf.h(), kf.v(), lidar_h);
    }
}
