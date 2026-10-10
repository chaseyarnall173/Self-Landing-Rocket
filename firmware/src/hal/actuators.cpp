#include "actuators.h"

#include "../core/params.h"
#include "../core/vec.h"
#include "board.h"

using namespace board;

static constexpr int PWM_BITS = 15;

void Actuators::write_us(uint8_t pin, float us, float hz) {
    // Hardware PWM: duty = pulse / period at 15-bit resolution.
    const float period_us = 1e6f / hz;
    uint32_t duty = (uint32_t)(clampf(us, 800, 2200) / period_us * ((1 << PWM_BITS) - 1));
    analogWrite(pin, duty);
}

void Actuators::begin() {
    pinMode(PYRO_A, OUTPUT);
    pinMode(PYRO_B, OUTPUT);
    digitalWrite(PYRO_A, LOW);
    digitalWrite(PYRO_B, LOW);
    pinMode(ARM_SENSE, INPUT_PULLUP);
    analogReadResolution(12);
    analogWriteResolution(PWM_BITS);
    const uint8_t fast[] = {SERVO_GIMBAL_Y, SERVO_GIMBAL_Z, SERVO_PADDLES};
    const uint8_t slow[] = {SERVO_DROGUE_LATCH, SERVO_DROGUE_CUT, SERVO_LEG_PIN, SERVO_CHUTE_LATCH};
    for (uint8_t p : fast) analogWriteFrequency(p, FAST_SERVO_HZ);
    for (uint8_t p : slow) analogWriteFrequency(p, SLOW_SERVO_HZ);
    gimbal(0, 0);
    throttle(1.0f);
    drogue_latch(false);
    drogue_cut(false);
    leg_pin(false);
    chute_latch(false);
}

void Actuators::gimbal(float dy, float dz) {
    const float lim = params::GIMBAL_LIMIT_DEG * DEG;
    write_us(SERVO_GIMBAL_Y, GIMBAL_Y_CENTER_US + GIMBAL_Y_US_PER_RAD * clampf(dy, -lim, lim),
             FAST_SERVO_HZ);
    write_us(SERVO_GIMBAL_Z, GIMBAL_Z_CENTER_US + GIMBAL_Z_US_PER_RAD * clampf(dz, -lim, lim),
             FAST_SERVO_HZ);
}

void Actuators::throttle(float k) {
    // Calibration table is at k = 1.0, 0.75, 0.5, 0.25; interpolate.
    k = clampf(k, 0.25f, 1.0f);
    float pos = (1.0f - k) / 0.25f;  // 0..3
    int i = (int)pos;
    if (i >= 3) i = 2;
    float u = pos - i;
    write_us(SERVO_PADDLES, PADDLE_US[i] + u * (PADDLE_US[i + 1] - PADDLE_US[i]), FAST_SERVO_HZ);
}

void Actuators::drogue_latch(bool open) {
    write_us(SERVO_DROGUE_LATCH, open ? DROGUE_LATCH_OPEN_US : DROGUE_LATCH_CLOSED_US,
             SLOW_SERVO_HZ);
}
void Actuators::drogue_cut(bool rel) {
    write_us(SERVO_DROGUE_CUT, rel ? DROGUE_CUT_RELEASE_US : DROGUE_CUT_HOLD_US, SLOW_SERVO_HZ);
}
void Actuators::leg_pin(bool rel) {
    write_us(SERVO_LEG_PIN, rel ? LEG_PIN_RELEASE_US : LEG_PIN_HOLD_US, SLOW_SERVO_HZ);
}
void Actuators::chute_latch(bool open) {
    write_us(SERVO_CHUTE_LATCH, open ? CHUTE_LATCH_OPEN_US : CHUTE_LATCH_CLOSED_US,
             SLOW_SERVO_HZ);
}

void Actuators::pyro_enable(bool en) {
    enabled_ = en;
    if (!en) {
        digitalWrite(PYRO_A, LOW);
        digitalWrite(PYRO_B, LOW);
    }
}

bool Actuators::fire_landing_igniters() {
    if (!enabled_ || fired_ || !arm_key()) return false;
    fired_ = true;
    fire_ms_ = millis();
    digitalWrite(PYRO_A, HIGH);
    digitalWrite(PYRO_B, HIGH);
    return true;
}

void Actuators::service() {
    if (fired_ && millis() - fire_ms_ >= PYRO_PULSE_MS) {
        digitalWrite(PYRO_A, LOW);
        digitalWrite(PYRO_B, LOW);
    }
}

static float adc_volts(uint8_t pin) { return analogRead(pin) * 3.3f / 4095.0f; }
bool Actuators::continuity_a() const { return adc_volts(CONT_A) > CONT_THRESHOLD_V; }
bool Actuators::continuity_b() const { return adc_volts(CONT_B) > CONT_THRESHOLD_V; }
bool Actuators::arm_key() const { return digitalRead(ARM_SENSE) == LOW; }
float Actuators::vbat() const { return adc_volts(VBAT_SENSE) * VBAT_DIVIDER; }
