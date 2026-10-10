// Servos (TVC gimbal, exhaust paddles, mechanism latches) and pyro channels.
#pragma once
#include <Arduino.h>

class Actuators {
public:
    void begin();
    void gimbal(float dy_rad, float dz_rad);   // thrust deflection (body frame)
    void throttle(float k);                    // exhaust-paddle throttle 0.25..1
    void drogue_latch(bool open);
    void drogue_cut(bool release);
    void leg_pin(bool release);
    void chute_latch(bool open);

    // Pyro: fire both landing igniters for PYRO_PULSE_MS. Only works when the
    // software enable is set AND the arm key is in; fires once per boot.
    void pyro_enable(bool en);
    bool fire_landing_igniters();
    void service();                            // call every loop: ends the pulse
    bool continuity_a() const;
    bool continuity_b() const;
    bool arm_key() const;
    float vbat() const;

private:
    void write_us(uint8_t pin, float us, float hz);
    bool enabled_ = false;
    bool fired_ = false;
    uint32_t fire_ms_ = 0;
};
