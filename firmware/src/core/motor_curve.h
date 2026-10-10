// Tabulated solid-motor thrust curve (from a RASP .eng file) with the same
// mass model the simulator uses: propellant burns in proportion to impulse.
#pragma once
#include <stdint.h>

struct MotorCurve {
    const float* t;      // s, ascending, t[0] may be > 0
    const float* f;      // N
    int n;
    float propellant_kg;
    float total_kg;

    float burn_time() const { return t[n - 1]; }
    float thrust(float tb) const;            // 0 outside the burn
    float impulse(float tb) const;           // delivered up to tb
    float total_impulse() const { return impulse(burn_time()); }
    float mass(float tb) const;              // casing + remaining propellant
    float casing_kg() const { return total_kg - propellant_kg; }
};
