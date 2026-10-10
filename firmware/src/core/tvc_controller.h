// Two-axis thrust-vector attitude controller (dynamic-inversion PID).
//
// Body frame: +x along the vehicle axis toward the nose, y/z lateral (the two
// gimbal axes). A thrust vector deflected by small angles (dy, dz) toward
// body +y/+z, applied a distance L aft of the CG, produces body torques
//      tau_y = +L*T*sin(dz) ... (nose pitches toward -z)
//      tau_z = -L*T*sin(dy) ... (nose yaws toward -y)
// so the controller inverts those relations, scaled by the CURRENT thrust,
// which keeps the loop bandwidth constant as thrust changes.
#pragma once
#include "vec.h"

struct TvcGains {
    float wn, zeta, ki, limit_rad, inertia, lever;
};

struct GimbalCmd {
    float dy = 0, dz = 0;   // thrust deflection toward body +y / +z (rad)
    bool saturated = false;
};

class TvcController {
public:
    explicit TvcController(const TvcGains& g) : g_(g) {}
    void reset() { iy_ = iz_ = 0; }

    // q: body->world attitude, w: body rates (rad/s), up_cmd: desired thrust
    // (body +x) direction in the world frame, thrust: current thrust (N),
    // offset_y: lateral thrust-line offset along body y (m), dt: seconds.
    GimbalCmd update(const Quat& q, const Vec3& w, const Vec3& up_cmd, float thrust,
                     float offset_y, float dt);

    // Steady deflection that puts an offset thrust line through the CG.
    float trim(float offset_y) const;

private:
    TvcGains g_;
    float iy_ = 0, iz_ = 0;
};
