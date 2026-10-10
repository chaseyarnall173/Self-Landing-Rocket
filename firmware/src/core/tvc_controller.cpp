#include "tvc_controller.h"

float TvcController::trim(float offset_y) const {
    // With no commanded rotation tau_z = -L*T*sin(dy) - offset*T = 0, so the
    // deflection that cancels the offset torque is sin(dy) = -offset / L.
    return asinf(clampf(-offset_y / g_.lever, -0.5f, 0.5f));
}

GimbalCmd TvcController::update(const Quat& q, const Vec3& w, const Vec3& up_cmd, float thrust,
                                float offset_y, float dt) {
    GimbalCmd out;
    if (thrust < 1.0f) {
        return out;  // no authority without thrust: centre the gimbal
    }
    // Attitude error: rotation taking body +x onto the commanded direction,
    // expressed in the body frame (small-angle components about y and z).
    Vec3 u_b = q.rotate_inv(up_cmd.normalized());
    Vec3 x_b(1, 0, 0);
    Vec3 err = x_b.cross(u_b);  // (0, -u_b.z, u_b.y)
    const float ey = err.y, ez = err.z;

    const float kp = g_.wn * g_.wn;
    const float kd = 2.0f * g_.zeta * g_.wn;
    float ay = kp * ey - kd * w.y + g_.ki * iy_;   // desired angular accel, body y
    float az = kp * ez - kd * w.z + g_.ki * iz_;   // desired angular accel, body z

    // Body torques: r = (-L, 0, offset) x F with F = T*(1, sin dy, sin dz):
    //   tau_y = L*T*sin(dz)        tau_z = -L*T*sin(dy) - offset*T
    float s_dz = (g_.inertia * ay) / (thrust * g_.lever);
    float s_dy = -(g_.inertia * az) / (thrust * g_.lever) - offset_y / g_.lever;
    const float lim = sinf(g_.limit_rad);
    bool sat_y = fabsf(s_dy) >= lim, sat_z = fabsf(s_dz) >= lim;
    s_dy = clampf(s_dy, -lim, lim);
    s_dz = clampf(s_dz, -lim, lim);
    if (!sat_z) iy_ += ey * dt;   // conditional integration (anti-windup)
    if (!sat_y) iz_ += ez * dt;
    out.dy = asinf(s_dy);
    out.dz = asinf(s_dz);
    out.saturated = sat_y || sat_z;
    return out;
}
