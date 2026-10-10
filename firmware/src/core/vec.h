// Minimal 3-vector / quaternion math for the flight computer (no heap, no STL).
#pragma once
#include <math.h>

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float norm() const { return sqrtf(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float n = norm();
        return n > 1e-9f ? (*this) * (1.0f / n) : Vec3(0, 0, 0);
    }
};

// Unit quaternion rotating BODY vectors into the WORLD frame.
struct Quat {
    float w = 1, x = 0, y = 0, z = 0;
    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator*(const Quat& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    void normalize() {
        float n = sqrtf(w * w + x * x + y * y + z * z);
        if (n > 1e-9f) { w /= n; x /= n; y /= n; z /= n; }
    }
    Vec3 rotate(const Vec3& v) const {  // body -> world
        Quat p(0, v.x, v.y, v.z);
        Quat r = (*this) * p * conj();
        return {r.x, r.y, r.z};
    }
    Vec3 rotate_inv(const Vec3& v) const {  // world -> body
        Quat p(0, v.x, v.y, v.z);
        Quat r = conj() * p * (*this);
        return {r.x, r.y, r.z};
    }
    static Quat from_rotvec(const Vec3& r) {
        float a = r.norm();
        if (a < 1e-9f) return {1, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z};
        float s = sinf(0.5f * a) / a;
        return {cosf(0.5f * a), r.x * s, r.y * s, r.z * s};
    }
    // Shortest rotation taking unit vector a onto unit vector b.
    static Quat between(const Vec3& a, const Vec3& b) {
        Vec3 c = a.cross(b);
        float d = a.dot(b);
        Quat q(1.0f + d, c.x, c.y, c.z);
        if (q.w < 1e-6f) q = Quat(0, 0, 1, 0);  // 180 deg: any orthogonal axis
        q.normalize();
        return q;
    }
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr float G0 = 9.80665f;
constexpr float DEG = 0.017453292519943295f;
