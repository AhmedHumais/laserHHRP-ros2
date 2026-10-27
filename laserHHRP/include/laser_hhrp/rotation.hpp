// Small math toolkit (no dependencies): 3-vectors, 3x3 matrices (rows) and unit quaternions
// (w, x, y, z), enough for IMU integration, odometry and the extrinsic calibration.
#pragma once

#include <array>
#include <cmath>

#include "laser_hhrp/calibration_data.hpp"

namespace laser_hhrp {

using Mat3 = std::array<Vec3, 3>;  // rows

inline Vec3 add(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 scale(const Vec3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline Vec3 normalized(const Vec3& a) {
  const double n = norm(a);
  return n > 0 ? scale(a, 1.0 / n) : a;
}

/// Angle between two vectors [rad] (0 if one is zero).
inline double angle_between(const Vec3& a, const Vec3& b) {
  const double n = norm(a) * norm(b);
  if (n <= 0) return 0;
  const double c = dot(a, b) / n;
  return std::acos(c > 1 ? 1 : c < -1 ? -1 : c);
}

inline Mat3 identity3() { return {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}; }
inline Vec3 mul(const Mat3& m, const Vec3& v) { return {dot(m[0], v), dot(m[1], v), dot(m[2], v)}; }
inline Mat3 transpose(const Mat3& m) {
  return {Vec3{m[0][0], m[1][0], m[2][0]}, Vec3{m[0][1], m[1][1], m[2][1]},
          Vec3{m[0][2], m[1][2], m[2][2]}};
}
inline Mat3 mul(const Mat3& a, const Mat3& b) {
  const Mat3 bt = transpose(b);
  Mat3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i][j] = dot(a[i], bt[j]);
  return r;
}

inline Quat quat_mul(const Quat& a, const Quat& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}
inline Quat quat_normalized(const Quat& q) {
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return n > 0 ? Quat{q[0] / n, q[1] / n, q[2] / n, q[3] / n} : Quat{1, 0, 0, 0};
}
/// Rotation by the rotation vector `r` (axis * angle, rad).
inline Quat quat_from_rotvec(const Vec3& r) {
  const double a = norm(r);
  if (a < 1e-12) return quat_normalized({1, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]});
  const double s = std::sin(0.5 * a) / a;
  return {std::cos(0.5 * a), r[0] * s, r[1] * s, r[2] * s};
}
/// Spherical interpolation from a (f = 0) to b (f = 1), shortest way.
inline Quat quat_slerp(const Quat& a, Quat b, double f) {
  double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0) {
    b = {-b[0], -b[1], -b[2], -b[3]};
    d = -d;
  }
  if (d > 0.9995) {  // nearly equal: linear is exact enough
    return quat_normalized({a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                            a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3])});
  }
  const double th = std::acos(d), s = std::sin(th);
  const double wa = std::sin((1 - f) * th) / s, wb = std::sin(f * th) / s;
  return {wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2],
          wa * a[3] + wb * b[3]};
}
inline Quat quat_conj(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }

inline Mat3 quat_to_mat(const Quat& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {Vec3{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
          Vec3{2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
          Vec3{2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
}
inline Quat mat_to_quat(const Mat3& m) {
  const double tr = m[0][0] + m[1][1] + m[2][2];
  Quat q;
  if (tr > 0) {
    const double s = 2 * std::sqrt(tr + 1);
    q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = 2 * std::sqrt(1 + m[0][0] - m[1][1] - m[2][2]);
    q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
  } else if (m[1][1] > m[2][2]) {
    const double s = 2 * std::sqrt(1 + m[1][1] - m[0][0] - m[2][2]);
    q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
  } else {
    const double s = 2 * std::sqrt(1 + m[2][2] - m[0][0] - m[1][1]);
    q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
  }
  if (q[0] < 0) q = {-q[0], -q[1], -q[2], -q[3]};
  return quat_normalized(q);
}

}  // namespace laser_hhrp
