// Poses: the laser pose from the base pose and the extrinsics (laser_pose), and the pose at any
// instant between two samples (interpolate: position linear, orientation slerp). Used by
// odometry_node and synchronizer_node.
#pragma once

#include "laser_hhrp/calibration_data.hpp"
#include "laser_hhrp/rotation.hpp"

namespace laser_hhrp {

struct Pose {
  Vec3 position_m{};             // in the world (odom) frame
  Quat orientation{1, 0, 0, 0};  // body -> world
};

/// Laser pose from the base pose: the laser sits at laser_in_roller_m in the base frame, rotated by
/// roller_from_laser (the roller frame is the base frame on the surface). Identity until the laser
/// extrinsics are calibrated: then the laser pose is the base pose.
inline Pose laser_pose(const Pose& base, const Extrinsics& e) {
  Pose out;
  out.position_m = add(base.position_m, mul(quat_to_mat(base.orientation), e.laser_in_roller_m));
  out.orientation = quat_normalized(quat_mul(base.orientation, e.roller_from_laser));
  return out;
}

/// Pose at fraction f between a (0) and b (1): position linear, orientation slerp.
inline Pose interpolate(const Pose& a, const Pose& b, double f) {
  Pose out;
  for (int i = 0; i < 3; ++i) out.position_m[i] = a.position_m[i] + f * (b.position_m[i] - a.position_m[i]);
  out.orientation = quat_slerp(a.orientation, b.orientation, f);
  return out;
}

}  // namespace laser_hhrp
