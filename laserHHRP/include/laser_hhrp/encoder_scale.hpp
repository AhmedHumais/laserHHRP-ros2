// Metres of roller travel per encoder count, and the counting direction: measured by the
// extrinsic calibration if done, otherwise from the parameters (encoder.roller_diameter_m,
// encoder.counts_per_turn, encoder.direction_sign). Shared by the nodes that need the scale.
#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "laser_hhrp/msg/calibration_status.hpp"

namespace laser_hhrp {

class EncoderScale {
 public:
  /// Declares the encoder parameters on `node`.
  explicit EncoderScale(rclcpp::Node& node) : node_(node) {
    node.declare_parameter("encoder.roller_diameter_m", 0.030);
    node.declare_parameter("encoder.counts_per_turn", 4096);
    node.declare_parameter("encoder.direction_sign", 1);
  }
  /// Uses the latest calibration (subscribe to calibration/status and pass every message).
  void set_calibration(const msg::CalibrationStatus& c) {
    measured_ = c.extrinsics_calibrated && c.metres_per_count > 0;
    measured_mpc_ = c.metres_per_count;
    measured_sign_ = c.encoder_direction_sign < 0 ? -1 : 1;
  }
  double metres_per_count() const {
    if (measured_) return measured_mpc_;
    return M_PI * node_.get_parameter("encoder.roller_diameter_m").as_double() /
           static_cast<double>(node_.get_parameter("encoder.counts_per_turn").as_int());
  }
  int direction_sign() const {
    if (measured_) return measured_sign_;
    return node_.get_parameter("encoder.direction_sign").as_int() < 0 ? -1 : 1;
  }

 private:
  rclcpp::Node& node_;
  bool measured_ = false;
  double measured_mpc_ = 0;
  int measured_sign_ = 1;
};

}  // namespace laser_hhrp
