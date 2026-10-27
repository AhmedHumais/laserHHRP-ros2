// acquisition_node (layer 2: data acquisition): calibrated IMU samples and the motion state.
//
// Subscribes  nano/encoder, nano/imu (raw), calibration/status
// Publishes
//   imu/corrected  laser_hhrp/ImuSample  gyro bias and accelerometer offset/scale removed
//   scan/motion    laser_hhrp/Motion     roller speed, device tilt, vibration (for monitoring + UI)
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "laser_hhrp/encoder_scale.hpp"
#include "laser_hhrp/msg/calibration_status.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"
#include "laser_hhrp/msg/motion.hpp"
#include "laser_hhrp/rotation.hpp"

namespace laser_hhrp {

class AcquisitionNode : public rclcpp::Node {
 public:
  // ---- setup: topics; until a calibration arrives nothing is corrected (scale 1, no bias) ----
  AcquisitionNode() : Node("acquisition"), scale_(*this) {
    imu_pub_ = create_publisher<msg::ImuSample>("imu/corrected", rclcpp::QoS(500));
    motion_pub_ = create_publisher<msg::Motion>("scan/motion", 10);
    calibration_sub_ = create_subscription<msg::CalibrationStatus>(
        "calibration/status", rclcpp::QoS(1).transient_local(),
        [this](const msg::CalibrationStatus& c) { calibration_ = c; scale_.set_calibration(c); });
    encoder_sub_ = create_subscription<msg::EncoderSample>(
        "nano/encoder", rclcpp::QoS(500), [this](const msg::EncoderSample& e) { on_encoder(e); });
    imu_sub_ = create_subscription<msg::ImuSample>(
        "nano/imu", rclcpp::QoS(500), [this](const msg::ImuSample& m) { on_imu(m); });
    calibration_.accel_scale = {1, 1, 1};
    calibration_.gravity_imu_mps2 = {0, 0, 9.80665};
  }

 private:
  // ---- encoder: speed ----
  // speed = (count - previous count) * metres_per_count / (t - previous t), times in Nano clock
  // (exact sample spacing, no USB jitter).
  void on_encoder(const msg::EncoderSample& e) {
    if (motion_.has_encoder) {
      const double dt = (e.device_time_ns - last_encoder_.device_time_ns) * 1e-9;
      if (dt > 0)
        motion_.speed_mps =
            static_cast<float>((e.count - last_encoder_.count) * scale_.metres_per_count() / dt);
    }
    last_encoder_ = e;
    motion_.has_encoder = true;
    motion_.encoder_count = e.count;
    publish_motion();
  }

  // ---- IMU: calibration, tilt, vibration ----
  void on_imu(msg::ImuSample m) {
    // Calibration: gyro minus bias, accelerometer (minus offset) x scale.
    for (int i = 0; i < 3; ++i) {
      m.gyro_radps[i] = static_cast<float>(m.gyro_radps[i] - calibration_.gyro_bias_radps[i]);
      m.accel_mps2[i] = static_cast<float>((m.accel_mps2[i] - calibration_.accel_bias_mps2[i]) *
                                           calibration_.accel_scale[i]);
    }
    imu_pub_->publish(m);
    // tilt = angle between the accelerometer now and gravity measured at rest during the gyro
    // calibration (so any IMU mounting works); rotation = |gyro| as a vibration indicator.
    const Vec3 g_ref = calibration_.state == msg::CalibrationStatus::CALIBRATION_REQUIRED
                           ? Vec3{0, 0, 9.80665}
                           : Vec3{calibration_.gravity_imu_mps2[0], calibration_.gravity_imu_mps2[1],
                                  calibration_.gravity_imu_mps2[2]};
    motion_.tilt_rad = static_cast<float>(
        angle_between(Vec3{m.accel_mps2[0], m.accel_mps2[1], m.accel_mps2[2]}, g_ref));
    motion_.rotation_radps = static_cast<float>(
        norm(Vec3{m.gyro_radps[0], m.gyro_radps[1], m.gyro_radps[2]}));
    publish_motion();
  }

  void publish_motion() {
    motion_.header.stamp = now();
    motion_pub_->publish(motion_);
  }

  EncoderScale scale_;
  msg::CalibrationStatus calibration_;
  msg::Motion motion_;
  msg::EncoderSample last_encoder_;
  rclcpp::Publisher<msg::ImuSample>::SharedPtr imu_pub_;
  rclcpp::Publisher<msg::Motion>::SharedPtr motion_pub_;
  rclcpp::Subscription<msg::CalibrationStatus>::SharedPtr calibration_sub_;
  rclcpp::Subscription<msg::EncoderSample>::SharedPtr encoder_sub_;
  rclcpp::Subscription<msg::ImuSample>::SharedPtr imu_sub_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::AcquisitionNode>());
  rclcpp::shutdown();
  return 0;
}
