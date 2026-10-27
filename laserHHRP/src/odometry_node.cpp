// odometry_node (layer 2): pose of the device relative to the zero position, from the calibrated
// IMU and the encoder (Kalman filter on the rolled distance, roller slip detection; see
// odometry.hpp). Active once the extrinsics are calibrated.
//
// Subscribes  imu/corrected, nano/encoder, nano/clock, calibration/status
// Publishes
//   base/pose            geometry_msgs/PoseStamped  frame "odom", IMU rate, Pi time
//   odometry/laser_pose  laser_hhrp/DevicePose      laser pose in Nano time (for the synchronizer)
//   odometry/slipping    std_msgs/Bool              roller turns without the device moving
// Service  odometry/zero (std_srvs/Trigger): the current pose becomes the zero position.
#include <deque>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "laser_hhrp/calibration_data.hpp"
#include "laser_hhrp/msg/calibration_status.hpp"
#include "laser_hhrp/msg/clock_map.hpp"
#include "laser_hhrp/msg/device_pose.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"
#include "laser_hhrp/odometry.hpp"
#include "laser_hhrp/pose.hpp"

namespace laser_hhrp {

geometry_msgs::msg::Pose to_ros(const Vec3& p, const Quat& q) {
  geometry_msgs::msg::Pose out;
  out.position.x = p[0];
  out.position.y = p[1];
  out.position.z = p[2];
  out.orientation.w = q[0];
  out.orientation.x = q[1];
  out.orientation.y = q[2];
  out.orientation.z = q[3];
  return out;
}

class OdometryNode : public rclcpp::Node {
 public:
  // ---- setup: topics and the zero service; the filter stays off until extrinsics exist ----
  OdometryNode() : Node("odometry") {
    file_ = declare_parameter("calibration.file", "~/laser_hhrp/calibration.yaml");
    if (!file_.empty() && file_[0] == '~') file_ = std::string(getenv("HOME")) + file_.substr(1);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("base/pose", 50);
    laser_pose_pub_ = create_publisher<msg::DevicePose>("odometry/laser_pose", rclcpp::QoS(500));
    slip_pub_ = create_publisher<std_msgs::msg::Bool>("odometry/slipping", rclcpp::QoS(1).transient_local());
    clock_sub_ = create_subscription<msg::ClockMap>(
        "nano/clock", rclcpp::QoS(10).transient_local(), [this](const msg::ClockMap& c) { clock_ = c; have_clock_ = true; });
    calibration_sub_ = create_subscription<msg::CalibrationStatus>(
        "calibration/status", rclcpp::QoS(1).transient_local(),
        [this](const msg::CalibrationStatus& c) { follow_calibration(c); });
    // Both streams are processed in Nano-time order (the encoder is a little ahead of the IMU).
    encoder_sub_ = create_subscription<msg::EncoderSample>(
        "nano/encoder", rclcpp::QoS(500), [this](const msg::EncoderSample& e) { counts_.push_back(e); });
    imu_sub_ = create_subscription<msg::ImuSample>(
        "imu/corrected", rclcpp::QoS(500), [this](const msg::ImuSample& m) { on_imu(m); });
    zero_srv_ = create_service<std_srvs::srv::Trigger>(
        "odometry/zero", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = filter_.configured();
          res->message = res->success ? "pose zeroed" : "no pose: calibrate the extrinsics first";
          if (res->success) filter_.zero();
        });
    publish_slip(false);
  }

 private:
  // ---- calibration changes ----
  /// A changed calibration: reload the file; without extrinsics there is no pose.
  void follow_calibration(const msg::CalibrationStatus& c) {
    const std::string key = std::to_string(c.state) + (c.extrinsics_calibrated ? "+" : "-") + c.message;
    if (key == calibration_key_) return;
    calibration_key_ = key;
    if (!c.extrinsics_calibrated || c.state != msg::CalibrationStatus::CALIBRATED) {
      if (filter_.configured()) RCLCPP_INFO(get_logger(), "pose paused: extrinsics not calibrated");
      filter_ = Odometry();
      return;
    }
    CalibrationData data;
    std::string error;
    if (!calibration_file::load(file_, data, error) || !data.extrinsics.calibrated) {
      RCLCPP_WARN(get_logger(), "no extrinsics in %s: %s", file_.c_str(), error.c_str());
      return;
    }
    filter_.configure(data.extrinsics);
    extrinsics_ = data.extrinsics;
    RCLCPP_INFO(get_logger(), "pose active (odom = the current position; %.3f um/count)",
                data.extrinsics.metres_per_count * 1e6);
  }

  // ---- one IMU sample = one filter step ----
  // The filter (odometry.hpp) needs encoder and IMU in Nano-time order: encoder samples are
  // queued and fed up to the IMU sample's time first, then the IMU sample itself.
  void on_imu(const msg::ImuSample& m) {
    // Encoder samples up to this IMU sample first.
    while (!counts_.empty() && counts_.front().device_time_ns <= m.device_time_ns) {
      filter_.encoder(counts_.front().count, counts_.front().device_time_ns);
      counts_.pop_front();
    }
    while (counts_.size() > 100) counts_.pop_front();
    if (!filter_.configured()) return;
    filter_.imu({m.gyro_radps[0], m.gyro_radps[1], m.gyro_radps[2]},
                {m.accel_mps2[0], m.accel_mps2[1], m.accel_mps2[2]}, m.device_time_ns);
    const OdometryPose& p = filter_.pose();

    // base/pose for RViz/tools: stamped in Pi time with the clock mapping
    // pi = host_ns + (nano - device_ns) * (1 + drift).
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = have_clock_ ? rclcpp::Time(clock_.host_ns + static_cast<int64_t>(
                                         (m.device_time_ns - clock_.device_ns) * (1.0 + clock_.drift)))
                                   : now();
    out.header.frame_id = "odom";
    out.pose = to_ros(p.position_m, p.orientation);
    pose_pub_->publish(out);

    // The laser pose (base pose x laser extrinsics) in Nano time.
    Pose base;
    base.position_m = p.position_m;
    base.orientation = p.orientation;
    const Pose laser = laser_pose(base, extrinsics_);
    msg::DevicePose l;
    l.device_time_ns = m.device_time_ns;
    l.pose = to_ros(laser.position_m, laser.orientation);
    laser_pose_pub_->publish(l);

    // Slip state changes are logged and latched on odometry/slipping (a warning in the UI).
    if (p.slipping != slipping_) {
      if (p.slipping)
        RCLCPP_WARN(get_logger(), "roller slip: encoder ignored (%.1f mm rejected so far)", p.slip_m * 1e3);
      else
        RCLCPP_INFO(get_logger(), "roller slip over");
      publish_slip(p.slipping);
    }
  }

  void publish_slip(bool slipping) {
    slipping_ = slipping;
    std_msgs::msg::Bool b;
    b.data = slipping;
    slip_pub_->publish(b);
  }

  std::string file_, calibration_key_;
  Odometry filter_;
  Extrinsics extrinsics_;
  msg::ClockMap clock_;
  bool have_clock_ = false, slipping_ = false;
  std::deque<msg::EncoderSample> counts_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<msg::DevicePose>::SharedPtr laser_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slip_pub_;
  rclcpp::Subscription<msg::ClockMap>::SharedPtr clock_sub_;
  rclcpp::Subscription<msg::CalibrationStatus>::SharedPtr calibration_sub_;
  rclcpp::Subscription<msg::EncoderSample>::SharedPtr encoder_sub_;
  rclcpp::Subscription<msg::ImuSample>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
