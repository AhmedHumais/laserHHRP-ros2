// synchronizer_node (layer 3: synchronization): one ScanFrame per laser profile, with its position
// along the scan axis (encoder) and the IMU at the same instant.
//
// Two modes (parameter sync.hardware_trigger):
//   true  (hardware sync) the Nano triggers the laser every N counts; each profile is paired with
//         its trigger record by number (frame_pairer.hpp).
//   false (software sync) the laser runs free; each profile gets the encoder count interpolated at
//         its exposure time between the Nano's encoder samples (software_pairer.hpp).
// Pairing and IMU interpolation use the Nano clock; frames are stamped in Pi time (nano/clock).
// Frames are made only while a scan runs (scan/status).
//
// Subscribes  nano/trigger, nano/encoder, laser/raw, imu/corrected, nano/clock, scan/status,
//             scan/motion, odometry/laser_pose, calibration/status
// Publishes
//   scan/frame  laser_hhrp/ScanFrame          one per profile
//   scan/pose   geometry_msgs/PoseStamped     laser pose at each frame (same stamp), once the
//                                             extrinsics are calibrated
//   scan/sync   laser_hhrp/SyncStats          10 Hz
#include <deque>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "laser_hhrp/encoder_scale.hpp"
#include "laser_hhrp/frame_pairer.hpp"
#include "laser_hhrp/msg/clock_map.hpp"
#include "laser_hhrp/msg/device_pose.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"
#include "laser_hhrp/msg/motion.hpp"
#include "laser_hhrp/msg/scan_frame.hpp"
#include "laser_hhrp/msg/scan_status.hpp"
#include "laser_hhrp/msg/sync_stats.hpp"
#include "laser_hhrp/msg/trigger_record.hpp"
#include "laser_hhrp/pose.hpp"
#include "laser_hhrp/software_pairer.hpp"

namespace laser_hhrp {

namespace {
constexpr int64_t kImuWaitNs = 100000000;   // a frame waits up to 100 ms for the IMU after it
constexpr int64_t kHistoryNs = 2000000000;  // IMU samples / poses kept for interpolation
}  // namespace

using Pair = FramePairer::Pair;
using ScanStatus = msg::ScanStatus;

class SynchronizerNode : public rclcpp::Node {
 public:
  // ---- setup ----
  // Callbacks only collect data: triggers / encoder references / profiles go into the pairer of
  // the active mode (only while a scan runs), IMU samples and poses into 2 s histories. The
  // pairing itself runs in step() every 2 ms.
  SynchronizerNode() : Node("synchronizer"), scale_(*this) {
    hardware_ = declare_parameter("sync.hardware_trigger", false);
    software_.set_latency(declare_parameter("sync.profile_latency_s", 0.002));

    const auto data = rclcpp::QoS(500);
    frame_pub_ = create_publisher<msg::ScanFrame>("scan/frame", rclcpp::QoS(200));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("scan/pose", 200);
    stats_pub_ = create_publisher<msg::SyncStats>("scan/sync", 10);

    clock_sub_ = create_subscription<msg::ClockMap>(
        "nano/clock", rclcpp::QoS(10).transient_local(), [this](const msg::ClockMap& c) { clock_ = c; have_clock_ = true; });
    calibration_sub_ = create_subscription<msg::CalibrationStatus>(
        "calibration/status", rclcpp::QoS(1).transient_local(),
        [this](const msg::CalibrationStatus& c) { scale_.set_calibration(c); });
    motion_sub_ = create_subscription<msg::Motion>(
        "scan/motion", 10, [this](const msg::Motion& m) { last_count_ = m.encoder_count; });
    scan_sub_ = create_subscription<ScanStatus>(
        "scan/status", 10, [this](const ScanStatus& s) { follow_scan(s.state); });
    imu_sub_ = create_subscription<msg::ImuSample>("imu/corrected", data, [this](const msg::ImuSample& m) {
      imu_history_.push_back(m);
      while (m.device_time_ns - imu_history_.front().device_time_ns > kHistoryNs) imu_history_.pop_front();
    });
    pose_sub_ = create_subscription<msg::DevicePose>("odometry/laser_pose", data, [this](const msg::DevicePose& p) {
      if (!pose_history_.empty() && p.device_time_ns <= pose_history_.back().device_time_ns) pose_history_.clear();
      pose_history_.push_back(p);
      while (p.device_time_ns - pose_history_.front().device_time_ns > kHistoryNs) pose_history_.pop_front();
    });
    trigger_sub_ = create_subscription<msg::TriggerRecord>("nano/trigger", data, [this](const msg::TriggerRecord& t) {
      if (active_ && hardware_) pairer_.add_trigger(to_trigger(t), now_ns());
    });
    encoder_sub_ = create_subscription<msg::EncoderSample>("nano/encoder", data, [this](const msg::EncoderSample& e) {
      if (active_ && !hardware_) add_software_reference(e);
    });
    raw_sub_ = create_subscription<msg::RawProfile>("laser/raw", rclcpp::QoS(200), [this](msg::RawProfile::ConstSharedPtr p) {
      if (!active_) return;
      Raw r;
      r.stamp_ns = rclcpp::Time(p->header.stamp).nanoseconds();
      r.profile_counter = p->profile_counter;
      r.shutter_open_s = p->shutter_open_s;
      r.msg = p;
      if (hardware_) pairer_.add_profile(r);
      else software_.add_profile(r);
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { step(); });
  }

 private:
  static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
  }
  static Trigger to_trigger(const msg::TriggerRecord& t) {
    Trigger out;
    out.device_ns = t.device_time_ns;
    out.index = t.index;
    out.count = t.count;
    out.direction = t.direction;
    out.flags = t.flags;
    return out;
  }

  // ---- software sync input ----
  // Each encoder sample is a reference with two times: Nano time (device_ns) and Pi time
  // (to_host). The pairer interpolates the count at each profile's Pi time.
  void add_software_reference(const msg::EncoderSample& e) {
    Trigger ref;
    ref.device_ns = e.device_time_ns;
    ref.count = e.count;
    ref.direction = static_cast<int8_t>(e.count > last_ref_count_ ? 1 : e.count < last_ref_count_ ? -1 : 0);
    last_ref_count_ = e.count;
    software_.add_record(ref, to_host(e.device_time_ns));
  }

  // ---- every 2 ms: completed pairs -> frames ----
  void step() {
    const int64_t now = now_ns();
    if (active_) {
      for (const Pair& p : hardware_ ? pairer_.take(now) : software_.take(now)) ready_.push_back(p);
    }
    // A frame goes out once the IMU sample after it has arrived (so the IMU is interpolated at
    // that instant), or after kImuWaitNs without it (imu_valid false).
    while (!ready_.empty()) {
      const Pair& pair = ready_.front();
      const bool imu_ready = !imu_history_.empty() &&
                             imu_history_.back().device_time_ns >= pair.trigger.device_ns;
      if ((!imu_ready || !pose_ready(pair.trigger.device_ns)) && now - pair.arrival_ns < kImuWaitNs) break;
      publish(pair);
      ready_.pop_front();
    }
    if (now - last_stats_ns_ >= 100000000) publish_stats(now);
  }

  // ---- scan state ----
  // start: reset everything, remember the count at the start (position 0) and the encoder
  // scale; resume: new trigger numbering, so the hardware pairer locks again; stop: flush.
  void follow_scan(uint8_t state) {
    if (state == scan_state_) return;
    const bool was_on = scan_state_ == ScanStatus::SCANNING || scan_state_ == ScanStatus::PAUSED;
    if (state == ScanStatus::SCANNING && !was_on) {
      ready_.clear();
      pairer_.reset();
      software_.reset();
      start_count_ = last_count_;
      metres_per_count_ = scale_.metres_per_count();
      sign_ = scale_.direction_sign();
      frames_ = 0;
      last_position_m_ = 0;
      active_ = true;
    } else if (state == ScanStatus::SCANNING) {
      pairer_.reset();
      software_.reset();
    } else if (state != ScanStatus::PAUSED && was_on) {
      for (const Pair& pair : ready_) publish(pair);
      ready_.clear();
      active_ = false;
    }
    scan_state_ = state;
  }

  // ---- one frame ----
  // position_m = sign * (count - start_count) * metres_per_count
  // IMU and pose are interpolated at the trigger's Nano time (same clock: exact).
  void publish(const Pair& pair) {
    auto f = std::make_unique<msg::ScanFrame>();
    f->header.stamp = rclcpp::Time(to_host(pair.trigger.device_ns));
    f->header.frame_id = "laser";
    f->index = frames_;  // unique within the scan (trigger numbers restart after a pause)
    f->encoder_count = pair.trigger.count;
    f->position_m = sign_ * (pair.trigger.count - start_count_) * metres_per_count_;
    f->direction = static_cast<int8_t>(sign_ * pair.trigger.direction);
    f->imu_valid = imu_at(pair.trigger.device_ns, *f);
    f->flags = static_cast<uint8_t>((pair.trigger.flags & 1) | (pair.timing_mismatch ? 2 : 0));
    f->x_m = pair.profile.msg->x_m;
    f->z_m = pair.profile.msg->z_m;
    f->valid = pair.profile.msg->valid;
    last_position_m_ = static_cast<float>(f->position_m);
    ++frames_;
    Pose p;
    if (pose_at(pair.trigger.device_ns, p)) {  // the laser pose at the same instant, same stamp
      geometry_msgs::msg::PoseStamped out;
      out.header.stamp = f->header.stamp;
      out.header.frame_id = "odom";
      out.pose.position.x = p.position_m[0];
      out.pose.position.y = p.position_m[1];
      out.pose.position.z = p.position_m[2];
      out.pose.orientation.w = p.orientation[0];
      out.pose.orientation.x = p.orientation[1];
      out.pose.orientation.y = p.orientation[2];
      out.pose.orientation.z = p.orientation[3];
      pose_pub_->publish(out);
    }
    frame_pub_->publish(std::move(f));
  }

  // ---- interpolation helpers (Nano time) ----
  // value(t) = a + k * (b - a), k = (t - t_a) / (t_b - t_a), a and b the samples around t;
  // poses: position linear, orientation slerp (pose.hpp).
  /// IMU linearly interpolated at Nano time t; false if no samples bracket t.
  bool imu_at(int64_t t, msg::ScanFrame& f) const {
    for (size_t i = 1; i < imu_history_.size(); ++i) {
      const auto& a = imu_history_[i - 1];
      const auto& b = imu_history_[i];
      if (a.device_time_ns > t || b.device_time_ns < t) continue;
      const double span = static_cast<double>(b.device_time_ns - a.device_time_ns);
      const double k = span > 0 ? (t - a.device_time_ns) / span : 0.0;
      for (int j = 0; j < 3; ++j) {
        f.accel_mps2[j] = static_cast<float>(a.accel_mps2[j] + k * (b.accel_mps2[j] - a.accel_mps2[j]));
        f.gyro_radps[j] = static_cast<float>(a.gyro_radps[j] + k * (b.gyro_radps[j] - a.gyro_radps[j]));
      }
      return true;
    }
    return false;
  }

  /// Wait for the pose after t, unless no poses come (extrinsics not calibrated).
  bool pose_ready(int64_t t) const {
    return pose_history_.empty() || pose_history_.back().device_time_ns >= t ||
           t - pose_history_.back().device_time_ns > 200000000;
  }

  static Pose to_pose(const msg::DevicePose& d) {
    Pose p;
    p.position_m = {d.pose.position.x, d.pose.position.y, d.pose.position.z};
    p.orientation = {d.pose.orientation.w, d.pose.orientation.x, d.pose.orientation.y, d.pose.orientation.z};
    return p;
  }
  bool pose_at(int64_t t, Pose& out) const {
    for (size_t i = 1; i < pose_history_.size(); ++i) {
      const auto& a = pose_history_[i - 1];
      const auto& b = pose_history_[i];
      if (a.device_time_ns > t || b.device_time_ns < t) continue;
      const double span = static_cast<double>(b.device_time_ns - a.device_time_ns);
      out = interpolate(to_pose(a), to_pose(b), span > 0 ? (t - a.device_time_ns) / span : 0.0);
      return true;
    }
    return false;
  }

  // pi = host_ns + (nano - device_ns) * (1 + drift), from the latest nano/clock.
  int64_t to_host(int64_t device_ns) const {
    if (!have_clock_) return now_ns();
    return clock_.host_ns + static_cast<int64_t>((device_ns - clock_.device_ns) * (1.0 + clock_.drift));
  }

  void publish_stats(int64_t now) {
    last_stats_ns_ = now;
    msg::SyncStats s;
    s.header.stamp = this->now();
    s.hardware_trigger = hardware_;
    s.locked = hardware_ ? pairer_.locked() : software_.stats().paired > 0;
    s.frames = frames_;
    s.missing_profiles = pairer_.stats().missing_profiles;
    s.missing_triggers = pairer_.stats().missing_triggers;
    s.timing_mismatches = pairer_.stats().timing_mismatches;
    s.unplaced = software_.stats().unplaced;
    s.last_position_m = last_position_m_;
    stats_pub_->publish(s);
  }

  EncoderScale scale_;
  bool hardware_ = false, active_ = false, have_clock_ = false;
  FramePairer pairer_;
  SoftwarePairer software_;
  std::deque<Pair> ready_;
  std::deque<msg::ImuSample> imu_history_;
  std::deque<msg::DevicePose> pose_history_;
  msg::ClockMap clock_;
  uint8_t scan_state_ = ScanStatus::IDLE;
  double metres_per_count_ = 0;
  int sign_ = 1;
  int32_t start_count_ = 0, last_count_ = 0, last_ref_count_ = 0;
  uint32_t frames_ = 0;
  float last_position_m_ = 0;
  int64_t last_stats_ns_ = 0;
  rclcpp::Publisher<msg::ScanFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<msg::SyncStats>::SharedPtr stats_pub_;
  rclcpp::Subscription<msg::ClockMap>::SharedPtr clock_sub_;
  rclcpp::Subscription<msg::CalibrationStatus>::SharedPtr calibration_sub_;
  rclcpp::Subscription<msg::Motion>::SharedPtr motion_sub_;
  rclcpp::Subscription<ScanStatus>::SharedPtr scan_sub_;
  rclcpp::Subscription<msg::ImuSample>::SharedPtr imu_sub_;
  rclcpp::Subscription<msg::DevicePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<msg::TriggerRecord>::SharedPtr trigger_sub_;
  rclcpp::Subscription<msg::EncoderSample>::SharedPtr encoder_sub_;
  rclcpp::Subscription<msg::RawProfile>::SharedPtr raw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::SynchronizerNode>());
  rclcpp::shutdown();
  return 0;
}
