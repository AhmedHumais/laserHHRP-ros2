// laser_driver_node (layer 1: hardware interface): the scanCONTROL laser profiler.
//
// Publishes
//   laser/raw      laser_hhrp/RawProfile        every profile, stamped with its exposure time
//   laser/profile  sensor_msgs/PointCloud2      every Nth profile, for the live view in RViz
//   laser/status   laser_hhrp/LaserStatus       connected or not (1 Hz)
// Trigger: sync.hardware_trigger true = one profile per Nano pulse on In1; false = free running
// at sync.software_rate_hz. A missing scanner is retried every 2 s.
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include "laser_hhrp/msg/laser_status.hpp"
#include "laser_hhrp/msg/raw_profile.hpp"
#include "laser_hhrp/profile_cloud.hpp"
#include "laser_hhrp/scancontrol.hpp"
#include "laser_hhrp/scanner_clock.hpp"

namespace laser_hhrp {

class LaserDriverNode : public rclcpp::Node {
 public:
  // ---- setup: scanner settings from the parameters ----
  LaserDriverNode() : Node("laser_driver") {
    settings_.interface = declare_parameter("laser.interface", "");
    settings_.resolution = declare_parameter("laser.resolution", 1280);
    settings_.exposure_time_us = declare_parameter("laser.exposure_time_us", 100);
    settings_.external_trigger = declare_parameter("sync.hardware_trigger", false);
    // Software sync: the scanner runs free. Its period is exposure + idle, so
    // idle = 1/rate - exposure (at least 10 us).
    if (!settings_.external_trigger) {  // free running: period = exposure + idle
      expected_hz_ = std::max(1.0, declare_parameter("sync.software_rate_hz", 120.0));
      settings_.idle_time_us = static_cast<uint32_t>(
          std::max(10.0, 1e6 / expected_hz_ - settings_.exposure_time_us));
    }
    live_view_every_ = std::max<int64_t>(1, declare_parameter("laser.live_view_every", 10));

    raw_pub_ = create_publisher<msg::RawProfile>("laser/raw", rclcpp::QoS(200));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("laser/profile", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<msg::LaserStatus>("laser/status", 10);
    connect();
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { poll(); });
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_status(); });
  }

 private:
  // ---- connect (at start, then every 2 s while disconnected) ----
  void connect() {
    clock_.reset();  // a reconnected scanner may have restarted its clock
    if (scanner_.connect(settings_, message_)) {
      message_ = "connected";
      RCLCPP_INFO(get_logger(), "scanner connected (%s trigger)",
                  settings_.external_trigger ? "external In1" : "internal");
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "scanner: %s", message_.c_str());
    }
    last_attempt_ = now();
  }

  // ---- main loop (every 1 ms): take all queued profiles and publish them ----
  void poll() {
    if (!scanner_.connected()) {
      if ((now() - last_attempt_).seconds() >= 2.0) connect();
      return;
    }
    LaserProfile p;
    std::string error;
    while (scanner_.read(p, error)) {
      auto m = std::make_unique<msg::RawProfile>();
      // Exposure midpoint in Pi time from the scanner's own clock (removes network jitter).
      m->header.stamp = rclcpp::Time(clock_.map(p.shutter_open_s, p.shutter_close_s, p.arrival_ns));
      m->header.frame_id = "laser";
      m->profile_counter = p.profile_counter;
      m->shutter_open_s = p.shutter_open_s;
      m->x_m = p.x_m;
      m->z_m = p.z_m;
      m->valid = p.valid;
      count_rate(p);
      if (++count_ % live_view_every_ == 0) cloud_pub_->publish(to_cloud(*m));
      raw_pub_->publish(std::move(m));
    }
    // The SDK reported the scanner lost: drop the connection, poll() reconnects in 2 s.
    if (!error.empty()) {
      message_ = error;
      RCLCPP_ERROR(get_logger(), "scanner: %s", error.c_str());
      scanner_.disconnect();
    }
    report_rate();
  }

  // ---- rate report ----
  // made = (counter span + 1) / seconds: the scanner numbers every profile it makes, so gaps in
  // the counter are profiles lost on the way; received < made means network losses.
  void count_rate(const LaserProfile& p) {
    if (window_received_++ == 0) {
      window_first_counter_ = p.profile_counter;
      window_first_lost_ = p.lost_profiles;
    }
    window_last_counter_ = p.profile_counter;
    window_last_lost_ = p.lost_profiles;
  }
  void report_rate() {
    if (window_start_.nanoseconds() == 0) window_start_ = now();
    const double seconds = (now() - window_start_).seconds();
    if (seconds < 5.0) return;
    if (window_received_ == 0) {
      RCLCPP_WARN(get_logger(), "no profiles in %.0f s%s", seconds,
                  settings_.external_trigger ? " (no trigger pulses? roll the roller during a scan)" : "");
    } else {
      const uint32_t span = (window_last_counter_ - window_first_counter_) & 0xFFFFFFu;
      const double made = window_received_ > 1 ? (span + 1.0) / seconds : window_received_ / seconds;
      RCLCPP_INFO(get_logger(), "profiles: %.1f /s received, %.1f /s made by the scanner, %u lost",
                  window_received_ / seconds, made, window_last_lost_ - window_first_lost_);
      // Asked for more than it can do (2900, full field: > ~140 Hz) it falls to ~18 profiles/s.
      if (expected_hz_ > 0 && made < 0.8 * expected_hz_)
        RCLCPP_ERROR(get_logger(), "scanner makes %.1f profiles/s instead of %.1f: lower "
                     "sync.software_rate_hz (2900, full field: max ~140)", made, expected_hz_);
    }
    window_start_ = now();
    window_received_ = 0;
  }

  void publish_status() {
    msg::LaserStatus s;
    s.header.stamp = now();
    s.connected = scanner_.connected();
    s.message = message_;
    status_pub_->publish(s);
  }

  ScanControl scanner_;
  ScanControlSettings settings_;
  ScannerClock clock_;
  double expected_hz_ = 0;
  int64_t live_view_every_ = 10;
  uint64_t count_ = 0;
  std::string message_;
  rclcpp::Time last_attempt_;
  rclcpp::Time window_start_{0, 0, RCL_ROS_TIME};
  uint32_t window_received_ = 0, window_first_counter_ = 0, window_last_counter_ = 0;
  uint32_t window_first_lost_ = 0, window_last_lost_ = 0;
  rclcpp::Publisher<msg::RawProfile>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<msg::LaserStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::LaserDriverNode>());
  rclcpp::shutdown();
  return 0;
}
