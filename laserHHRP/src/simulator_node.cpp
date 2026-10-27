// simulator_node: replaces nano_driver_node and laser_driver_node when there is no hardware
// (launch argument simulate:=true). It publishes the same topics they do:
//   nano/encoder, nano/trigger, nano/imu, imu/data, nano/status, nano/clock  (simulated Nano)
//   laser/raw, laser/profile, laser/status                                  (simulated laser)
// and listens to nano/command like the real Nano.
//
// The simulated roller stands still until a scan starts, then rolls at sim.speed_mps. The surface
// is flat with one rivet head 20 um proud at 20 mm along the scan axis (radius 2.5 mm), so a
// recorded scan can be checked (scripts/check_session.py). The simulated Nano clock runs 20 ppm
// slow, like a real crystal. With sync.hardware_trigger the laser makes one profile per trigger,
// otherwise it runs free at sync.software_rate_hz.
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "laser_hhrp/msg/clock_map.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"
#include "laser_hhrp/msg/laser_status.hpp"
#include "laser_hhrp/msg/nano_command.hpp"
#include "laser_hhrp/msg/nano_status.hpp"
#include "laser_hhrp/msg/raw_profile.hpp"
#include "laser_hhrp/msg/trigger_record.hpp"
#include "laser_hhrp/profile_cloud.hpp"

namespace laser_hhrp {

namespace {
constexpr double kClockRate = 1.0 - 20e-6;  // simulated Nano clock: 20 ppm slow
constexpr int64_t kSamplePeriodNs = 4167000;
constexpr double kGyroBias[3] = {0.012, -0.021, 0.006};  // typical MPU6050 bias (rad/s)
}  // namespace

class SimulatorNode : public rclcpp::Node {
 public:
  SimulatorNode() : Node("simulator") {
    speed_mps_ = declare_parameter("sim.speed_mps", 0.02);
    diameter_m_ = declare_parameter("encoder.roller_diameter_m", 0.030);
    counts_per_turn_ = declare_parameter("encoder.counts_per_turn", 4096);
    hardware_ = declare_parameter("sync.hardware_trigger", false);
    free_run_hz_ = declare_parameter("sync.software_rate_hz", 120.0);
    live_view_every_ = std::max<int64_t>(1, declare_parameter("laser.live_view_every", 10));

    const auto data = rclcpp::QoS(500);
    encoder_pub_ = create_publisher<msg::EncoderSample>("nano/encoder", data);
    trigger_pub_ = create_publisher<msg::TriggerRecord>("nano/trigger", data);
    imu_pub_ = create_publisher<msg::ImuSample>("nano/imu", data);
    ros_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<msg::NanoStatus>("nano/status", 10);
    clock_pub_ = create_publisher<msg::ClockMap>("nano/clock", rclcpp::QoS(10).transient_local());
    raw_pub_ = create_publisher<msg::RawProfile>("laser/raw", rclcpp::QoS(200));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("laser/profile", rclcpp::SensorDataQoS());
    laser_status_pub_ = create_publisher<msg::LaserStatus>("laser/status", 10);
    command_sub_ = create_subscription<msg::NanoCommand>(
        "nano/command", 10, [this](const msg::NanoCommand& c) { on_command(c); });

    t0_ns_ = last_ns_ = now_ns();
    // The exact clock mapping (a real Nano's comes from clock exchanges).
    msg::ClockMap c;
    c.header.stamp = now();
    c.device_ns = device_ns(t0_ns_);
    c.host_ns = t0_ns_;
    c.drift = 1.0 / kClockRate - 1.0;
    clock_pub_->publish(c);
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { step(); });
    RCLCPP_INFO(get_logger(), "simulated Nano and laser (%s sync)", hardware_ ? "hardware" : "software");
  }

 private:
  static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
  }
  int64_t device_ns(int64_t host_ns) const {
    return static_cast<int64_t>((host_ns - t0_ns_) * kClockRate) + 1000;
  }
  double position_m() const { return static_cast<double>(counts_) / counts_per_turn_ * M_PI * diameter_m_; }

  // ---- commands, as the Nano handles them ----
  void on_command(const msg::NanoCommand& c) {
    if (c.command == msg::NanoCommand::START) {  // like LaserTrigger::start on the Nano
      pitch_ = std::max<int>(1, c.counts_per_trigger);
      min_period_ns_ = static_cast<int64_t>(c.min_trigger_period_us) * 1000;
      anchor_ = count_;
      trigger_index_ = 0;
      triggering_ = true;
      fired_once_ = false;
    } else if (c.command == msg::NanoCommand::STOP) {
      triggering_ = false;
    } else if (c.command == msg::NanoCommand::ZERO) {
      count_ = 0;
      anchor_ = 0;
    }
  }

  // ---- every 1 ms: roll, trigger, free-running laser, samples, status ----
  // travel += speed * dt while triggering; counts = travel / (pi * d) * counts_per_turn.
  void step() {
    const int64_t now = now_ns(), prev = last_ns_;
    last_ns_ = now;
    // Roll while a scan triggers. Walk through every count in between at its own time, as the
    // real Nano reads the encoder every few 100 us.
    if (triggering_) travel_m_ += speed_mps_ * (now - prev) * 1e-9;
    const int64_t target = static_cast<int64_t>(std::floor(travel_m_ / (M_PI * diameter_m_) * counts_per_turn_));
    const int64_t steps = target - counts_;
    for (int64_t i = 1; i <= steps; ++i) {
      ++counts_;
      ++count_;
      encoder_read(prev + (now - prev) * i / steps);
    }
    // Free-running laser (software sync).
    if (!hardware_ && free_run_hz_ > 0) {
      const int64_t period = static_cast<int64_t>(1e9 / free_run_hz_);
      if (next_free_ns_ == 0) next_free_ns_ = now;
      for (; next_free_ns_ <= now; next_free_ns_ += period) make_profile(next_free_ns_);
    }
    if (now - last_sample_ns_ >= kSamplePeriodNs) {
      last_sample_ns_ = now;
      publish_samples(now);
    }
    if (now - last_status_ns_ >= 500000000) {
      last_status_ns_ = now;
      publish_status(now);
    }
  }

  // ---- trigger decision (same rules as arduino_nano/laser_trigger.h) ----
  void encoder_read(int64_t t_ns) {
    if (!triggering_) return;
    const int32_t travel = count_ - anchor_;
    const int32_t distance = std::abs(travel);
    if (distance < pitch_) return;
    if (fired_once_ && t_ns - last_trigger_ns_ < min_period_ns_) {
      held_back_ = true;
      return;
    }
    const int8_t direction = travel > 0 ? 1 : -1;
    const bool overspeed = held_back_ || distance >= 2 * pitch_;
    anchor_ += direction * (distance / pitch_) * pitch_;
    held_back_ = false;
    fired_once_ = true;
    last_trigger_ns_ = t_ns;
    msg::TriggerRecord r;
    r.header.stamp = rclcpp::Time(t_ns);
    r.device_time_ns = device_ns(t_ns);
    r.index = trigger_index_++;
    r.count = count_;
    r.direction = direction;
    r.flags = overspeed ? 1 : 0;
    trigger_pub_->publish(r);
    if (hardware_) make_profile(t_ns);  // the pulse reaches the laser
  }

  // ---- laser ----
  // z = standoff - 20 um where the point is within 2.5 mm of the rivet centre (y = 20 mm).
  void make_profile(int64_t t_ns) {
    auto p = std::make_unique<msg::RawProfile>();
    p->header.stamp = rclcpp::Time(now_ns());
    p->header.frame_id = "laser";
    p->profile_counter = profile_counter_++ & 0xFFFFFFu;
    p->shutter_open_s = std::fmod(device_ns(t_ns) * 1e-9, 128.0);
    const int n = 640;
    const double width = 0.010, standoff = 0.0525, rivet_y = 0.020, rivet_r = 0.0025, rivet_h = 20e-6;
    const double y = position_m();
    p->x_m.resize(n);
    p->z_m.resize(n);
    p->valid.assign(n, true);
    for (int i = 0; i < n; ++i) {
      const double x = -width / 2 + width * i / (n - 1);
      const double head = std::hypot(x, y - rivet_y) < rivet_r ? rivet_h : 0.0;
      p->x_m[i] = static_cast<float>(x);
      p->z_m[i] = static_cast<float>(standoff - head);
    }
    if (profile_counter_ % live_view_every_ == 0) cloud_pub_->publish(to_cloud(*p));
    raw_pub_->publish(std::move(p));
  }

  // ---- 240 Hz encoder + IMU samples: level, still device, gyro = bias + noise ----
  void publish_samples(int64_t now) {
    msg::EncoderSample e;
    e.header.stamp = rclcpp::Time(now);
    e.device_time_ns = device_ns(now);
    e.count = count_;
    e.raw = static_cast<uint16_t>(((counts_ % counts_per_turn_) + counts_per_turn_) % counts_per_turn_);
    encoder_pub_->publish(e);

    msg::ImuSample m;
    m.header = e.header;
    m.header.frame_id = "imu";
    m.device_time_ns = e.device_time_ns;
    m.accel_mps2 = {0.0f, 0.0f, 9.80665f};
    for (int i = 0; i < 3; ++i) m.gyro_radps[i] = static_cast<float>(kGyroBias[i] + 0.002 * noise());
    imu_pub_->publish(m);
    sensor_msgs::msg::Imu r;
    r.header = m.header;
    r.orientation_covariance[0] = -1;
    r.linear_acceleration.z = m.accel_mps2[2];
    r.angular_velocity.x = m.gyro_radps[0];
    r.angular_velocity.y = m.gyro_radps[1];
    r.angular_velocity.z = m.gyro_radps[2];
    ros_imu_pub_->publish(r);
  }

  void publish_status(int64_t now) {
    msg::NanoStatus s;
    s.header.stamp = rclcpp::Time(now);
    s.encoder_ok = true;
    s.imu_ok = true;
    s.triggering = triggering_;
    s.triggers = trigger_index_;
    status_pub_->publish(s);
    msg::LaserStatus l;
    l.header.stamp = s.header.stamp;
    l.connected = true;
    l.message = "simulated";
    laser_status_pub_->publish(l);
  }

  // Repeatable noise with unit variance.
  double noise() {
    lcg_ = lcg_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return (static_cast<double>(lcg_ >> 11) / 9007199254740992.0 - 0.5) * 3.4641016;
  }

  double speed_mps_, diameter_m_, free_run_hz_;
  int64_t counts_per_turn_, live_view_every_;
  bool hardware_;
  int64_t t0_ns_, last_ns_, next_free_ns_ = 0, last_sample_ns_ = 0, last_status_ns_ = 0;
  double travel_m_ = 0;
  int64_t counts_ = 0;       // total counts rolled
  int32_t count_ = 0;        // the Nano's count (zeroable)
  bool triggering_ = false, fired_once_ = false, held_back_ = false;
  int32_t pitch_ = 4, anchor_ = 0;
  int64_t min_period_ns_ = 0, last_trigger_ns_ = 0;
  uint32_t trigger_index_ = 0, profile_counter_ = 0;
  uint64_t lcg_ = 42;
  rclcpp::Publisher<msg::EncoderSample>::SharedPtr encoder_pub_;
  rclcpp::Publisher<msg::TriggerRecord>::SharedPtr trigger_pub_;
  rclcpp::Publisher<msg::ImuSample>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_;
  rclcpp::Publisher<msg::NanoStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::ClockMap>::SharedPtr clock_pub_;
  rclcpp::Publisher<msg::RawProfile>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<msg::LaserStatus>::SharedPtr laser_status_pub_;
  rclcpp::Subscription<msg::NanoCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
