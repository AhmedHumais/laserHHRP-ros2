// calibration_node: the device's IMU calibration, saved to a YAML file (calibration.file).
//
//   gyro calibration       hold the device still ~2 s: gyro bias (gyro_calibrator.hpp)
//   extrinsic calibration  rolling procedure on a flat surface: IMU <-> base rotation, metres per
//                          encoder count and counting direction (extrinsic_calibrator.hpp)
// At start the file is loaded; without a gyro calibration the state is CALIBRATION_REQUIRED and
// the scan controller refuses to scan.
//
// Subscribes  nano/imu (raw), nano/encoder
// Publishes   calibration/status  laser_hhrp/CalibrationStatus (latched, on change and 2 Hz)
// Services (std_srvs/Trigger)
//   calibration/start       gyro calibration
//   calibration/reset       delete the calibration (a running scan stops)
//   calibration/extrinsics  rolling procedure; prompts appear in the status message
#include <cmath>
#include <ctime>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "laser_hhrp/calibration_data.hpp"
#include "laser_hhrp/extrinsic_calibrator.hpp"
#include "laser_hhrp/gyro_calibrator.hpp"
#include "laser_hhrp/msg/calibration_status.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"

namespace laser_hhrp {

using Status = msg::CalibrationStatus;
using Trigger = std_srvs::srv::Trigger;

std::string iso_now() {
  const std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
  return buf;
}

class CalibrationNode : public rclcpp::Node {
 public:
  // ---- setup: parameters, topics, services; then load the calibration file ----
  CalibrationNode() : Node("calibration") {
    file_ = declare_parameter("calibration.file", "~/laser_hhrp/calibration.yaml");
    if (!file_.empty() && file_[0] == '~') file_ = std::string(getenv("HOME")) + file_.substr(1);
    GyroCalibrationSettings s;
    s.duration_s = declare_parameter("calibration.duration_s", 2.0);
    s.max_gyro_std_radps = declare_parameter("calibration.max_gyro_std_radps", 0.02);
    gyro_ = GyroCalibrator(s);
    // For the roll reports: the nominal scale from the configured roller.
    nominal_mpc_ = M_PI * declare_parameter("encoder.roller_diameter_m", 0.030) /
                   static_cast<double>(declare_parameter("encoder.counts_per_turn", 4096));
    counts_per_turn_ = static_cast<double>(get_parameter("encoder.counts_per_turn").as_int());

    status_pub_ = create_publisher<Status>("calibration/status", rclcpp::QoS(1).transient_local());
    encoder_sub_ = create_subscription<msg::EncoderSample>(
        "nano/encoder", rclcpp::QoS(500), [this](const msg::EncoderSample& e) {
          if (extrinsics_running_) extrinsic_.encoder(e.count);
          else if (state_ == Status::CALIBRATING) gyro_.encoder(e.count, now().nanoseconds());
        });
    imu_sub_ = create_subscription<msg::ImuSample>(
        "nano/imu", rclcpp::QoS(500), [this](const msg::ImuSample& m) { on_imu(m); });
    start_srv_ = service("calibration/start", &CalibrationNode::on_start);
    reset_srv_ = service("calibration/reset", &CalibrationNode::on_reset);
    extrinsics_srv_ = service("calibration/extrinsics", &CalibrationNode::on_extrinsics);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish(); });

    load();
  }

 private:
  using Handler = void (CalibrationNode::*)(Trigger::Response&);
  rclcpp::Service<Trigger>::SharedPtr service(const std::string& name, Handler h) {
    return create_service<Trigger>(name, [this, h](const std::shared_ptr<Trigger::Request>,
                                                   std::shared_ptr<Trigger::Response> res) { (this->*h)(*res); });
  }

  // ---- start-up: a file with a gyro calibration -> CALIBRATED, else CALIBRATION_REQUIRED ----
  void load() {
    std::string error;
    CalibrationData loaded;
    if (calibration_file::load(file_, loaded, error) && loaded.gyro_calibrated) {
      data_ = loaded;
      state_ = Status::CALIBRATED;
      message_ = "calibration loaded (" + data_.date + ")";
      RCLCPP_INFO(get_logger(), "calibration loaded from %s (%s)", file_.c_str(), data_.date.c_str());
    } else {
      state_ = Status::CALIBRATION_REQUIRED;
      message_ = "calibration required: hold the device still and press Calibrate gyro";
      RCLCPP_WARN(get_logger(), "%s: %s", error.empty() ? "no gyro calibration" : error.c_str(), message_.c_str());
    }
    publish();
  }

  // ---- samples go to the running procedure ----
  // The extrinsic procedure integrates the IMU over time, so it uses the Nano time; the gyro
  // calibration only needs "how long still" and uses the Pi time.
  void on_imu(const msg::ImuSample& m) {
    const Vec3 g{m.gyro_radps[0], m.gyro_radps[1], m.gyro_radps[2]};
    const Vec3 a{m.accel_mps2[0], m.accel_mps2[1], m.accel_mps2[2]};
    if (extrinsics_running_) {
      extrinsic_.imu(g, a, m.device_time_ns);  // Nano time, as the encoder
      follow_extrinsics();
    } else if (state_ == Status::CALIBRATING) {
      gyro_.imu(g, a, now().nanoseconds());
      follow_gyro();
    }
  }

  // ---- gyro procedure finished? ----
  // done: bias = mean gyro over the still period; gravity = mean accel (tilt reference); saved.
  void follow_gyro() {
    if (gyro_.state() == GyroCalibrator::State::done) {
      data_.gyro_calibrated = true;
      data_.gyro_bias_radps = gyro_.bias();
      data_.gyro_noise_radps = gyro_.noise();
      data_.gravity_imu_mps2 = gyro_.gravity();
      data_.date = iso_now();  // the extrinsics (if any) stay: the IMU mounting did not change
      const bool saved = save();
      state_ = Status::CALIBRATED;
      message_ = saved ? "gyro calibrated" : "gyro calibrated (could not write " + file_ + ")";
      RCLCPP_INFO(get_logger(), "gyro bias [%g %g %g] rad/s; saved to %s", data_.gyro_bias_radps[0],
                  data_.gyro_bias_radps[1], data_.gyro_bias_radps[2], saved ? file_.c_str() : "(failed)");
      publish();
    } else if (gyro_.state() == GyroCalibrator::State::failed) {
      state_ = data_.gyro_calibrated ? Status::CALIBRATED : Status::CALIBRATION_REQUIRED;
      message_ = "calibration failed: " + gyro_.message();
      RCLCPP_WARN(get_logger(), "%s", message_.c_str());
      publish();
    } else {
      message_ = "calibrating: " + gyro_.message();
    }
  }

  // ---- extrinsic procedure: new prompt, done or failed ----
  // done: effective roller diameter = metres_per_count * counts_per_turn / pi; saved and
  // published, so every node uses the measured scale from now on (encoder_scale.hpp).
  void follow_extrinsics() {
    switch (extrinsic_.state()) {
      case ExtrinsicCalibrator::State::done: {
        log_rolls(false);
        extrinsics_running_ = false;
        const Extrinsics& r = extrinsic_.result();
        Extrinsics& x = data_.extrinsics;
        x.calibrated = r.calibrated;
        x.base_from_imu = r.base_from_imu;
        x.metres_per_count = r.metres_per_count;
        x.encoder_direction_sign = r.encoder_direction_sign;
        x.gravity_mps2 = r.gravity_mps2;
        x.residual_m = r.residual_m;
        x.imu_distance_ratio = r.imu_distance_ratio;
        x.roller_effective_diameter_m = x.metres_per_count * counts_per_turn_ / M_PI;
        data_.accel_bias_mps2 = extrinsic_.accel_bias();
        data_.date = iso_now();
        const bool saved = save();
        state_ = Status::CALIBRATED;
        message_ = extrinsic_.message() + (saved ? "" : " (could not write " + file_ + ")");
        RCLCPP_INFO(get_logger(), "%s; roller diameter %.3f mm, encoder sign %d", message_.c_str(),
                    x.roller_effective_diameter_m * 1e3, x.encoder_direction_sign);
        publish();
        break;
      }
      case ExtrinsicCalibrator::State::failed:
        log_rolls(true);
        extrinsics_running_ = false;
        state_ = Status::CALIBRATED;  // the gyro calibration stays valid
        message_ = "extrinsic calibration failed: " + extrinsic_.message();
        RCLCPP_WARN(get_logger(), "%s", message_.c_str());
        publish();
        break;
      default:
        log_rolls(false);
        if (message_ != extrinsic_.message()) {  // next step: new prompt
          message_ = extrinsic_.message();
          RCLCPP_INFO(get_logger(), "%s", message_.c_str());
          publish();
        }
    }
  }

  /// What each finished roll measured (diagnostics in the log).
  void log_rolls(bool all_as_warning) {
    for (int i = all_as_warning ? 0 : rolls_logged_; i < extrinsic_.rolls_done(); ++i) {
      if (all_as_warning) RCLCPP_WARN(get_logger(), "%s", extrinsic_.roll_report(i, nominal_mpc_).c_str());
      else RCLCPP_INFO(get_logger(), "%s", extrinsic_.roll_report(i, nominal_mpc_).c_str());
    }
    rolls_logged_ = extrinsic_.rolls_done();
  }

  bool save() {
    std::error_code ec;
    const auto dir = std::filesystem::path(file_).parent_path();
    if (!dir.empty()) std::filesystem::create_directories(dir, ec);
    return calibration_file::save(file_, data_);
  }

  // ---- calibration/status (latched: late subscribers get the last one) ----
  void publish() {
    Status s;
    s.header.stamp = now();
    s.state = state_;
    s.message = message_;
    for (int i = 0; i < 3; ++i) {
      s.gyro_bias_radps[i] = data_.gyro_bias_radps[i];
      s.accel_bias_mps2[i] = data_.accel_bias_mps2[i];
      s.accel_scale[i] = data_.accel_scale[i];
      s.gravity_imu_mps2[i] = data_.gravity_imu_mps2[i];
    }
    s.extrinsics_calibrated = data_.extrinsics.calibrated;
    s.metres_per_count = data_.extrinsics.calibrated ? data_.extrinsics.metres_per_count : 0.0;
    s.encoder_direction_sign = static_cast<int8_t>(data_.extrinsics.encoder_direction_sign);
    status_pub_->publish(s);
  }

  // ---- services ----
  void on_start(Trigger::Response& res) {
    if (state_ == Status::CALIBRATING || extrinsics_running_) {
      res.success = false;
      res.message = "calibration already running";
      return;
    }
    gyro_.start(now().nanoseconds());
    state_ = Status::CALIBRATING;
    message_ = "calibrating: hold the device still";
    RCLCPP_INFO(get_logger(), "gyro calibration started: hold the device still");
    publish();
    res.success = true;
    res.message = message_;
  }

  void on_reset(Trigger::Response& res) {
    gyro_.cancel();
    extrinsic_.cancel();
    extrinsics_running_ = false;
    std::error_code ec;
    std::filesystem::remove(file_, ec);
    data_ = CalibrationData();
    state_ = Status::CALIBRATION_REQUIRED;
    message_ = "calibration reset: calibrate before scanning";
    RCLCPP_WARN(get_logger(), "calibration reset (%s removed)", file_.c_str());
    publish();
    res.success = true;
    res.message = message_;
  }

  void on_extrinsics(Trigger::Response& res) {
    if (!data_.gyro_calibrated || state_ == Status::CALIBRATING || extrinsics_running_) {
      res.success = false;
      res.message = extrinsics_running_ ? "extrinsic calibration already running"
                                        : "calibrate the gyro first (calibration/start)";
      return;
    }
    extrinsic_.start(data_.gyro_bias_radps);
    extrinsics_running_ = true;
    rolls_logged_ = 0;
    state_ = Status::CALIBRATING;
    message_ = extrinsic_.message();
    RCLCPP_INFO(get_logger(), "extrinsic calibration started: %s", message_.c_str());
    publish();
    res.success = true;
    res.message = message_ + " (follow the prompts in calibration/status)";
  }

  std::string file_, message_;
  GyroCalibrator gyro_;
  ExtrinsicCalibrator extrinsic_;
  bool extrinsics_running_ = false;
  int rolls_logged_ = 0;
  double nominal_mpc_ = 0, counts_per_turn_ = 4096;
  CalibrationData data_;
  uint8_t state_ = Status::CALIBRATION_REQUIRED;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::EncoderSample>::SharedPtr encoder_sub_;
  rclcpp::Subscription<msg::ImuSample>::SharedPtr imu_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, reset_srv_, extrinsics_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
