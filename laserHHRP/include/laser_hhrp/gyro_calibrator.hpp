// Gyro bias calibration. Used by calibration_node ("Calibrate gyro").
//
// A gyro at rest should read 0; what it reads instead is its bias. So:
//   1. collect IMU samples while the device rests still, for duration_s (2 s)
//   2. bias    = mean gyro reading            (per axis, rad/s)
//      noise   = standard deviation           (information only)
//      gravity = mean accelerometer reading   (the rest pose: tilt reference)
// Collecting restarts whenever the device is not still: the encoder moved, |accel| is not ~g,
// or the gyro spread / mean is too large. It gives up after timeout_s (20 s).
// Times are Pi time (only used for durations).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "laser_hhrp/calibration_data.hpp"

namespace laser_hhrp {

struct GyroCalibrationSettings {
  double duration_s = 2.0;              // stationary time to average
  double max_gyro_std_radps = 0.02;     // larger spread = moving / vibrating
  double max_gyro_abs_radps = 0.2;      // larger mean = rotating (or a broken sensor)
  double gravity_tolerance_mps2 = 0.8;  // |accel| must be ~g
  int32_t max_encoder_counts = 2;       // roller must not roll
  double timeout_s = 20.0;              // give up if never still long enough
};

class GyroCalibrator {
 public:
  enum class State { idle, collecting, done, failed };

  explicit GyroCalibrator(const GyroCalibrationSettings& s = GyroCalibrationSettings()) : s_(s) {}

  /// Starts collecting.
  void start(int64_t now_ns) {
    state_ = State::collecting;
    started_ns_ = now_ns;
    restart(now_ns);
    message_ = "hold the device still";
  }
  void cancel() { state_ = State::idle; }
  State state() const { return state_; }
  const std::string& message() const { return message_; }
  /// Seconds of stillness collected so far (progress).
  double collected_s() const { return n_ > 0 ? (last_ns_ - window_start_ns_) * 1e-9 : 0.0; }

  /// Feeds an encoder count (Nano time irrelevant: any change means motion).
  void encoder(int32_t count, int64_t now_ns) {
    if (state_ != State::collecting) return;
    if (!has_count_) {
      has_count_ = true;
      ref_count_ = count;
    } else if (std::abs(count - ref_count_) > s_.max_encoder_counts) {
      message_ = "roller moved: keep it still";
      ref_count_ = count;
      restart(now_ns);
    }
  }

  /// Feeds a raw IMU sample (gyro rad/s, accel m/s^2) received at `now_ns`.
  void imu(const Vec3& gyro, const Vec3& accel, int64_t now_ns) {
    if (state_ != State::collecting) return;
    // Give up?
    if (now_ns - started_ns_ > static_cast<int64_t>(s_.timeout_s * 1e9)) {
      state_ = State::failed;
      message_ = "device never still long enough";
      return;
    }
    // Accelerating (|accel| far from g): not at rest, start the window again.
    const double g = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (std::fabs(g - 9.80665) > s_.gravity_tolerance_mps2) {
      message_ = "device accelerating: keep it still";
      restart(now_ns);
      return;
    }
    // Add the sample to the running sums (for mean and spread).
    if (n_ == 0) window_start_ns_ = now_ns;
    ++n_;
    last_ns_ = now_ns;
    for (int i = 0; i < 3; ++i) {
      sum_[i] += gyro[i];
      sum2_[i] += gyro[i] * gyro[i];
      accel_sum_[i] += accel[i];
    }
    if (collected_s() < s_.duration_s || n_ < 20) return;  // not long enough yet
    // Window complete: mean = sum / n, std = sqrt(sum2 / n - mean^2). Too noisy or rotating ->
    // the device was moving after all, start again.
    Vec3 mean, std;
    for (int i = 0; i < 3; ++i) {
      mean[i] = sum_[i] / n_;
      std[i] = std::sqrt(std::max(0.0, sum2_[i] / n_ - mean[i] * mean[i]));
      if (std[i] > s_.max_gyro_std_radps || std::fabs(mean[i]) > s_.max_gyro_abs_radps) {
        message_ = "device moving or vibrating: keep it still";
        restart(now_ns);
        return;
      }
    }
    // Done.
    bias_ = mean;
    noise_ = std;
    for (int i = 0; i < 3; ++i) gravity_[i] = accel_sum_[i] / n_;
    state_ = State::done;
    message_ = "gyro calibrated";
  }

  const Vec3& bias() const { return bias_; }
  const Vec3& noise() const { return noise_; }
  const Vec3& gravity() const { return gravity_; }

 private:
  void restart(int64_t now_ns) {
    n_ = 0;
    sum_ = sum2_ = accel_sum_ = Vec3{0, 0, 0};
    window_start_ns_ = last_ns_ = now_ns;
  }

  GyroCalibrationSettings s_;
  State state_ = State::idle;
  std::string message_;
  int64_t started_ns_ = 0, window_start_ns_ = 0, last_ns_ = 0;
  long n_ = 0;                         // samples in the current window
  Vec3 sum_{}, sum2_{}, accel_sum_{};  // sums of gyro, gyro^2, accel
  Vec3 bias_{}, noise_{}, gravity_{};
  bool has_count_ = false;
  int32_t ref_count_ = 0;
};

}  // namespace laser_hhrp
