// Extrinsic calibration: how the IMU is mounted in the device (IMU <-> base rotation) and how far
// the roller travels per encoder count. Used by calibration_node ("Calibrate extrinsics"), after
// the gyro calibration, on a flat level surface with a ruler. Times are Nano clock.
//
// Procedure (the operator follows the prompts in the status message):
//   step 0  hold still 2 s     gravity -> base z axis (up); |a| - g = accelerometer bias along
//                              gravity (the only bias one resting pose shows)
//   step 1  roll forward 10 cm, hold still
//   step 2  roll backward 20 cm, hold still
//   step 3  roll forward 30 cm, hold still
// A roll starts when the encoder moves start_counts and ends after still_s (1 s) at rest.
//
// During a roll:
//   q  = q * rotation(gyro * dt)                   orientation relative to step 0 (gyro - bias)
//   a  = q * accel - gravity at the rest before    acceleration in the reference frame
//   v += a * dt,  p += v * dt + a dt^2 / 2         IMU velocity and displacement
// The velocity must be 0 at both rests, so its linear drift is removed:
//   v_corrected(t) = v(t) - v_end * t / T,         displacement = p_end - v_end * T / 2
//
// Results (solve):
//   metres per count:  k = sum(counts_i * d_i) / sum(counts_i^2)   (least squares, d = 10, -20,
//                      30 cm); sign of k = encoder direction; roller diameter = |k| * cpt / pi
//   base x axis:       the horizontal direction in which the IMU velocity follows the encoder
//                      velocity: sum over all samples of v_imu * v_encoder (one integration is
//                      much less sensitive to tilt and noise than the displacement)
//   base y = z x x
// Checks: each roll within max_residual_m (15 mm) of its distance, each IMU motion in the
// instructed direction, IMU speed / encoder speed between 0.5 and 2 (a roller spun without the
// device moving fails here).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "laser_hhrp/calibration_data.hpp"
#include "laser_hhrp/rotation.hpp"

namespace laser_hhrp {

struct ExtrinsicCalibrationSettings {
  double reference_still_s = 2.0;     // step 1: rest to measure gravity
  double still_s = 1.0;               // rest that ends a roll
  double still_gyro_radps = 0.05;     // |gyro| below this (bias removed) ...
  double still_accel_mps2 = 0.3;      // ... and | |accel| - gravity | below this: at rest
  int32_t still_counts = 2;           // encoder change allowed while at rest
  int32_t start_counts = 20;          // encoder change that starts a roll
  double max_residual_m = 0.015;      // encoder distance vs instructed, per roll
  double min_imu_ratio = 0.5;         // IMU / encoder distance accepted between these
  double max_imu_ratio = 2.0;
  double step_timeout_s = 60.0;       // per step
  double distances_m[3] = {0.10, -0.20, 0.30};  // instructed rolls (+ forward)
};

class ExtrinsicCalibrator {
 public:
  enum class State { idle, collecting, done, failed };

  explicit ExtrinsicCalibrator(const ExtrinsicCalibrationSettings& s = ExtrinsicCalibrationSettings())
      : s_(s) {}

  /// Nominal extrinsics (identity until measured).
  static Extrinsics nominal() { return Extrinsics(); }

  /// Starts the procedure; `gyro_bias` from the gyro calibration is removed from every sample.
  void start(const Vec3& gyro_bias) {
    *this = ExtrinsicCalibrator(s_);
    bias_ = gyro_bias;
    state_ = State::collecting;
    message_ = prompt();
  }
  void cancel() { state_ = State::idle; }
  State state() const { return state_; }
  const std::string& message() const { return message_; }
  /// 0: reference rest, 1..3: rolls.
  int step() const { return step_; }

  /// Latest encoder count (multi-turn, Nano time irrelevant).
  void encoder(int32_t count) {
    count_ = count;
    has_count_ = true;
  }

  /// One raw IMU sample (gyro rad/s, accel m/s^2) at Nano time `t_ns`.
  void imu(const Vec3& gyro_raw, const Vec3& accel, int64_t t_ns) {
    if (state_ != State::collecting || !has_count_) return;
    const Vec3 gyro = sub(gyro_raw, bias_);
    const double dt = last_ns_ != 0 ? (t_ns - last_ns_) * 1e-9 : 0.0;
    last_ns_ = t_ns;
    if (step_start_ns_ == 0) step_start_ns_ = t_ns;
    if ((t_ns - step_start_ns_) * 1e-9 > s_.step_timeout_s) return fail("step timed out: " + prompt());
    if (dt <= 0 || dt > 0.1) return;  // first sample or a gap: nothing to integrate

    // Orientation of the IMU in the reference frame (the IMU at the end of step 0).
    q_ = quat_normalized(quat_mul(q_, quat_from_rotvec(scale(gyro, dt))));
    const Vec3 f = mul(quat_to_mat(q_), accel);  // specific force, reference frame
    const bool imu_still = norm(gyro) < s_.still_gyro_radps &&
                           std::fabs(norm(accel) - gravity_) < s_.still_accel_mps2;
    const bool still = imu_still && std::abs(count_ - rest_count_) <= s_.still_counts;

    // Not rolling yet: measure the rest, wait for the encoder to start a roll.
    if (!rolling_) {
      if (still) {  // at rest: gravity (reference frame) from this rest, nothing integrated
        if (rest_n_ == 0) rest_start_ns_ = t_ns;
        ++rest_n_;
        rest_sum_ = add(rest_sum_, f);
        rest_raw_sum_ = add(rest_raw_sum_, accel);
        const double rested = (t_ns - rest_start_ns_) * 1e-9;
        if (step_ == 0) {
          if (rested >= s_.reference_still_s) finish_reference();
          return;
        }
        if (rested >= 0.3) {  // a real rest: gravity known, a roll may start from here
          armed_ = true;
          gravity_ref_ = scale(rest_sum_, 1.0 / rest_n_);
          v_ = p_ = Vec3{0, 0, 0};
          roll_ns_ = 0;
          roll_start_count_ = count_;
          roll_t_.clear();
          roll_v_.clear();
          roll_c_.clear();
          return;
        }
        // A short still moment (the slow start of a roll): keep integrating.
      } else {  // left the rest: integrate from here; it is a roll once the encoder has moved
        reset_rest();
        rest_count_ = count_;
      }
      if (!armed_) return;
      if (std::abs(count_ - roll_start_count_) > s_.start_counts) {
        rolling_ = true;
        end_count_ = count_;
        message_ = "rolling: stop and hold still at the end";
      }
    }

    // Rolling: integrate the acceleration (reference frame, gravity of the last rest removed).
    // The roll ends after still_s at rest.
    const Vec3 a = sub(f, gravity_ref_);
    p_ = add(p_, add(scale(v_, dt), scale(a, 0.5 * dt * dt)));
    v_ = add(v_, scale(a, dt));
    roll_ns_ += static_cast<int64_t>(dt * 1e9);
    roll_t_.push_back(roll_ns_ * 1e-9);
    roll_v_.push_back(v_);
    roll_c_.push_back(count_);
    if (imu_still && std::abs(count_ - end_count_) <= s_.still_counts) {
      if (end_n_ == 0) {
        end_start_ns_ = t_ns;
        end_v_ = v_;
        end_p_ = p_;
        end_roll_ns_ = roll_ns_;
      }
      ++end_n_;
      if ((t_ns - end_start_ns_) * 1e-9 >= s_.still_s) return finish_roll();
    } else {
      end_n_ = 0;
      end_count_ = count_;
    }
  }

  /// The result (valid in state done).
  const Extrinsics& result() const { return result_; }
  /// Accelerometer bias along gravity (IMU frame), valid in state done.
  const Vec3& accel_bias() const { return accel_bias_; }
  /// Encoder counts and IMU displacement along x of each roll (diagnostics).
  int32_t roll_counts(int i) const { return counts_[i]; }
  double roll_imu_m(int i) const { return dot(displacement_[i], x_axis_); }
  /// Rolls finished so far (0..3) and a one-line report of roll i (diagnostics, any state).
  int rolls_done() const { return rolls_done_; }
  std::string roll_report(int i, double metres_per_count_nominal) const {
    const Vec3 z = normalized(gravity_raw_);
    const Vec3 h = sub(displacement_[i], scale(z, dot(displacement_[i], z)));
    char text[200];
    std::snprintf(text, sizeof(text),
                  "roll %d (asked %+.0f cm): %d counts = %+.1f cm at the configured diameter, "
                  "IMU %.1f cm horizontal, %.1f s",
                  i + 1, s_.distances_m[i] * 100, counts_[i],
                  counts_[i] * metres_per_count_nominal * 100, norm(h) * 100, durations_s_[i]);
    return text;
  }

 private:
  std::string prompt() const {
    switch (step_) {
      case 0: return "extrinsics 1/4: device on a flat surface, hold it still";
      case 1: return "extrinsics 2/4: roll forward 10 cm, then hold still";
      case 2: return "extrinsics 3/4: roll backward 20 cm, then hold still";
      default: return "extrinsics 4/4: roll forward 30 cm, then hold still";
    }
  }
  void reset_rest() {
    rest_n_ = 0;
    rest_sum_ = rest_raw_sum_ = Vec3{0, 0, 0};
  }
  void fail(const std::string& why) {
    state_ = State::failed;
    message_ = why;
  }
  void next_step() {
    ++step_;
    step_start_ns_ = last_ns_;
    rolling_ = armed_ = false;
    end_n_ = 0;
    reset_rest();
    rest_count_ = roll_start_count_ = end_count_ = count_;
    message_ = prompt();
  }

  /// Step 0 done: gravity in the IMU frame and its magnitude.
  void finish_reference() {
    gravity_raw_ = scale(rest_raw_sum_, 1.0 / rest_n_);  // IMU frame
    gravity_ = norm(gravity_raw_);
    if (std::fabs(gravity_ - 9.80665) > 1.5)
      return fail("accelerometer reads " + std::to_string(gravity_) + " m/s^2 at rest");
    q_ = Quat{1, 0, 0, 0};  // the reference frame is the IMU now
    gravity_ref_ = gravity_raw_;
    next_step();
  }

  /// A roll ended: its displacement (drift removed), counts, velocity correlation; next step or
  /// solve after the third roll.
  void finish_roll() {
    const int i = step_ - 1;
    const double T = end_roll_ns_ * 1e-9;
    // Zero velocity at both rests: remove the linear velocity drift from the displacement.
    displacement_[i] = sub(end_p_, scale(end_v_, 0.5 * T));
    counts_[i] = count_ - roll_start_count_;
    durations_s_[i] = T;
    correlate(i, T);
    rolls_done_ = i + 1;
    if (std::abs(counts_[i]) < 2 * s_.start_counts)
      return fail("roll " + std::to_string(i + 1) + ": the encoder hardly moved");
    if (step_ < 3) return next_step();
    solve();
  }

  /// Velocity correlation of roll i (samples up to the rest at T): the IMU velocity with its
  /// linear drift removed (zero at both rests) times the encoder velocity (counts/s, central
  /// difference over +-25 ms against the count quantization).
  void correlate(int i, double T) {
    const size_t n = roll_t_.size();
    const Vec3 drift = T > 0 ? scale(end_v_, 1.0 / T) : Vec3{0, 0, 0};
    corr_[i] = Vec3{0, 0, 0};
    enc2_[i] = 0;
    size_t w = 1;
    while (w < n && roll_t_[w] - roll_t_[0] < 0.025) ++w;
    for (size_t k = w; k + w < n && roll_t_[k] <= T; ++k) {
      const double dt = roll_t_[k + w] - roll_t_[k - w];
      if (dt <= 0) continue;
      const double v_enc = (roll_c_[k + w] - roll_c_[k - w]) / dt;
      const Vec3 v_imu = sub(roll_v_[k], scale(drift, roll_t_[k]));
      const double h = roll_t_[k] - roll_t_[k - 1];
      corr_[i] = add(corr_[i], scale(v_imu, v_enc * h));
      enc2_[i] += v_enc * v_enc * h;
    }
  }

  /// All three rolls done: metres per count, encoder direction, base axes, checks.
  void solve() {
    // Encoder: metres per count (least squares through the origin, signed).
    double cd = 0, cc = 0;
    for (int i = 0; i < 3; ++i) {
      cd += counts_[i] * s_.distances_m[i];
      cc += static_cast<double>(counts_[i]) * counts_[i];
    }
    const double k = cd / cc;
    // Base z: up (the specific force at rest points up). x: v_imu = x * k * v_encoder, so the
    // correlation sum points along x * sign(k); its horizontal part gives x.
    const Vec3 z = normalized(gravity_raw_);
    Vec3 sum{0, 0, 0};
    double e2 = 0;
    for (int i = 0; i < 3; ++i) {
      sum = add(sum, corr_[i]);
      e2 += enc2_[i];
    }
    const Vec3 h = sub(sum, scale(z, dot(sum, z)));
    if (e2 <= 0 || norm(h) / e2 < 1e-9)
      return fail("the IMU saw no horizontal motion: was the device moved?");
    x_axis_ = scale(normalized(h), k > 0 ? 1.0 : -1.0);
    const Vec3 y = cross(z, x_axis_);
    double imu = 0, enc = 0;
    double residual = 0;
    for (int i = 0; i < 3; ++i) {
      const double err = std::fabs(k * counts_[i] - s_.distances_m[i]);
      if (err > residual) residual = err;
      if (err > s_.max_residual_m) {
        char text[160];
        std::snprintf(text, sizeof(text),
                      "roll %d: %.1f cm instead of %.0f cm (or another roll was off): repeat", i + 1,
                      std::fabs(k * counts_[i]) * 100, std::fabs(s_.distances_m[i]) * 100);
        return fail(text);
      }
      // The IMU velocity must follow the encoder in this roll too (positive correlation).
      if (dot(corr_[i], x_axis_) * k <= 0)
        return fail("roll " + std::to_string(i + 1) + ": the IMU moved the other way: repeat");
      imu += std::fabs(dot(corr_[i], x_axis_));
      enc += std::fabs(k) * enc2_[i];
    }
    const double ratio = imu / enc;  // IMU speed / encoder speed (velocity regression slope)
    if (ratio < s_.min_imu_ratio || ratio > s_.max_imu_ratio) {
      char text[160];
      std::snprintf(text, sizeof(text),
                    "IMU speed %.0f %% of the encoder speed: roller turned without moving, "
                    "or slipping: repeat",
                    ratio * 100);
      return fail(text);
    }
    Extrinsics& e = result_;
    e.calibrated = true;
    e.base_from_imu = mat_to_quat(Mat3{x_axis_, y, z});  // rows: base axes in IMU coordinates
    e.metres_per_count = std::fabs(k);
    e.encoder_direction_sign = k > 0 ? 1 : -1;
    e.gravity_mps2 = 9.80665;
    e.residual_m = residual;
    e.imu_distance_ratio = ratio;
    accel_bias_ = scale(z, gravity_ - 9.80665);
    state_ = State::done;
    char text[160];
    std::snprintf(text, sizeof(text),
                  "extrinsics calibrated: %.3f um/count, residual %.1f mm, IMU/encoder %.2f",
                  e.metres_per_count * 1e6, residual * 1e3, ratio);
    message_ = text;
  }

  ExtrinsicCalibrationSettings s_;
  State state_ = State::idle;
  std::string message_;
  Vec3 bias_{};
  int step_ = 0;
  int64_t last_ns_ = 0, step_start_ns_ = 0;
  bool has_count_ = false;
  int32_t count_ = 0, rest_count_ = 0, roll_start_count_ = 0, end_count_ = 0;
  Quat q_{1, 0, 0, 0};
  double gravity_ = 9.80665;
  Vec3 gravity_raw_{0, 0, 9.80665}, gravity_ref_{0, 0, 9.80665};
  // rest detection
  long rest_n_ = 0;
  int64_t rest_start_ns_ = 0;
  Vec3 rest_sum_{}, rest_raw_sum_{};
  bool armed_ = false;
  // roll integration
  bool rolling_ = false;
  Vec3 v_{}, p_{};
  int64_t roll_ns_ = 0;
  long end_n_ = 0;
  int64_t end_start_ns_ = 0, end_roll_ns_ = 0;
  Vec3 end_v_{}, end_p_{};
  // results
  Vec3 displacement_[3]{};
  int32_t counts_[3]{};
  Vec3 corr_[3]{};    // sum of v_imu * v_encoder per roll
  double enc2_[3]{};  // sum of v_encoder^2 per roll
  std::vector<double> roll_t_;
  std::vector<Vec3> roll_v_;
  std::vector<int32_t> roll_c_;
  double durations_s_[3]{};
  int rolls_done_ = 0;
  Vec3 x_axis_{1, 0, 0};
  Vec3 accel_bias_{};
  Extrinsics result_;
};

}  // namespace laser_hhrp
