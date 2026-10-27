// The calibration of one device and its file. Used by calibration_node (writes it) and
// odometry_node (reads the extrinsics).
//
//   IMU intrinsics  gyro bias, accelerometer bias and scale, gravity at rest (tilt reference)
//   extrinsics      IMU <-> base rotation, metres per encoder count, encoder direction;
//                   laser <-> roller values are still nominal (identity)
//
// File: parameter calibration.file (default ~/laser_hhrp/calibration.yaml), one per device.
// A simple flat YAML subset (`key: value`, numbers, strings, [a, b, c] lists), read and written
// here without a YAML library.
#pragma once

#include <array>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace laser_hhrp {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

/// Extrinsic calibration (frames: laser = x along the line, z distance; imu; roller = rotation
/// axis / contact line; base = the device on the surface: x forward (rolling direction), z up
/// (against gravity), y = z x x). The IMU <-> base rotation and the encoder scale come from the
/// rolling procedure (ExtrinsicCalibrator); the laser <-> roller values are still nominal.
struct Extrinsics {
  bool calibrated = false;                 // IMU <-> base and encoder scale measured
  Quat base_from_imu = {1, 0, 0, 0};       // rotation IMU frame -> base frame
  double metres_per_count = 0;             // rolled distance per encoder count (0 = not measured)
  int encoder_direction_sign = 1;          // +1: the count increases when rolling forward
  double gravity_mps2 = 9.80665;           // |accel| at rest (after the bias along gravity)
  double residual_m = 0;                   // worst encoder distance error of the procedure
  double imu_distance_ratio = 0;           // IMU / encoder distance (check, ~1)
  Quat imu_from_laser = {1, 0, 0, 0};      // rotation laser frame -> IMU frame
  Vec3 laser_in_roller_m = {0, 0, 0};      // laser origin in the roller frame
  Quat roller_from_laser = {1, 0, 0, 0};   // rotation laser frame -> roller frame
  double roller_effective_diameter_m = 0;  // 0 = use encoder.roller_diameter_m
};

struct CalibrationData {
  std::string device = "laserHHRP";  // unit name / serial
  std::string date;                  // ISO time of the last calibration
  bool gyro_calibrated = false;
  Vec3 gyro_bias_radps = {0, 0, 0};   // subtracted from every gyro sample
  Vec3 gyro_noise_radps = {0, 0, 0};  // std of the stationary samples (information)
  Vec3 accel_bias_mps2 = {0, 0, 0};   // subtracted (extrinsic procedure: along gravity only)
  Vec3 accel_scale = {1, 1, 1};       // ... then multiplied
  Vec3 gravity_imu_mps2 = {0, 0,
                           9.80665};  // mean accel during the gyro calibration (reference pose)
  Extrinsics extrinsics;

  /// Corrected IMU sample: gyro - bias; (accel - bias) * scale.
  template <typename Imu>
  void correct(Imu& m) const {
    for (int i = 0; i < 3; ++i) {
      m.gyro_radps[i] = static_cast<float>(m.gyro_radps[i] - gyro_bias_radps[i]);
      m.accel_mps2[i] = static_cast<float>((m.accel_mps2[i] - accel_bias_mps2[i]) * accel_scale[i]);
    }
  }
};

namespace calibration_file {

inline std::string vec(const double* v, int n) {
  std::ostringstream o;
  o.precision(9);
  o << "[";
  for (int i = 0; i < n; ++i) o << (i ? ", " : "") << v[i];
  o << "]";
  return o.str();
}

inline std::string num(double v) {
  std::ostringstream o;
  o.precision(9);
  o << v;
  return o.str();
}

inline bool save(const std::string& path, const CalibrationData& c) {
  std::ofstream f(path);
  if (!f) return false;
  const Extrinsics& e = c.extrinsics;
  f << "# laserHHRP device calibration (written by calibration_node; see README "
       "\"Calibration\").\n"
    << "# Delete it, or call the calibration/reset service, to calibrate again.\n"
    << "version: 1\n"
    << "device: " << c.device << "\n"
    << "date: " << c.date << "\n"
    << "# IMU intrinsics\n"
    << "gyro_calibrated: " << (c.gyro_calibrated ? "true" : "false") << "\n"
    << "gyro_bias_radps: " << vec(c.gyro_bias_radps.data(), 3) << "\n"
    << "gyro_noise_radps: " << vec(c.gyro_noise_radps.data(), 3) << "\n"
    << "accel_bias_mps2: " << vec(c.accel_bias_mps2.data(), 3) << "\n"
    << "accel_scale: " << vec(c.accel_scale.data(), 3) << "\n"
    << "gravity_imu_mps2: " << vec(c.gravity_imu_mps2.data(), 3) << "\n"
    << "# Extrinsics (calibration/extrinsics: rolling procedure; laser values still nominal)\n"
    << "extrinsics_calibrated: " << (e.calibrated ? "true" : "false") << "\n"
    << "base_from_imu_wxyz: " << vec(e.base_from_imu.data(), 4) << "\n"
    << "metres_per_count: " << num(e.metres_per_count) << "\n"
    << "encoder_direction_sign: " << e.encoder_direction_sign << "\n"
    << "gravity_mps2: " << num(e.gravity_mps2) << "\n"
    << "residual_m: " << num(e.residual_m) << "\n"
    << "imu_distance_ratio: " << num(e.imu_distance_ratio) << "\n"
    << "imu_from_laser_wxyz: " << vec(e.imu_from_laser.data(), 4) << "\n"
    << "laser_in_roller_m: " << vec(e.laser_in_roller_m.data(), 3) << "\n"
    << "roller_from_laser_wxyz: " << vec(e.roller_from_laser.data(), 4) << "\n"
    << "roller_effective_diameter_m: " << e.roller_effective_diameter_m << "\n";
  return static_cast<bool>(f);
}

namespace detail {
inline std::string trim(const std::string& s) {
  const size_t a = s.find_first_not_of(" \t\r");
  const size_t b = s.find_last_not_of(" \t\r");
  return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}
template <size_t N>
inline bool list(const std::string& text, std::array<double, N>& out) {
  std::string t = trim(text);
  if (t.size() < 2 || t.front() != '[' || t.back() != ']') return false;
  t = t.substr(1, t.size() - 2);
  std::stringstream ss(t);
  std::string item;
  size_t i = 0;
  while (std::getline(ss, item, ',')) {
    if (i >= N) return false;
    try {
      out[i++] = std::stod(item);
    } catch (...) {
      return false;
    }
  }
  return i == N;
}
}  // namespace detail

/// Loads a calibration file; false (and `error`) if missing or invalid. Unknown keys are ignored.
inline bool load(const std::string& path, CalibrationData& c, std::string& error) {
  std::ifstream f(path);
  if (!f) {
    error = "no calibration file " + path;
    return false;
  }
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(f, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    kv[detail::trim(line.substr(0, colon))] = detail::trim(line.substr(colon + 1));
  }
  if (kv["version"] != "1") {
    error = "unsupported calibration file version in " + path;
    return false;
  }
  CalibrationData r;
  Extrinsics& e = r.extrinsics;
  r.device = kv["device"];
  r.date = kv["date"];
  r.gyro_calibrated = kv["gyro_calibrated"] == "true";
  e.calibrated = kv["extrinsics_calibrated"] == "true";
  bool ok = detail::list(kv["gyro_bias_radps"], r.gyro_bias_radps) &&
            detail::list(kv["accel_bias_mps2"], r.accel_bias_mps2) &&
            detail::list(kv["accel_scale"], r.accel_scale) &&
            detail::list(kv["imu_from_laser_wxyz"], e.imu_from_laser) &&
            detail::list(kv["laser_in_roller_m"], e.laser_in_roller_m) &&
            detail::list(kv["roller_from_laser_wxyz"], e.roller_from_laser);
  detail::list(kv["gyro_noise_radps"], r.gyro_noise_radps);  // optional
  detail::list(kv["gravity_imu_mps2"], r.gravity_imu_mps2);  // optional
  detail::list(kv["base_from_imu_wxyz"], e.base_from_imu);    // optional (older files)
  auto number = [&kv](const char* key, double& out) {
    if (!kv[key].empty()) out = std::stod(kv[key]);
  };
  try {
    number("metres_per_count", e.metres_per_count);
    number("gravity_mps2", e.gravity_mps2);
    number("residual_m", e.residual_m);
    number("imu_distance_ratio", e.imu_distance_ratio);
    double sign = e.encoder_direction_sign;
    number("encoder_direction_sign", sign);
    e.encoder_direction_sign = sign < 0 ? -1 : 1;
    e.roller_effective_diameter_m = kv["roller_effective_diameter_m"].empty()
                                        ? 0.0
                                        : std::stod(kv["roller_effective_diameter_m"]);
  } catch (...) {
    ok = false;
  }
  if (!ok) {
    error = "invalid calibration file " + path;
    return false;
  }
  c = r;
  return true;
}

}  // namespace calibration_file
}  // namespace laser_hhrp
