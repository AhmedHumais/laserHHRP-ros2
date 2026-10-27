// recorder_node (layer 6: storage and export): one folder per scan in system.storage_path,
// named YYYYmmdd-HHMMSS:
//   frames.bin    every ScanFrame: [uint32 length][ROS 2 serialized ScanFrame] ...
//                 (read it with scripts/read_session.py)
//   results.json  inspection results
//   session.json  start/stop time, frame count, synchronization summary, all parameters
// A session opens when a scan starts and closes when it stops (scan/status).
//
// Subscribes  scan/frame, inspection/result, scan/status, scan/sync
// Publishes   scan/storage  laser_hhrp/StorageStatus (2 Hz)
#include <sys/statvfs.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "laser_hhrp/msg/inspection_result.hpp"
#include "laser_hhrp/msg/scan_frame.hpp"
#include "laser_hhrp/msg/scan_status.hpp"
#include "laser_hhrp/msg/storage_status.hpp"
#include "laser_hhrp/msg/sync_stats.hpp"

namespace laser_hhrp {

using ScanStatus = msg::ScanStatus;

std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o;
}

std::string iso_time(std::time_t t) {
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
  return buf;
}

class RecorderNode : public rclcpp::Node {
 public:
  // All parameters of the launch file are accepted, so session.json can list them.
  RecorderNode()
      : Node("recorder", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    if (!has_parameter("system.storage_path")) declare_parameter("system.storage_path", "~/laser_hhrp/scans");
    root_ = get_parameter("system.storage_path").as_string();
    if (!root_.empty() && root_[0] == '~') root_ = std::string(getenv("HOME")) + root_.substr(1);

    status_pub_ = create_publisher<msg::StorageStatus>("scan/storage", 10);
    frame_sub_ = create_subscription<msg::ScanFrame>(
        "scan/frame", rclcpp::QoS(200), [this](const msg::ScanFrame& f) { write(f); });
    result_sub_ = create_subscription<msg::InspectionResult>(
        "inspection/result", 50, [this](const msg::InspectionResult& r) { add_result(r); });
    stats_sub_ = create_subscription<msg::SyncStats>("scan/sync", 10, [this](const msg::SyncStats& s) { stats_ = s; });
    scan_sub_ = create_subscription<ScanStatus>("scan/status", 10, [this](const ScanStatus& s) {
      const bool on = s.state == ScanStatus::SCANNING || s.state == ScanStatus::PAUSED;
      if (on && !scanning_) begin();
      if (!on && scanning_) end();
      scanning_ = on;
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_status(); });
  }

  ~RecorderNode() override { end(); }

 private:
  // ---- session start: new folder named after the local time, frames.bin opened ----
  void begin() {
    end();
    start_ = std::time(nullptr);
    char name[32];
    std::strftime(name, sizeof(name), "%Y%m%d-%H%M%S", std::localtime(&start_));
    dir_ = root_ + "/" + name;
    std::error_code ec;
    std::filesystem::create_directories(dir_, ec);
    file_.open(dir_ + "/frames.bin", std::ios::binary | std::ios::trunc);
    results_.clear();
    frames_ = 0;
    write_failed_ = !file_.is_open();
    RCLCPP_INFO(get_logger(), "recording to %s", dir_.c_str());
  }

  // ---- session end: close frames.bin, write results.json and session.json ----
  void end() {
    if (!file_.is_open()) return;
    file_.close();
    std::ofstream results(dir_ + "/results.json");
    results << "[\n";
    for (size_t i = 0; i < results_.size(); ++i)
      results << "  " << results_[i] << (i + 1 < results_.size() ? ",\n" : "\n");
    results << "]\n";
    std::ofstream session(dir_ + "/session.json");
    session << "{\n  \"start\": \"" << iso_time(start_) << "\",\n  \"stop\": \"" << iso_time(std::time(nullptr))
            << "\",\n  \"frames\": " << frames_ << ",\n  \"summary\": {\"missing_profiles\": "
            << stats_.missing_profiles << ", \"missing_triggers\": " << stats_.missing_triggers
            << ", \"timing_mismatches\": " << stats_.timing_mismatches
            << ", \"length_m\": " << stats_.last_position_m << "},\n  \"parameters\": " << parameters_json()
            << "\n}\n";
    RCLCPP_INFO(get_logger(), "session closed: %lu frames in %s", static_cast<unsigned long>(frames_), dir_.c_str());
  }

  // ---- one frame: [uint32 length][ROS 2 serialized ScanFrame] (read_session.py reads it) ----
  void write(const msg::ScanFrame& f) {
    if (!file_.is_open()) return;
    rclcpp::SerializedMessage data;
    serializer_.serialize_message(&f, &data);
    const uint32_t length = static_cast<uint32_t>(data.size());
    file_.write(reinterpret_cast<const char*>(&length), sizeof(length));
    file_.write(reinterpret_cast<const char*>(data.get_rcl_serialized_message().buffer), length);
    if (!file_) write_failed_ = true;
    ++frames_;
  }

  void add_result(const msg::InspectionResult& r) {
    if (!file_.is_open()) return;
    std::ostringstream o;
    o << "{\"feature_type\": \"" << json_escape(r.feature_type) << "\", \"position_m\": [" << r.position_m[0]
      << ", " << r.position_m[1] << ", " << r.position_m[2] << "], \"flushness_m\": " << r.flushness_m
      << ", \"countersink_depth_m\": " << r.countersink_depth_m
      << ", \"surface_deviation_m\": " << r.surface_deviation_m << ", \"passed\": " << (r.passed ? "true" : "false")
      << ", \"detail\": \"" << json_escape(r.detail) << "\"}";
    results_.push_back(o.str());
  }

  std::string parameters_json() {
    std::ostringstream o;
    o << "{";
    bool first = true;
    for (const auto& name : list_parameters({}, 0).names) {
      if (name == "use_sim_time") continue;
      const rclcpp::Parameter p = get_parameter(name);
      const std::string value = p.get_type() == rclcpp::ParameterType::PARAMETER_STRING
                                    ? "\"" + json_escape(p.as_string()) + "\""
                                    : p.value_to_string();
      o << (first ? "" : ", ") << "\"" << name << "\": " << value;
      first = false;
    }
    return o.str() + "}";
  }

  // ---- scan/storage (2 Hz): free space = available blocks * block size ----
  void publish_status() {
    std::error_code ec;
    std::filesystem::create_directories(root_, ec);
    struct statvfs s {};
    msg::StorageStatus m;
    m.header.stamp = now();
    m.recording = file_.is_open();
    m.session_dir = dir_;
    m.free_mb = statvfs(root_.c_str(), &s) == 0
                    ? static_cast<float>(static_cast<double>(s.f_bavail) * s.f_frsize / (1024.0 * 1024.0))
                    : -1.0f;
    m.write_failed = write_failed_;
    status_pub_->publish(m);
  }

  std::string root_, dir_;
  std::ofstream file_;
  std::vector<std::string> results_;
  uint64_t frames_ = 0;
  bool write_failed_ = false, scanning_ = false;
  std::time_t start_ = 0;
  msg::SyncStats stats_;
  rclcpp::Serialization<msg::ScanFrame> serializer_;
  rclcpp::Publisher<msg::StorageStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::ScanFrame>::SharedPtr frame_sub_;
  rclcpp::Subscription<msg::InspectionResult>::SharedPtr result_sub_;
  rclcpp::Subscription<msg::SyncStats>::SharedPtr stats_sub_;
  rclcpp::Subscription<ScanStatus>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::RecorderNode>());
  rclcpp::shutdown();
  return 0;
}
