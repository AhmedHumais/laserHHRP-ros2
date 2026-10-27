// point_cloud_node (layer 4: point cloud generation) - TO BE IMPLEMENTED.
//
// Input   scan/frame (laser_hhrp/ScanFrame): profile points x (along the line), z (distance),
//         position_m along the scan axis, IMU at the same instant; scan/pose (laser pose, same
//         stamp) once the extrinsics are calibrated.
// Output  scan/cloud      (sensor_msgs/PointCloud2) for the inspection node
//         scan/cloud_view (sensor_msgs/PointCloud2) for RViz ("3D scan" display)
//
// Steps (architecture document): profile points to metric coordinates, encoder position as the
// scan axis (Y), tilt compensation or quality check with the IMU, filter invalid points, merge
// the profiles into a cloud.
//
// For now it does the simplest thing: every valid point becomes (x, position_m, z), and the cloud
// of the scan so far is published once per second.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "laser_hhrp/msg/scan_frame.hpp"

namespace laser_hhrp {

class PointCloudNode : public rclcpp::Node {
 public:
  PointCloudNode() : Node("point_cloud") {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan/cloud", 5);
    view_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan/cloud_view", 5);
    frame_sub_ = create_subscription<msg::ScanFrame>(
        "scan/frame", rclcpp::QoS(200), [this](const msg::ScanFrame& f) { add(f); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish(); });
  }

 private:
  // Every valid profile point -> (x along the line, y = position along the scan axis, z = distance).
  void add(const msg::ScanFrame& f) {
    if (f.index == 0) points_.clear();  // a new scan
    for (size_t i = 0; i < f.x_m.size(); ++i) {
      if (!f.valid[i]) continue;
      points_.push_back(f.x_m[i]);
      points_.push_back(static_cast<float>(f.position_m));
      points_.push_back(f.z_m[i]);
    }
    changed_ = true;
  }

  // Once per second: the whole cloud so far as PointCloud2 (x, y, z float32).
  void publish() {
    if (!changed_) return;
    changed_ = false;
    sensor_msgs::msg::PointCloud2 c;
    c.header.stamp = now();
    c.header.frame_id = "laser";
    c.height = 1;
    c.width = static_cast<uint32_t>(points_.size() / 3);
    c.point_step = 12;
    c.row_step = c.width * 12;
    c.is_dense = true;
    const char* names[3] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      c.fields.push_back(f);
    }
    const auto* b = reinterpret_cast<const uint8_t*>(points_.data());
    c.data.assign(b, b + points_.size() * sizeof(float));
    cloud_pub_->publish(c);
    view_pub_->publish(c);
  }

  std::vector<float> points_;  // x, y, z, x, y, z, ...
  bool changed_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, view_pub_;
  rclcpp::Subscription<msg::ScanFrame>::SharedPtr frame_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::PointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
