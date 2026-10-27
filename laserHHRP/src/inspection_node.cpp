// inspection_node (layer 5: inspection algorithms) - TO BE IMPLEMENTED.
//
// Input   scan/cloud (sensor_msgs/PointCloud2) from point_cloud_node
// Output  inspection/result (laser_hhrp/InspectionResult), one per feature; recorder_node writes
//         them to results.json
//
// Steps (architecture document): detect the rivet region, select the reference surface, fit a
// local plane, estimate the head height, compute the height difference, compare with the
// tolerance -> flushness, countersink depth, surface deviation, position, pass / fail.
// The tolerances are parameters (config/hhrp.yaml, inspection.*).
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "laser_hhrp/msg/inspection_result.hpp"

namespace laser_hhrp {

class InspectionNode : public rclcpp::Node {
 public:
  InspectionNode() : Node("inspection") {
    flushness_tolerance_m_ = declare_parameter("inspection.flushness_tolerance_m", 0.00005);
    declare_parameter("inspection.countersink_tolerance_m", 0.00005);
    declare_parameter("inspection.fastener_type", "rivet");
    declare_parameter("inspection.fastener_diameter_m", 0.005);
    declare_parameter("inspection.surface_deviation_limit_m", 0.0001);
    result_pub_ = create_publisher<msg::InspectionResult>("inspection/result", 50);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan/cloud", 5, [this](const sensor_msgs::msg::PointCloud2& c) { inspect(c); });
  }

 private:
  void inspect(const sensor_msgs::msg::PointCloud2& cloud) {
    (void)cloud;  // the algorithm goes here; publish one InspectionResult per feature found
  }

  double flushness_tolerance_m_ = 0;
  rclcpp::Publisher<msg::InspectionResult>::SharedPtr result_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::InspectionNode>());
  rclcpp::shutdown();
  return 0;
}
