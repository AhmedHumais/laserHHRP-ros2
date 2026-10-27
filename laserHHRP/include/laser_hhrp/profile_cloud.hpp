// A laser profile as sensor_msgs/PointCloud2 for RViz (x along the line, y = 0, z = distance;
// invalid points dropped). Used by laser_driver_node and simulator_node.
#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "laser_hhrp/msg/raw_profile.hpp"

namespace laser_hhrp {

inline sensor_msgs::msg::PointCloud2 to_cloud(const msg::RawProfile& p) {
  sensor_msgs::msg::PointCloud2 c;
  c.header = p.header;
  c.height = 1;
  c.point_step = 12;
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
  c.data.reserve(p.x_m.size() * 12);
  for (size_t i = 0; i < p.x_m.size(); ++i) {
    if (!p.valid[i]) continue;
    const float xyz[3] = {p.x_m[i], 0.0f, p.z_m[i]};
    const auto* b = reinterpret_cast<const uint8_t*>(xyz);
    c.data.insert(c.data.end(), b, b + 12);
  }
  c.width = static_cast<uint32_t>(c.data.size() / 12);
  c.row_step = c.width * 12;
  return c;
}

}  // namespace laser_hhrp
