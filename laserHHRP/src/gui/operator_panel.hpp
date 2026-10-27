// Operator panel for RViz 2 (layer 7: user interface): scan and calibration buttons and the live
// scan status. It talks to the nodes only through ROS 2 (the std_srvs/Trigger services and
// scan/status), so it can also run on a laptop on the same network.
#pragma once

#include <QLabel>
#include <QPushButton>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "laser_hhrp/msg/scan_status.hpp"

namespace laser_hhrp {

class OperatorPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit OperatorPanel(QWidget* parent = nullptr);
  void onInitialize() override;

 Q_SIGNALS:
  void reply_received(QString text, bool ok);

 private Q_SLOTS:
  void show_reply(QString text, bool ok);
  void check_link();

 private:
  QPushButton* add_button(const QString& text, const std::string& service);
  void call(const std::string& service);
  void show_status(const msg::ScanStatus& s);  // on the Qt thread

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::ScanStatus>::SharedPtr status_sub_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  std::map<std::string, QPushButton*> buttons_;
  QLabel* state_ = nullptr;
  QLabel* calibration_ = nullptr;
  QLabel* details_ = nullptr;
  QLabel* message_ = nullptr;
  QLabel* reply_ = nullptr;
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
};

}  // namespace laser_hhrp
