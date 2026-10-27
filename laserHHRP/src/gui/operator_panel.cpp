#include "operator_panel.hpp"

#include <QGridLayout>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace laser_hhrp {

using msg::ScanStatus;

// ---- layout: status labels on top, touch buttons below ----
OperatorPanel::OperatorPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout(this);
  state_ = new QLabel("Waiting for the laser roller ...");
  state_->setStyleSheet("font-size: 20pt; font-weight: bold;");
  calibration_ = new QLabel("-");
  calibration_->setStyleSheet("font-size: 13pt;");
  details_ = new QLabel("-");
  details_->setWordWrap(true);
  message_ = new QLabel("");
  message_->setWordWrap(true);
  message_->setStyleSheet("font-size: 13pt;");
  layout->addWidget(state_);
  layout->addWidget(calibration_);
  layout->addWidget(details_);
  layout->addWidget(message_);

  // Large buttons: the Pi has a touchscreen.
  auto* grid = new QGridLayout();
  grid->addWidget(add_button("Start scan", "/scan/start"), 0, 0);
  grid->addWidget(add_button("Pause / resume", "/scan/pause"), 0, 1);
  grid->addWidget(add_button("Stop scan", "/scan/stop"), 0, 2);
  grid->addWidget(add_button("Zero position", "/scan/zero"), 1, 0);
  grid->addWidget(add_button("Calibrate gyro", "/calibration/start"), 1, 1);
  grid->addWidget(add_button("Reset calibration", "/calibration/reset"), 1, 2);
  grid->addWidget(add_button("Calibrate extrinsics", "/calibration/extrinsics"), 2, 0, 1, 3);
  layout->addLayout(grid);
  reply_ = new QLabel("");
  reply_->setWordWrap(true);
  layout->addWidget(reply_);
  layout->addStretch(1);

  connect(this, &OperatorPanel::reply_received, this, &OperatorPanel::show_reply);
}

QPushButton* OperatorPanel::add_button(const QString& text, const std::string& service) {
  auto* b = new QPushButton(text);
  b->setMinimumHeight(56);
  b->setStyleSheet("font-size: 13pt;");
  b->setEnabled(false);
  connect(b, &QPushButton::clicked, this, [this, service] { call(service); });
  buttons_[service] = b;
  return b;
}

// ---- ROS side: one service client per button, the scan/status subscription ----
void OperatorPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  for (const auto& b : buttons_) clients_[b.first] = node_->create_client<std_srvs::srv::Trigger>(b.first);
  // ROS callbacks run on RViz's spinner thread: hand the message to the Qt thread.
  status_sub_ = node_->create_subscription<ScanStatus>("/scan/status", 10, [this](ScanStatus::ConstSharedPtr m) {
    const ScanStatus s = *m;
    QMetaObject::invokeMethod(this, [this, s] { show_status(s); }, Qt::QueuedConnection);
  });
  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &OperatorPanel::check_link);
  timer->start(1000);
}

// ---- a button: call its std_srvs/Trigger service; the reply is shown under the buttons ----
void OperatorPanel::call(const std::string& service) {
  auto& client = clients_[service];
  if (!client || !client->service_is_ready()) {
    show_reply(QString::fromStdString(service + ": not reachable"), false);
    return;
  }
  reply_->setText(QString::fromStdString(service + " ..."));
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                             [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
                               const auto r = f.get();
                               Q_EMIT reply_received(QString::fromStdString(r->message), r->success);
                             });
}

// ---- scan/status -> labels and which buttons are enabled ----
void OperatorPanel::show_status(const ScanStatus& s) {
  last_status_ = node_->get_clock()->now();
  static const char* states[] = {"Idle", "Scanning", "Paused", "Error"};
  state_->setText(s.state <= ScanStatus::ERROR ? states[s.state] : "Unknown");

  QString calibration = s.calibration == 2 ? "Calibrated"
                        : s.calibration == 1 ? "Calibrating: follow the instructions below"
                                             : "Calibration required";
  if (s.calibration == 2 && !s.extrinsics_calibrated) calibration += " (nominal extrinsics)";
  calibration_->setText(calibration);

  char buf[200];
  std::snprintf(buf, sizeof(buf),
                "frames %u  |  speed %.1f / %.1f mm/s  |  tilt %.1f deg  |  position %.1f mm  |  faults %u",
                s.frames, s.speed_mps * 1e3, s.max_speed_mps * 1e3, s.tilt_rad * 180.0 / M_PI,
                s.position_m * 1e3, s.active_faults);
  details_->setText(buf);
  message_->setText(QString::fromStdString(s.message));
  const bool alert = s.state == ScanStatus::ERROR || s.active_faults > 0;
  message_->setStyleSheet(alert ? "font-size: 13pt; color: #c62828; font-weight: bold;" : "font-size: 13pt;");

  // Only the buttons that make sense now are enabled.
  const bool active = s.state == ScanStatus::SCANNING || s.state == ScanStatus::PAUSED;
  const bool can_calibrate = !active && s.calibration != 1;
  buttons_["/scan/start"]->setEnabled(!active && s.calibration == 2);
  buttons_["/scan/stop"]->setEnabled(active);
  buttons_["/scan/pause"]->setEnabled(active);
  buttons_["/scan/zero"]->setEnabled(true);
  buttons_["/calibration/start"]->setEnabled(can_calibrate);
  buttons_["/calibration/reset"]->setEnabled(true);  // always available
  buttons_["/calibration/extrinsics"]->setEnabled(can_calibrate);
}

void OperatorPanel::show_reply(QString text, bool ok) {
  reply_->setText(text);
  reply_->setStyleSheet(ok ? "color: #2e7d32;" : "color: #c62828;");
}

void OperatorPanel::check_link() {  // no status for 2 s: the nodes are not running
  if (!node_ || (node_->get_clock()->now() - last_status_).seconds() < 2.0) return;
  state_->setText("Waiting for the laser roller ...");
  for (auto& b : buttons_) b.second->setEnabled(false);
}

}  // namespace laser_hhrp

PLUGINLIB_EXPORT_CLASS(laser_hhrp::OperatorPanel, rviz_common::Panel)
