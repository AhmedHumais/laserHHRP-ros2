// nano_driver_node (layer 1: hardware interface): the Arduino Nano over USB serial.
//
// Publishes (every message carries the Nano time and the Pi time)
//   nano/encoder   laser_hhrp/EncoderSample   240 Hz
//   nano/trigger   laser_hhrp/TriggerRecord   one per laser trigger pulse
//   nano/imu       laser_hhrp/ImuSample       240 Hz (raw, not calibrated)
//   imu/data       sensor_msgs/Imu            the same samples, for RViz / other tools
//   nano/status    laser_hhrp/NanoStatus      2 Hz
//   nano/clock     laser_hhrp/ClockMap        Nano clock -> Pi clock, after each clock exchange
// Subscribes
//   nano/command   laser_hhrp/NanoCommand     start / stop triggering, zero (scan controller)
// The port is reopened every 2 s if the Nano is unplugged.
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "laser_hhrp/clock_sync.hpp"
#include "laser_hhrp/msg/clock_map.hpp"
#include "laser_hhrp/msg/encoder_sample.hpp"
#include "laser_hhrp/msg/imu_sample.hpp"
#include "laser_hhrp/msg/nano_command.hpp"
#include "laser_hhrp/msg/nano_status.hpp"
#include "laser_hhrp/msg/trigger_record.hpp"
#include "laser_hhrp/serial_protocol.hpp"

namespace laser_hhrp {

namespace p = protocol;

/// Opens a serial port in raw mode at `baud` (1000000 for the Nano). -1 on failure.
int open_serial(const std::string& port, int baud) {
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios tty{};
  tcgetattr(fd, &tty);
  cfmakeraw(&tty);
  const speed_t speed = baud == 1000000 ? B1000000 : baud == 500000 ? B500000 : B115200;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tcsetattr(fd, TCSANOW, &tty);
  // USB-serial adapters (FTDI) hold bytes back up to 16 ms by default: samples come in bursts and
  // the clock mapping jumps. Low latency = 1 ms (no root needed).
  serial_struct ss{};
  if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ss);
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

class NanoDriverNode : public rclcpp::Node {
 public:
  // ---- setup: parameters, topics, the 1 kHz poll timer ----
  NanoDriverNode() : Node("nano_driver") {
    port_name_ = declare_parameter("nano.port", "/dev/ttyUSB0");
    baud_ = declare_parameter("nano.baud", 1000000);

    const auto data_qos = rclcpp::QoS(500);
    encoder_pub_ = create_publisher<msg::EncoderSample>("nano/encoder", data_qos);
    trigger_pub_ = create_publisher<msg::TriggerRecord>("nano/trigger", data_qos);
    imu_pub_ = create_publisher<msg::ImuSample>("nano/imu", data_qos);
    ros_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<msg::NanoStatus>("nano/status", 10);
    clock_pub_ = create_publisher<msg::ClockMap>("nano/clock", rclcpp::QoS(10).transient_local());
    command_sub_ = create_subscription<msg::NanoCommand>(
        "nano/command", 10, [this](const msg::NanoCommand& c) { send_command(c); });

    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { poll(); });
  }

  ~NanoDriverNode() override {
    msg::NanoCommand stop;
    stop.command = msg::NanoCommand::STOP;
    send_command(stop);
    if (fd_ >= 0) ::close(fd_);
  }

 private:
  static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
  }

  // ---- main loop (every 1 ms) ----
  // 1. (re)open the port if needed, 2. read all waiting bytes and handle complete packets,
  // 3. send a clock request when due, 4. warn if the Nano has been silent for 0.5 s.
  void poll() {
    const int64_t now = now_ns();
    // (Re)open the port.
    if (fd_ < 0) {
      if (now - last_open_ns_ < 2000000000LL) return;
      last_open_ns_ = now;
      fd_ = open_serial(port_name_, static_cast<int>(baud_));
      if (fd_ < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s: serial port did not open",
                             port_name_.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "%s opened, waiting for the Nano", port_name_.c_str());
      clock_.reset();
      last_heard_ns_ = now;
    }
    // Read everything waiting (non-blocking). PacketReader finds 0xAA 0x55, checks the length and
    // the checksum; push() returns true when one packet is complete.
    uint8_t buf[512];
    ssize_t n;
    while ((n = ::read(fd_, buf, sizeof(buf))) > 0)
      for (ssize_t i = 0; i < n; ++i)
        if (reader_.push(buf[i])) handle_packet();
    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {  // unplugged
      RCLCPP_ERROR(get_logger(), "%s: Nano disconnected", port_name_.c_str());
      ::close(fd_);
      fd_ = -1;
      return;
    }
    sync_clock(now);
    if (now - last_heard_ns_ > 500000000LL)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Nano silent");
  }

  void write_packet(const std::vector<uint8_t>& bytes) {
    if (fd_ >= 0 && ::write(fd_, bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()))
      RCLCPP_WARN(get_logger(), "write to the Nano failed");
  }

  // ---- Pi -> Nano ----
  // A NanoCommand from the scan controller becomes one COMMAND packet.
  void send_command(const msg::NanoCommand& c) {
    p::CommandPacket packet{c.command, c.counts_per_trigger, c.imu_period_ms, c.min_trigger_period_us};
    write_packet(p::encode(p::PKT_COMMAND, packet));
  }

  /// Clock exchanges: 10 Hz until the mapping has a few points, then 2 Hz to follow the drift.
  void sync_clock(int64_t now) {
    const int64_t period = clock_.samples() < 8 ? 100000000 : 500000000;
    if (now - last_sync_ns_ < period) return;
    last_sync_ns_ = now;
    p::TimeRequestPacket r{++sync_sequence_, now_ns()};
    clock_.sent(r.sequence, r.host_send_ns);
    write_packet(p::encode(p::PKT_TIME_REQUEST, r));
  }

  /// Pi time of a Nano time (arrival time until the first clock exchange).
  rclcpp::Time pi_time(int64_t device_ns) const {
    return rclcpp::Time(clock_.valid() ? clock_.to_host(device_ns) : now_ns());
  }

  // ---- Nano -> Pi: one complete packet ----
  // Every packet carries the Nano time in us. We keep it (device_time_ns, used for pairing and
  // IMU interpolation) and add the Pi time (header.stamp) through the clock mapping.
  void handle_packet() {
    last_heard_ns_ = now_ns();
    switch (reader_.type()) {
      case p::PKT_ENCODER: {
        p::EncoderPacket e;
        if (!reader_.payload(e)) return;
        msg::EncoderSample m;
        m.device_time_ns = static_cast<int64_t>(e.time_us) * 1000;
        m.header.stamp = pi_time(m.device_time_ns);
        m.count = e.count;
        m.raw = e.raw;
        m.flags = e.flags;
        encoder_pub_->publish(m);
        break;
      }
      case p::PKT_TRIGGER: {
        p::TriggerPacket t;
        if (!reader_.payload(t)) return;
        msg::TriggerRecord m;
        m.device_time_ns = static_cast<int64_t>(t.time_us) * 1000;
        m.header.stamp = pi_time(m.device_time_ns);
        m.index = t.index;
        m.count = t.count;
        m.direction = t.direction;
        m.flags = t.flags;
        trigger_pub_->publish(m);
        break;
      }
      case p::PKT_IMU: {
        p::ImuPacket i;
        if (!reader_.payload(i)) return;
        msg::ImuSample m;
        m.device_time_ns = static_cast<int64_t>(i.time_us) * 1000;
        m.header.stamp = pi_time(m.device_time_ns);
        m.header.frame_id = "imu";
        // Raw MPU6050 counts -> SI: accel x 9.80665/16384 (m/s^2), gyro x (pi/180)/131 (rad/s).
        for (int k = 0; k < 3; ++k) {
          m.accel_mps2[k] = static_cast<float>(i.accel[k] * p::ACCEL_MPS2_PER_COUNT);
          m.gyro_radps[k] = static_cast<float>(i.gyro[k] * p::GYRO_RADPS_PER_COUNT);
        }
        imu_pub_->publish(m);
        sensor_msgs::msg::Imu r;
        r.header = m.header;
        r.orientation_covariance[0] = -1;  // no orientation
        r.linear_acceleration.x = m.accel_mps2[0];
        r.linear_acceleration.y = m.accel_mps2[1];
        r.linear_acceleration.z = m.accel_mps2[2];
        r.angular_velocity.x = m.gyro_radps[0];
        r.angular_velocity.y = m.gyro_radps[1];
        r.angular_velocity.z = m.gyro_radps[2];
        ros_imu_pub_->publish(r);
        break;
      }
      case p::PKT_STATUS: {
        p::StatusPacket s;
        if (!reader_.payload(s)) return;
        msg::NanoStatus m;
        m.header.stamp = pi_time(static_cast<int64_t>(s.time_us) * 1000);
        m.encoder_ok = s.encoder_ok;
        m.imu_ok = s.imu_ok;
        m.triggering = s.triggering;
        m.triggers = s.triggers;
        m.overspeed = s.overspeed;
        status_pub_->publish(m);
        break;
      }
      // Clock exchange answer: add the point to the Nano -> Pi fit (clock_sync.hpp) and publish
      // the new mapping for the other nodes: pi = host_ns + (nano - device_ns) * (1 + drift).
      case p::PKT_TIME_REPLY: {
        p::TimeReplyPacket r;
        if (!reader_.payload(r)) return;
        const int64_t receive_ns = static_cast<int64_t>(r.receive_us) * 1000;
        const int64_t send_ns = static_cast<int64_t>(r.send_us) * 1000;
        if (!clock_.reply(r.sequence, r.host_send_ns, receive_ns, send_ns, now_ns()) ||
            !clock_.valid())
          return;
        msg::ClockMap c;
        c.header.stamp = now();
        c.device_ns = send_ns;
        c.host_ns = clock_.to_host(send_ns);
        c.drift = clock_.drift();
        clock_pub_->publish(c);
        break;
      }
      default:
        break;
    }
  }

  std::string port_name_;
  int64_t baud_ = 1000000;
  int fd_ = -1;
  p::PacketReader reader_;
  ClockSync clock_;
  int64_t last_open_ns_ = -10000000000LL, last_heard_ns_ = 0, last_sync_ns_ = 0;
  uint32_t sync_sequence_ = 0;
  rclcpp::Publisher<msg::EncoderSample>::SharedPtr encoder_pub_;
  rclcpp::Publisher<msg::TriggerRecord>::SharedPtr trigger_pub_;
  rclcpp::Publisher<msg::ImuSample>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_;
  rclcpp::Publisher<msg::NanoStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::ClockMap>::SharedPtr clock_pub_;
  rclcpp::Subscription<msg::NanoCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace laser_hhrp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_hhrp::NanoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
