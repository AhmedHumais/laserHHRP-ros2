// Serial protocol Pi <-> Arduino Nano. Same packets as arduino_nano/protocol.h: change both
// together.
//
//   0xAA 0x55 | type | length | payload | Fletcher-16 checksum (2 bytes, over type+length+payload)
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace laser_hhrp {
namespace protocol {

// Nano -> Pi
constexpr uint8_t PKT_ENCODER = 0x01;
constexpr uint8_t PKT_TRIGGER = 0x02;
constexpr uint8_t PKT_IMU = 0x03;
constexpr uint8_t PKT_STATUS = 0x04;
constexpr uint8_t PKT_TIME_REPLY = 0x05;
// Pi -> Nano
constexpr uint8_t PKT_COMMAND = 0x10;
constexpr uint8_t PKT_TIME_REQUEST = 0x11;

// MPU6050 raw counts -> SI units (ranges set by the firmware: +-2 g, +-250 deg/s).
constexpr double ACCEL_MPS2_PER_COUNT = 9.80665 / 16384.0;
constexpr double GYRO_RADPS_PER_COUNT = 0.017453292519943295 / 131.0;

#pragma pack(push, 1)
struct EncoderPacket {
  uint64_t time_us;
  int32_t count;
  uint16_t raw;
  uint8_t flags;
};
struct TriggerPacket {
  uint64_t time_us;
  uint32_t index;
  int32_t count;
  int8_t direction;
  uint8_t flags;
};
struct ImuPacket {
  uint64_t time_us;
  int16_t accel[3];
  int16_t gyro[3];
};
struct StatusPacket {
  uint64_t time_us;
  uint8_t encoder_ok;
  uint8_t imu_ok;
  uint8_t triggering;
  uint32_t triggers;
  uint32_t overspeed;
};
struct TimeReplyPacket {
  uint32_t sequence;
  int64_t host_send_ns;
  uint64_t receive_us;
  uint64_t send_us;
};
struct CommandPacket {
  uint8_t command;
  uint16_t counts_per_trigger;
  uint16_t imu_period_ms;
  uint16_t min_trigger_period_us;
};
struct TimeRequestPacket {
  uint32_t sequence;
  int64_t host_send_ns;
};
#pragma pack(pop)

inline uint16_t fletcher16(const uint8_t* data, size_t length, uint16_t start = 0) {
  uint16_t a = start & 0xFF, b = start >> 8;
  for (size_t i = 0; i < length; ++i) {
    a = (a + data[i]) % 255;
    b = (b + a) % 255;
  }
  return static_cast<uint16_t>((b << 8) | a);
}

/// The bytes of one packet.
template <typename Payload>
std::vector<uint8_t> encode(uint8_t type, const Payload& payload) {
  std::vector<uint8_t> out = {0xAA, 0x55, type, static_cast<uint8_t>(sizeof(Payload))};
  const auto* p = reinterpret_cast<const uint8_t*>(&payload);
  out.insert(out.end(), p, p + sizeof(Payload));
  const uint16_t sum = fletcher16(out.data() + 2, out.size() - 2);
  out.push_back(sum & 0xFF);
  out.push_back(sum >> 8);
  return out;
}

/// Collects received bytes into packets. push() returns true when a valid packet is complete;
/// then type() and payload<T>() give it.
class PacketReader {
 public:
  bool push(uint8_t byte) {
    if (pos_ == 0 && byte != 0xAA) return false;  // wait for the start bytes
    if (pos_ == 1 && byte != 0x55) {
      pos_ = byte == 0xAA ? 1 : 0;
      return false;
    }
    buf_[pos_++] = byte;
    if (pos_ < 4) return false;
    const size_t total = 4 + buf_[3] + 2;
    if (total > sizeof(buf_)) {
      pos_ = 0;
      return false;
    }
    if (pos_ < total) return false;
    pos_ = 0;
    const uint16_t sum = fletcher16(buf_ + 2, 2 + buf_[3]);
    const uint16_t received = buf_[total - 2] | (buf_[total - 1] << 8);
    if (sum != received) {
      ++errors_;
      return false;
    }
    return true;
  }
  uint8_t type() const { return buf_[2]; }
  uint8_t length() const { return buf_[3]; }
  template <typename T>
  bool payload(T& out) const {
    if (buf_[3] != sizeof(T)) return false;
    std::memcpy(&out, buf_ + 4, sizeof(T));
    return true;
  }
  uint32_t errors() const { return errors_; }

 private:
  uint8_t buf_[64] = {};
  size_t pos_ = 0;
  uint32_t errors_ = 0;
};

}  // namespace protocol
}  // namespace laser_hhrp
