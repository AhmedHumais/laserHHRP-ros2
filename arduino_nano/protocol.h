// Serial protocol between the Arduino Nano and the Raspberry Pi (USB serial, 1,000,000 baud).
//
// Every packet looks like this:
//
//   0xAA 0x55 | type (1 byte) | length (1 byte) | payload (length bytes) | checksum (2 bytes)
//
// The payload is one of the packed structs below (little endian, the same on the Nano and the
// Pi). The checksum is a Fletcher-16 over type, length and payload. The Pi side has the same
// definitions in laserHHRP/include/laser_hhrp/serial_protocol.hpp: change both together.
//
// All times are in microseconds of the Nano's clock (micros(), extended to 64 bit).
#pragma once

#include <stdint.h>

// ---- packet types --------------------------------------------------------------------------
// Nano -> Pi
const uint8_t PKT_ENCODER = 0x01;     // encoder sample (fixed rate, 240 Hz)
const uint8_t PKT_TRIGGER = 0x02;     // one trigger pulse sent to the laser
const uint8_t PKT_IMU = 0x03;         // IMU sample (fixed rate, 240 Hz)
const uint8_t PKT_STATUS = 0x04;      // health, twice per second
const uint8_t PKT_TIME_REPLY = 0x05;  // answer to a clock exchange
// Pi -> Nano
const uint8_t PKT_COMMAND = 0x10;       // start / stop triggering, zero the encoder
const uint8_t PKT_TIME_REQUEST = 0x11;  // clock exchange request

// Commands (CommandPacket.command)
const uint8_t CMD_STOP = 0;   // stop triggering (samples keep flowing)
const uint8_t CMD_START = 1;  // restart trigger numbering at 0, trigger every N counts
const uint8_t CMD_ZERO = 2;   // set the encoder count to 0 at the current position

// Encoder flags (EncoderPacket.flags)
const uint8_t ENC_READ_ERROR = 1;  // a read failed (bad checksum or no answer)
const uint8_t ENC_JUMP = 2;        // moved half a turn between two reads: count unreliable

// ---- payloads --------------------------------------------------------------------------------
struct __attribute__((packed)) EncoderPacket {
  uint64_t time_us;
  int32_t count;  // multi-turn count since the last CMD_ZERO
  uint16_t raw;   // single-turn position 0 .. counts_per_turn-1
  uint8_t flags;  // ENC_* bits since the previous sample
};

struct __attribute__((packed)) TriggerPacket {
  uint64_t time_us;  // time of the rising edge of the pulse
  uint32_t index;    // trigger number since CMD_START (0, 1, 2, ...)
  int32_t count;     // encoder count at the pulse
  int8_t direction;  // +1 forward, -1 backward
  uint8_t flags;     // bit 0: overspeed (pulse late or a pitch skipped)
};

struct __attribute__((packed)) ImuPacket {
  uint64_t time_us;
  int16_t accel[3];  // raw MPU6050 counts, +-2 g range: 16384 counts per g
  int16_t gyro[3];   // raw MPU6050 counts, +-250 deg/s range: 131 counts per deg/s
};

struct __attribute__((packed)) StatusPacket {
  uint64_t time_us;
  uint8_t encoder_ok;  // 1 if the last encoder read was valid
  uint8_t imu_ok;      // 1 if the IMU is answering
  uint8_t triggering;  // 1 between CMD_START and CMD_STOP
  uint32_t triggers;   // pulses since CMD_START
  uint32_t overspeed;  // pulses flagged overspeed since CMD_START
};

struct __attribute__((packed)) TimeReplyPacket {
  uint32_t sequence;
  int64_t host_send_ns;  // copied from the request
  uint64_t receive_us;   // Nano clock when the request arrived
  uint64_t send_us;      // Nano clock when the reply was sent
};

struct __attribute__((packed)) CommandPacket {
  uint8_t command;                 // CMD_*
  uint16_t counts_per_trigger;     // CMD_START: encoder counts per laser trigger
  uint16_t imu_period_ms;          // IMU/encoder sample period in ms (0 = keep the default)
  uint16_t min_trigger_period_us;  // CMD_START: never trigger faster (laser's max rate)
};

struct __attribute__((packed)) TimeRequestPacket {
  uint32_t sequence;
  int64_t host_send_ns;  // Pi clock when sent
};

// ---- checksum ---------------------------------------------------------------------------------
inline uint16_t fletcher16(const uint8_t* data, uint8_t length, uint16_t start = 0) {
  uint16_t a = start & 0xFF, b = start >> 8;
  for (uint8_t i = 0; i < length; ++i) {
    a = (a + data[i]) % 255;
    b = (b + a) % 255;
  }
  return (b << 8) | a;
}
