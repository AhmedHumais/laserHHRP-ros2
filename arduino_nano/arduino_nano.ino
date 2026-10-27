// Laser roller (laserHHRP) - Arduino Nano firmware
//
// What it does:
//   1. Reads the AMT22 encoder (SPI) as fast as possible and keeps a multi-turn count.
//   2. Every N encoder counts it sends a short pulse to the laser's trigger input (In1).
//      (In software-sync mode the wire is not connected; the pulses and their records are
//      still sent, the Pi just uses the encoder samples instead.)
//   3. Reads the MPU6050 IMU (I2C).
//   4. Sends encoder samples, trigger records, IMU samples and status to the Raspberry Pi over
//      USB serial (binary packets, see protocol.h), all stamped with the Nano's own clock.
//   5. Answers clock exchanges so the Pi can convert Nano time to Pi time.
//
// Wiring (Nano pin -> device):
//   D2  -> AMT22 /CS      D11 -> AMT22 MOSI    D12 -> AMT22 MISO    D13 -> AMT22 SCLK
//   A4  -> MPU6050 SDA    A5  -> MPU6050 SCL   (MPU6050 AD0 -> GND: address 0x68)
//   D3  -> laser In1 (pin 6, yellow),  GND -> laser GND-In1 (pin 4, green)
//   5V/GND -> encoder and IMU supply
//
// Build: Arduino IDE (board "Arduino Nano", processor ATmega328P) or `pio run` in this folder.
// Never use Serial.print(): the serial port carries binary packets only.

#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "encoder_amt22.h"
#include "imu_mpu6050.h"
#include "laser_trigger.h"
#include "protocol.h"

// ---------------- settings ----------------
const uint8_t ENCODER_CS_PIN = 2;
const uint8_t TRIGGER_PIN = 3;
const uint8_t TRIGGER_PULSE_US = 20;       // pulse width; the laser starts on the rising edge
const unsigned long SERIAL_BAUD = 1000000;  // must match nano.baud on the Pi
const uint8_t ENCODER_BITS = 12;           // AMT222C-V: 12 bit (AMT223x: 14 bit)
const uint32_t SAMPLE_PERIOD_US = 4167;    // IMU + encoder samples: 240 Hz (2x the laser's 120 Hz)
// MPU6050: internal rate 1 kHz (divider 0), low-pass 94 Hz (DLPF 2, ~3 ms delay).
const uint8_t IMU_DLPF = 2;
const uint8_t IMU_SAMPLE_DIVIDER = 0;

// ---------------- devices and state ----------------
Amt22Encoder encoder(ENCODER_CS_PIN, ENCODER_BITS);
EncoderCounter counter(1UL << ENCODER_BITS);
Mpu6050 imu(IMU_DLPF, IMU_SAMPLE_DIVIDER);
LaserTrigger trigger;

bool encoder_ok = false;
uint32_t imu_period_us = SAMPLE_PERIOD_US;  // the Pi may change it (CommandPacket.imu_period_ms)
uint32_t next_encoder_us = 0, next_imu_us = 0;
uint32_t last_status_ms = 0, last_imu_retry_ms = 0;
uint32_t overspeed_count = 0;
int16_t last_accel[3], last_gyro[3];  // previous IMU sample (to skip duplicates)

// ---------------- 64-bit clock ----------------
// micros() is 32 bit and wraps every 71 minutes; this keeps counting. Called every loop.
uint64_t clock_us = 0;
uint32_t last_micros = 0;

uint64_t now_us() {
  const uint32_t m = micros();
  clock_us += uint32_t(m - last_micros);  // unsigned subtraction handles the wrap
  last_micros = m;
  return clock_us;
}

// ---------------- sending packets ----------------
void send_packet(uint8_t type, const void* payload, uint8_t length) {
  const uint8_t header[4] = {0xAA, 0x55, type, length};
  uint16_t sum = fletcher16(header + 2, 2);
  sum = fletcher16((const uint8_t*)payload, length, sum);
  Serial.write(header, 4);
  Serial.write((const uint8_t*)payload, length);
  Serial.write(uint8_t(sum & 0xFF));
  Serial.write(uint8_t(sum >> 8));
}

// ---------------- receiving packets (from the Pi) ----------------
void handle_command(const CommandPacket& c) {
  if (c.command == CMD_START) {
    trigger.start(counter.count(), c.counts_per_trigger, c.min_trigger_period_us);
    overspeed_count = 0;
  } else if (c.command == CMD_STOP) {
    trigger.stop();
  } else if (c.command == CMD_ZERO) {
    counter.zero();
  }
  if (c.imu_period_ms != 0) imu_period_us = c.imu_period_ms * 1000UL;
}

void handle_time_request(const TimeRequestPacket& r) {
  TimeReplyPacket reply;
  reply.sequence = r.sequence;
  reply.host_send_ns = r.host_send_ns;
  reply.receive_us = now_us();
  reply.send_us = now_us();
  send_packet(PKT_TIME_REPLY, &reply, sizeof(reply));
}

// Reads the bytes waiting on the serial port and handles complete packets.
void receive_packets() {
  static uint8_t buffer[32];
  static uint8_t pos = 0;  // bytes of the current packet received so far
  while (Serial.available() > 0) {
    const uint8_t byte = Serial.read();
    // Wait for the start bytes 0xAA 0x55.
    if (pos == 0 && byte != 0xAA) continue;
    if (pos == 1 && byte != 0x55) {
      pos = 0;
      continue;
    }
    buffer[pos++] = byte;
    if (pos >= 4 && buffer[3] > sizeof(buffer) - 6) {  // length too large: not a packet
      pos = 0;
      continue;
    }
    if (pos < 4 || pos < 4 + buffer[3] + 2) continue;  // not complete yet

    const uint8_t type = buffer[2], length = buffer[3];
    const uint16_t sum = fletcher16(buffer + 2, 2 + length);
    const uint16_t received = buffer[4 + length] | (uint16_t(buffer[5 + length]) << 8);
    pos = 0;
    if (sum != received) continue;  // corrupted: ignore
    if (type == PKT_COMMAND && length == sizeof(CommandPacket)) {
      CommandPacket c;
      memcpy(&c, buffer + 4, sizeof(c));
      handle_command(c);
    } else if (type == PKT_TIME_REQUEST && length == sizeof(TimeRequestPacket)) {
      TimeRequestPacket r;
      memcpy(&r, buffer + 4, sizeof(r));
      handle_time_request(r);
    }
  }
}

// ---------------- fixed-rate timing ----------------
// True when `next_us` is reached, then moves it one period on. After a long stall it restarts
// from now instead of sending a burst of samples.
bool due(uint32_t& next_us, uint32_t period_us) {
  const uint32_t now = micros();
  if (int32_t(now - next_us) < 0) return false;
  next_us += period_us;
  if (int32_t(now - next_us) >= 0) next_us = now + period_us;
  return true;
}

// ---------------- encoder + laser trigger ----------------
void fire_trigger(int8_t direction, bool overspeed) {
  digitalWrite(TRIGGER_PIN, HIGH);
  const uint64_t edge_us = now_us();
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(TRIGGER_PIN, LOW);

  TriggerPacket p;
  p.time_us = edge_us;
  p.index = trigger.index() - 1;
  p.count = counter.count();
  p.direction = direction;
  p.flags = overspeed ? 1 : 0;
  if (overspeed) ++overspeed_count;
  send_packet(PKT_TRIGGER, &p, sizeof(p));
}

void poll_encoder() {
  const uint64_t t = now_us();
  uint16_t position;
  encoder_ok = encoder.read(position);
  if (encoder_ok) {
    counter.update(position);
    int8_t direction;
    bool overspeed;
    if (trigger.update(counter.count(), now_us(), direction, overspeed))
      fire_trigger(direction, overspeed);
  } else {
    counter.flag_read_error();
  }

  // A sample to the Pi at the fixed rate (also when standing still).
  if (due(next_encoder_us, SAMPLE_PERIOD_US)) {
    EncoderPacket p;
    p.time_us = t;
    p.count = counter.count();
    p.raw = counter.raw();
    p.flags = counter.take_flags();
    send_packet(PKT_ENCODER, &p, sizeof(p));
  }
}

// ---------------- IMU ----------------
void poll_imu() {
  if (!due(next_imu_us, imu_period_us)) return;
  ImuPacket p;
  p.time_us = now_us();
  if (!imu.read(p.accel, p.gyro)) return;
  // The Nano and IMU clocks drift: sometimes the same IMU sample is read twice (all six values
  // identical - with sensor noise a new sample practically never is). Send it only once.
  bool same = true;
  for (uint8_t i = 0; i < 3; ++i)
    same = same && p.accel[i] == last_accel[i] && p.gyro[i] == last_gyro[i];
  if (same) return;
  memcpy(last_accel, p.accel, sizeof(last_accel));
  memcpy(last_gyro, p.gyro, sizeof(last_gyro));
  send_packet(PKT_IMU, &p, sizeof(p));
}

// ---------------- status + reconnect ----------------
void report_status() {
  const uint32_t ms = millis();
  if (ms - last_status_ms < 500) return;  // twice per second
  last_status_ms = ms;
  StatusPacket s;
  s.time_us = now_us();
  s.encoder_ok = encoder_ok;
  s.imu_ok = imu.ok();
  s.triggering = trigger.active();
  s.triggers = trigger.index();
  s.overspeed = overspeed_count;
  send_packet(PKT_STATUS, &s, sizeof(s));
}

void retry_imu() {  // an unplugged / hung IMU is set up again once per second
  const uint32_t ms = millis();
  if (imu.ok() || ms - last_imu_retry_ms < 1000) return;
  last_imu_retry_ms = ms;
  imu.begin();
}

// ---------------- Arduino entry points ----------------
void setup() {
  wdt_disable();  // a watchdog reset leaves the watchdog running during boot
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  SPI.begin();
  encoder.begin();
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a stuck I2C bus must not stall the trigger loop
#endif
  Serial.begin(SERIAL_BAUD);

  delay(100);  // MPU6050 power-up time
  imu.begin();
  last_micros = micros();
  next_encoder_us = next_imu_us = micros();

  // Watchdog: if anything hangs for 0.5 s the Nano restarts instead of going silent.
  wdt_enable(WDTO_500MS);
}

void loop() {
  wdt_reset();
  now_us();  // keep the 64-bit clock running
  receive_packets();
  poll_encoder();
  poll_imu();
  report_status();
  retry_imu();
}
