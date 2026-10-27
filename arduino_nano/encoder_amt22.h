// AMT22 absolute encoder (Same Sky AMT222C-V, 12 bit) read over SPI, and the multi-turn count.
//
// The AMT22 answers every SPI read with a 16-bit word: 14 data bits plus 2 check bits. A 12-bit
// encoder puts its position in bits 13..2. The encoder is single-turn, so we "unwrap" it: if the
// position jumps by more than half a turn between two reads, it went over the 0 point.
#pragma once

#include <Arduino.h>
#include <SPI.h>

class Amt22Encoder {
 public:
  Amt22Encoder(uint8_t cs_pin, uint8_t bits) : cs_pin_(cs_pin), bits_(bits) {}

  void begin() {
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);
  }

  uint32_t counts_per_turn() const { return 1UL << bits_; }

  // Reads the position. Returns false if the answer is invalid (check bits wrong / unplugged).
  bool read(uint16_t& position) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin_, LOW);
    delayMicroseconds(3);  // the AMT22 needs short pauses between the bytes
    const uint8_t high = SPI.transfer(0);
    delayMicroseconds(3);
    const uint8_t low = SPI.transfer(0);
    delayMicroseconds(3);
    digitalWrite(cs_pin_, HIGH);
    SPI.endTransaction();
    delayMicroseconds(40);  // minimum time before the next read

    const uint16_t word = (uint16_t(high) << 8) | low;
    if (!check_bits_ok(word)) return false;
    if (bits_ == 12 && (word & 3) != 0) return false;  // 12-bit: the two low bits are always 0
    position = (word & 0x3FFF) >> (14 - bits_);
    return true;
  }

 private:
  // Odd parity: bit 15 checks the odd bits, bit 14 the even bits.
  static bool check_bits_ok(uint16_t word) {
    uint16_t check = 3;
    for (uint8_t i = 0; i < 14; i += 2) check ^= (word >> i) & 3;
    return check == (word >> 14);
  }

  uint8_t cs_pin_;
  uint8_t bits_;
};

// Turns consecutive single-turn positions into a multi-turn count.
class EncoderCounter {
 public:
  explicit EncoderCounter(uint32_t counts_per_turn) : cpt_(counts_per_turn) {}

  void update(uint16_t raw) {
    if (!started_) {  // first read: this is count 0
      started_ = true;
      last_raw_ = raw;
      return;
    }
    int32_t delta = int32_t(raw) - int32_t(last_raw_);
    const int32_t half = cpt_ / 2;
    if (delta > half) delta -= cpt_;   // passed 0 going backward
    if (delta < -half) delta += cpt_;  // passed 0 going forward
    if (delta == half || delta == -half) flags_ |= 2;  // can't tell the direction: flag JUMP
    count_ += delta;
    last_raw_ = raw;
  }

  void zero() { count_ = 0; }
  void flag_read_error() { flags_ |= 1; }
  int32_t count() const { return count_; }
  uint16_t raw() const { return last_raw_; }

  // Flags collected since the last call (then cleared).
  uint8_t take_flags() {
    const uint8_t f = flags_;
    flags_ = 0;
    return f;
  }

 private:
  uint32_t cpt_;
  bool started_ = false;
  uint16_t last_raw_ = 0;
  int32_t count_ = 0;
  uint8_t flags_ = 0;
};
