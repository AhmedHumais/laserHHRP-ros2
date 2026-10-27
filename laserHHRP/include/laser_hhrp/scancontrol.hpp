// Micro-Epsilon scanCONTROL 29xx (LLT2900-10/BL) through the scanCONTROL Linux SDK (libllt).
//
// connect(): finds/opens the scanner, sets resolution, exposure, trigger (In1 from the Nano, or
// internal = free running) and starts the transfer. Profiles arrive on the SDK's thread and are
// queued; read() takes them in order. Without the SDK installed (LASER_HHRP_HAS_SDK not defined)
// connect() always fails with a clear message.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace laser_hhrp {

struct ScanControlSettings {
  std::string interface;           // scanner IP / name as listed by the SDK; "" = first found
  uint32_t resolution = 1280;      // points per profile (highest supported value <= this)
  uint32_t exposure_time_us = 100;  // multiples of 10 us
  uint32_t idle_time_us = 900;      // free running only: period = exposure + idle
  bool external_trigger = false;    // true: one profile per rising edge on In1 (hardware sync)
  bool inputs_24v = false;          // input level: false = TTL 5 V (the Nano)
  uint32_t queue_size = 256;        // profiles buffered between the SDK thread and read()
};

struct LaserProfile {
  std::vector<float> x_m, z_m;
  std::vector<bool> valid;
  uint32_t profile_counter = 0;  // scanner counter (24 bit, wraps)
  double shutter_open_s = 0;     // scanner clock (wraps every 128 s)
  double shutter_close_s = 0;
  int64_t arrival_ns = 0;        // Pi system time when the SDK delivered it
  uint32_t lost_profiles = 0;    // dropped since connect (queue full or gaps in the counter)
};

class ScanControl {
 public:
  ScanControl();
  ~ScanControl();
  ScanControl(const ScanControl&) = delete;
  ScanControl& operator=(const ScanControl&) = delete;

  bool connect(const ScanControlSettings& settings, std::string& error);
  void disconnect();
  bool connected() const;
  /// Next profile (non-blocking). False if none waiting; `error` set if the scanner was lost.
  bool read(LaserProfile& profile, std::string& error);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace laser_hhrp
