// Laser clock -> Pi time. Used by laser_driver_node to stamp every profile with the Pi time of
// its exposure.
//
// Every scanCONTROL profile carries the laser's own shutter times (open / close, in seconds,
// microsecond resolution, wrapping every 128 s). The Pi sees the profile arrive later:
//   arrival (Pi) = exposure + transfer delay
// The delay (network, SDK thread, scheduling) is never negative and jitters by 0.5..several ms.
// So, over a sliding 2 s window:
//   offset          = arrival - laser time          (per profile)
//   min_offset      = smallest offset in the window  (true clock offset + shortest delay)
//   Pi time (exposure) = laser time + min_offset    (jitter removed)
// The window follows the slow drift between the two clocks (~50 ppm: 0.1 ms per window). The
// result still contains the shortest transfer delay; the software pairer subtracts it
// (sync.profile_latency_s).
#pragma once

#include <cstdint>
#include <deque>

namespace laser_hhrp {

class ScannerClock {
 public:
  explicit ScannerClock(double window_s = 2.0, double period_s = 128.0)
      : window_ns_(static_cast<int64_t>(window_s * 1e9)),
        period_ns_(static_cast<int64_t>(period_s * 1e9)) {}

  /// Start over (laser reconnected: its clock may have restarted).
  void reset() {
    started_ = false;
    wraps_ = 0;
    offsets_.clear();
  }

  /// One profile: scanner shutter open / close [s] (wrapping clock) and its Pi arrival [ns].
  /// Returns the Pi time of the exposure midpoint [ns].
  int64_t map(double shutter_open_s, double shutter_close_s, int64_t arrival_ns) {
    // Exposure midpoint in laser time.
    double mid_s = 0.5 * (shutter_open_s + shutter_close_s);
    if (shutter_close_s < shutter_open_s) mid_s = shutter_open_s;  // close wrapped: use open
    const int64_t raw_ns = static_cast<int64_t>(mid_s * 1e9);
    // Unwrap the 128 s clock into a continuous time.
    if (started_) {
      if (raw_ns < last_raw_ns_ - period_ns_ / 2) ++wraps_;  // the 128 s clock wrapped
      else if (raw_ns < last_raw_ns_ - window_ns_) reset();  // jumped back: scanner restarted
    }
    if (!started_) {
      started_ = true;
      wraps_ = 0;
    }
    last_raw_ns_ = raw_ns;
    const int64_t scanner_ns = raw_ns + wraps_ * period_ns_;
    const int64_t offset = arrival_ns - scanner_ns;  // Pi - laser, includes the transfer delay
    // Offsets far below the current minimum mean a clock jump: start over.
    if (!offsets_.empty() && offset < min_offset() - window_ns_) offsets_.clear();
    // Keep the offsets of the last window_s and return laser time + their minimum.
    offsets_.push_back(Sample{scanner_ns, offset});
    while (offsets_.size() > 1 && scanner_ns - offsets_.front().scanner_ns > window_ns_)
      offsets_.pop_front();
    return scanner_ns + min_offset();
  }

  /// Current minimum offset (Pi - scanner) [ns]; 0 before the first profile.
  int64_t min_offset() const {
    int64_t m = 0;
    bool first = true;
    for (const Sample& s : offsets_) {
      if (first || s.offset < m) m = s.offset;
      first = false;
    }
    return m;
  }

 private:
  struct Sample {
    int64_t scanner_ns;  // unwrapped laser time
    int64_t offset;      // Pi arrival - laser time
  };
  int64_t window_ns_, period_ns_;
  bool started_ = false;
  int64_t last_raw_ns_ = 0;
  int64_t wraps_ = 0;  // 128 s wraps since the start
  std::deque<Sample> offsets_;
};

}  // namespace laser_hhrp
