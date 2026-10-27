// Software synchronization: gives every free-running laser profile the encoder count at its
// exposure time. Used by synchronizer_node when sync.hardware_trigger is false (no trigger wire).
//
// Inputs
//   references  the Nano's encoder samples (240 Hz, also at rest), each with its Nano time and
//               its Pi time (through the Nano -> Pi clock mapping)
//   profiles    laser profiles, stamped with the Pi time of their exposure
//
// For each profile:
//   t = profile Pi time - latency                          (sync.profile_latency_s)
//   find the two references a, b with a.pi <= t <= b.pi
//   f = (t - a.pi) / (b.pi - a.pi)                         (0 at a, 1 at b)
//   count     = round(a.count + f * (b.count - a.count))   -> the scan-axis position
//   nano time = a.nano + f * (b.nano - a.nano)             -> where the IMU is interpolated
//
// Special cases: no reference after the profile for hold_s (standing still) -> keep the last
// count; profile before the scan's first reference -> cannot be placed, counted as unplaced.
// Accuracy is limited by the timing jitter of the clock mappings (a few ms, i.e. speed x jitter
// of travel); hardware synchronization has no such error.
#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

#include "laser_hhrp/frame_pairer.hpp"

namespace laser_hhrp {

class SoftwarePairer {
 public:
  using Pair = FramePairer::Pair;
  struct Stats {
    uint32_t paired = 0;
    uint32_t unplaced = 0;  // profiles before the scan's first reference
  };

  explicit SoftwarePairer(double latency_s = 0.0, double hold_s = 0.2)
      : latency_ns_(static_cast<int64_t>(latency_s * 1e9)),
        hold_ns_(static_cast<int64_t>(hold_s * 1e9)) {}

  void reset() {
    records_.clear();
    profiles_.clear();
    stats_ = Stats();
    next_index_ = 0;
  }
  void set_latency(double latency_s) { latency_ns_ = static_cast<int64_t>(latency_s * 1e9); }
  const Stats& stats() const { return stats_; }

  /// A position reference (an encoder sample: count + Nano time) and its Pi time.
  void add_record(const Trigger& t, int64_t host_ns) { records_.push_back(Record{t, host_ns}); }
  /// A profile (stamp_ns: Pi time of the exposure).
  void add_profile(const Raw& p) { profiles_.push_back(p); }

  /// Profiles that can be placed now, in arrival order (a profile waits until a reference after
  /// it has arrived, or hold_s has passed).
  std::vector<Pair> take(int64_t now_ns) {
    std::vector<Pair> out;
    while (!profiles_.empty()) {
      const Raw& p = profiles_.front();
      const int64_t t = p.stamp_ns - latency_ns_;  // Pi time of the exposure
      size_t after = 0;  // first reference at or after t
      while (after < records_.size() && records_[after].host_ns < t) ++after;
      if (records_.empty()) {  // no reference yet: the scan's first one may still come
        if (now_ns - t <= hold_ns_) break;
        ++stats_.unplaced;
        profiles_.pop_front();
        continue;
      }
      if (after == 0 && records_.front().host_ns > t) {  // before the scan's first reference
        ++stats_.unplaced;
        profiles_.pop_front();
        continue;
      }
      Pair pair;
      if (after < records_.size()) {  // between two references a, b: linear interpolation
        const Record& b = records_[after];
        const Record& a = after > 0 ? records_[after - 1] : b;
        const double f = b.host_ns > a.host_ns
                             ? static_cast<double>(t - a.host_ns) / (b.host_ns - a.host_ns)
                             : 1;
        pair.trigger = b.msg;
        pair.trigger.count = static_cast<int32_t>(
            std::lround(a.msg.count + f * static_cast<double>(b.msg.count - a.msg.count)));
        pair.trigger.device_ns =
            a.msg.device_ns +
            static_cast<int64_t>(f * static_cast<double>(b.msg.device_ns - a.msg.device_ns));
      } else if (now_ns - t > hold_ns_) {  // no newer reference: standing still, keep the count
        const Record& a = records_.back();
        pair.trigger = a.msg;
        // Nano time = last reference + the Pi time since then (clocks equal over a short time).
        pair.trigger.device_ns = a.msg.device_ns + (t - a.host_ns);
        pair.trigger.flags = 0;
      } else {
        break;  // wait for the next reference
      }
      pair.trigger.index = next_index_++;  // frame number (software sync has no trigger numbers)
      pair.profile = p;
      pair.arrival_ns = now_ns;
      ++stats_.paired;
      out.push_back(pair);
      profiles_.pop_front();
      // References older than the one just used are not needed any more.
      while (records_.size() > 2 && records_[1].host_ns < t) records_.pop_front();
    }
    return out;
  }

 private:
  struct Record {
    Trigger msg;      // count, Nano time
    int64_t host_ns;  // Pi time
  };
  int64_t latency_ns_;
  int64_t hold_ns_;
  std::deque<Record> records_;
  std::deque<Raw> profiles_;
  Stats stats_;
  uint32_t next_index_ = 0;
};

}  // namespace laser_hhrp
