// Hardware synchronization: pairs each trigger record from the Nano with the laser profile that
// trigger produced. Used by synchronizer_node when sync.hardware_trigger is true.
//
// Idea: the Nano pulses the laser's In1 once per pitch, so trigger k makes exactly one profile.
// Triggers are numbered by the Nano (0, 1, 2, ... since scan start) and profiles by the laser
// (24-bit profile counter, starting anywhere). Once we know the offset between the two numbers,
// pairing is a simple lookup, and a profile or record lost on the way is detected, never
// mis-paired.
//
// How it works:
//   1. Lock (try_lock): collect the first few triggers and profiles. Compare the time gaps
//      between consecutive triggers (Nano clock) with the gaps between consecutive profiles
//      (laser clock) for each candidate offset -3..+3. Hand-held rolling is irregular, so only the
//      right offset makes the two gap sequences match.
//   2. Pair (take): after locking, profile counter c belongs to trigger
//      c - ref_counter + ref_index. Pairs are output in trigger order; an item waiting longer
//      than give_up_s (Pi time) is counted as missing and dropped.
//   3. Check (check_timing): for every pair, the time since the previous good pair must be the
//      same in the Nano clock and the laser clock (within 2 ms). Compared pair to pair, so the
//      slow drift between the two clocks never adds up.
//
// Clocks: Trigger.device_ns = Nano clock; Raw.shutter_open_s = laser clock (wraps every 128 s);
// Raw.stamp_ns and all arrival times = Pi time (only used for the give-up timeouts).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "laser_hhrp/msg/raw_profile.hpp"

namespace laser_hhrp {

/// A trigger record (or, in software sync, an encoder position reference). Times: Nano clock.
struct Trigger {
  int64_t device_ns = 0;
  uint32_t index = 0;
  int32_t count = 0;
  int8_t direction = 0;
  uint8_t flags = 0;  // bit 0: overspeed
};

/// A received profile; stamp_ns is its Pi time.
struct Raw {
  int64_t stamp_ns = 0;
  uint32_t profile_counter = 0;
  double shutter_open_s = 0;
  msg::RawProfile::ConstSharedPtr msg;  // the points
};

class FramePairer {
 public:

  /// One synchronized result: a trigger and its profile.
  struct Pair {
    Trigger trigger;               // Nano clock
    Raw profile;                   // Pi time
    int64_t arrival_ns = 0;        // Pi time when both halves had arrived
    bool timing_mismatch = false;  // failed the timing check (frame gets flag bit 1)
  };
  struct Stats {
    uint32_t paired = 0;
    uint32_t missing_profiles = 0;  // triggers whose profile never arrived
    uint32_t missing_triggers = 0;  // profiles whose trigger record never arrived
    uint32_t timing_mismatches = 0;
  };

  /// lock_samples: profiles needed to lock; max_offset: offsets tried (+-);
  /// timing_tolerance_s: allowed gap difference; give_up_s: wait for the other half this long.
  explicit FramePairer(size_t lock_samples = 6, int max_offset = 3,
                       double timing_tolerance_s = 0.002, double give_up_s = 0.5)
      : lock_samples_(lock_samples),
        max_offset_(max_offset),
        tolerance_s_(timing_tolerance_s),
        give_up_ns_(static_cast<int64_t>(give_up_s * 1e9)) {}

  /// Forget everything (new scan, or resume after pause: the Nano restarts its numbering).
  void reset() {
    pending_triggers_.clear();
    pending_profiles_.clear();
    early_triggers_.clear();
    early_profiles_.clear();
    locked_ = false;
    stats_ = Stats();
  }
  bool locked() const { return locked_; }
  int offset() const { return offset_; }
  const Stats& stats() const { return stats_; }

  /// A trigger record that arrived at Pi time `arrival_ns` (default: its own Nano time, for tests
  /// where both clocks are the same). Before the lock it is collected; after, stored by number.
  void add_trigger(const Trigger& t, int64_t arrival_ns = -1) {
    const Timed<Trigger> item{t, arrival_ns < 0 ? t.device_ns : arrival_ns};
    if (!locked_) {
      early_triggers_.push_back(item);
      try_lock();
    } else {
      pending_triggers_[static_cast<int64_t>(t.index)] = item;
    }
  }
  /// A profile (stamp_ns: Pi time). Before the lock it is collected; after, stored under the
  /// trigger number it belongs to.
  void add_profile(const Raw& p) {
    const Timed<Raw> item{p, p.stamp_ns};
    if (!locked_) {
      early_profiles_.push_back(item);
      try_lock();
    } else {
      pending_profiles_[profile_index(p)] = item;
    }
  }

  /// Completed pairs in trigger order; items waiting longer than give_up_s (Pi time) are counted
  /// as missing and dropped.
  std::vector<Pair> take(int64_t now_ns) {
    std::vector<Pair> out;
    // Walk the waiting triggers in number order: matched -> output; waited too long -> missing.
    for (auto it = pending_triggers_.begin(); it != pending_triggers_.end();) {
      auto p = pending_profiles_.find(it->first);
      if (p != pending_profiles_.end()) {
        Pair pair;
        pair.trigger = it->second.msg;
        pair.profile = p->second.msg;
        pair.arrival_ns = std::max(it->second.arrival_ns, p->second.arrival_ns);
        pair.timing_mismatch = !check_timing(pair.trigger, pair.profile);
        if (pair.timing_mismatch) ++stats_.timing_mismatches;
        ++stats_.paired;
        out.push_back(pair);
        pending_profiles_.erase(p);
        it = pending_triggers_.erase(it);
      } else if (now_ns - it->second.arrival_ns > give_up_ns_) {
        ++stats_.missing_profiles;
        it = pending_triggers_.erase(it);
      } else {
        ++it;
      }
    }
    // Profiles still without a trigger record after give_up_s: their record was lost.
    for (auto it = pending_profiles_.begin(); it != pending_profiles_.end();) {
      if (now_ns - it->second.arrival_ns > give_up_ns_) {
        ++stats_.missing_triggers;
        it = pending_profiles_.erase(it);
      } else {
        ++it;
      }
    }
    return out;
  }

 private:
  // An item and the Pi time it arrived (for the give-up timeout).
  template <typename M>
  struct Timed {
    M msg;
    int64_t arrival_ns;
  };

  // Difference of two 24-bit profile counters (b after a), wrap included.
  static uint32_t counter_delta(uint32_t a, uint32_t b) { return (b - a) & 0xffffffu; }
  static double scanner_delta_s(double a, double b) {  // shutter clock wraps every 128 s
    double d = b - a;
    if (d < 0) d += 128.0;
    return d;
  }
  // Trigger number of a profile, from the locked reference pair.
  int64_t profile_index(const Raw& p) const {
    return static_cast<int64_t>(counter_delta(ref_counter_, p.profile_counter)) + ref_index_;
  }
  /// Timing check of one pair:
  ///   nano_s    = trigger time - reference trigger time        (Nano clock)
  ///   scanner_s = shutter time - reference shutter time        (laser clock)
  ///   ok if |nano_s - scanner_s| <= tolerance (modulo the 128 s wrap)
  /// A good pair becomes the new reference; a bad one does not, so one bad pair is flagged alone.
  bool check_timing(const Trigger& t, const Raw& p) {
    const double nano_s = (t.device_ns - ref_trigger_ns_) * 1e-9;
    const double scanner_s = scanner_delta_s(ref_shutter_s_, p.shutter_open_s);
    const double diff = std::fmod(std::fabs(nano_s - scanner_s), 128.0);
    const bool ok = std::min(diff, 128.0 - diff) <= tolerance_s_;
    if (ok) {
      ref_trigger_ns_ = t.device_ns;
      ref_shutter_s_ = p.shutter_open_s;
    }
    return ok;
  }

  /// Finds the offset o (early profile j <-> early trigger j + o) whose time gaps match best:
  ///   error(o) = mean over j of | gap between profiles j-1, j  -  gap between triggers k-1, k |
  /// with k = j + o (profile gaps divided by the counter step, in case a profile was lost).
  void try_lock() {
    // Enough triggers to try every offset, and enough profiles?
    if (early_triggers_.size() < lock_samples_ + static_cast<size_t>(max_offset_) ||
        early_profiles_.size() < lock_samples_)
      return;
    double best = 1e9;
    int best_o = 0;
    for (int o = -max_offset_; o <= max_offset_; ++o) {
      double err = 0;
      int n = 0;
      for (size_t j = 1; j < early_profiles_.size(); ++j) {
        const long k = static_cast<long>(j) + o;
        if (k < 1 || k >= static_cast<long>(early_triggers_.size())) continue;
        const Raw& a = early_profiles_[j - 1].msg;
        const Raw& b = early_profiles_[j].msg;
        const double dp =
            scanner_delta_s(a.shutter_open_s, b.shutter_open_s) /
            std::max<uint32_t>(1, counter_delta(a.profile_counter, b.profile_counter));
        const double dt =
            (early_triggers_[k].msg.device_ns - early_triggers_[k - 1].msg.device_ns) * 1e-9;
        err += std::fabs(dp - dt);
        ++n;
      }
      if (n >= static_cast<int>(lock_samples_) - 1 && err / n < best) {
        best = err / n;
        best_o = o;
      }
    }
    // Even the best offset does not fit (e.g. profiles from before the scan): drop the oldest
    // profile and try again when more arrive.
    if (best > tolerance_s_) {
      early_profiles_.pop_front();
      return;
    }
    offset_ = best_o;
    // Profile j <-> trigger j + offset. A negative offset means the first profiles' trigger
    // records were lost: those profiles cannot be placed and are dropped.
    long k0 = best_o;
    while (k0 < 0 && !early_profiles_.empty()) {
      early_profiles_.pop_front();
      ++stats_.missing_triggers;
      ++k0;
    }
    if (early_profiles_.empty()) return;
    // Reference pair: the first profile and its trigger. All later pairing counts from here.
    const Raw& first = early_profiles_.front().msg;
    const Trigger& t0 = early_triggers_[static_cast<size_t>(k0)].msg;
    ref_counter_ = first.profile_counter;
    ref_index_ = static_cast<int64_t>(t0.index);
    ref_trigger_ns_ = t0.device_ns;
    ref_shutter_s_ = first.shutter_open_s;
    locked_ = true;
    // Everything collected so far now goes into the numbered waiting lists.
    for (const auto& t : early_triggers_) pending_triggers_[static_cast<int64_t>(t.msg.index)] = t;
    for (const auto& p : early_profiles_) pending_profiles_[profile_index(p.msg)] = p;
    early_triggers_.clear();
    early_profiles_.clear();
  }

  size_t lock_samples_;
  int max_offset_;
  double tolerance_s_;
  int64_t give_up_ns_;
  std::deque<Timed<Trigger>> early_triggers_;          // collected before the lock
  std::deque<Timed<Raw>> early_profiles_;
  std::map<int64_t, Timed<Trigger>> pending_triggers_;  // after the lock, by trigger number
  std::map<int64_t, Timed<Raw>> pending_profiles_;
  bool locked_ = false;
  int offset_ = 0;
  uint32_t ref_counter_ = 0;    // reference pair: profile counter ...
  int64_t ref_index_ = 0;       // ... and its trigger number
  int64_t ref_trigger_ns_ = 0;  // last good pair (timing check): Nano time ...
  double ref_shutter_s_ = 0;    // ... and laser time
  Stats stats_;
};

}  // namespace laser_hhrp
