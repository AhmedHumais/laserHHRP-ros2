// Nano clock -> Pi clock, from clock exchanges (request with Pi time, reply with Nano times).
//
// Each accepted exchange gives one point (Nano time at the reply midpoint, Pi time at the round
// trip midpoint). A straight line (offset + drift) is fitted through the last 32 points (16 s at
// 2 Hz). Exchanges whose round trip is more than 2 ms above the shortest recent one are skipped:
// a long round trip is usually one-sided and would shift the offset by up to half of it.
// Pi-time accuracy is about RTT/2 (~1 ms); trigger <-> IMU alignment never depends on it, both
// use the Nano clock.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace laser_hhrp {

class ClockSync {
 public:
  void sent(uint32_t sequence, int64_t host_send_ns) {
    sequence_ = sequence;
    host_send_ns_ = host_send_ns;
    pending_ = true;
  }

  /// A reply; true if it was used.
  bool reply(uint32_t sequence, int64_t host_send_ns, int64_t device_receive_ns,
             int64_t device_send_ns, int64_t host_receive_ns) {
    if (!pending_ || sequence != sequence_ || host_send_ns != host_send_ns_) return false;
    pending_ = false;
    const int64_t rtt = host_receive_ns - host_send_ns;
    const int64_t processing = device_send_ns - device_receive_ns;
    if (rtt <= 0 || rtt > kMaxRttNs || processing < 0 || processing > rtt) return false;

    // Shortest round trip of the last 16 exchanges.
    recent_rtt_[recent_next_] = rtt;
    recent_next_ = (recent_next_ + 1) % kRecent;
    if (recent_count_ < kRecent) ++recent_count_;
    int64_t best = rtt;
    for (size_t i = 0; i < recent_count_; ++i) best = recent_rtt_[i] < best ? recent_rtt_[i] : best;
    if (rtt > best + kRttToleranceNs) return false;

    add(device_receive_ns + processing / 2, host_send_ns + rtt / 2);
    if (!std::isfinite(drift_) || std::fabs(drift_) > 0.01) {  // implausible: start over
      reset();
      return false;
    }
    return true;
  }

  bool valid() const { return count_ > 0; }
  size_t samples() const { return count_; }
  double drift() const { return drift_; }
  int64_t to_host(int64_t device_ns) const {
    if (count_ == 0) return device_ns;
    return ref_host_ + static_cast<int64_t>((device_ns - ref_device_) * (1.0 + drift_));
  }
  void reset() {
    count_ = next_ = 0;
    recent_count_ = recent_next_ = 0;
    pending_ = false;
    drift_ = 0;
  }

 private:
  /// Least-squares line host - device = a + drift * device over the window.
  void add(int64_t device_ns, int64_t host_ns) {
    dev_[next_] = device_ns;
    host_[next_] = host_ns;
    next_ = (next_ + 1) % kWindow;
    if (count_ < kWindow) ++count_;
    const size_t first = (next_ + kWindow - count_) % kWindow;
    const int64_t d0 = dev_[first], h0 = host_[first];
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t k = 0; k < count_; ++k) {
      const size_t i = (first + k) % kWindow;
      const double x = static_cast<double>(dev_[i] - d0);
      const double y = static_cast<double>((host_[i] - h0) - (dev_[i] - d0));
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
    }
    const double n = static_cast<double>(count_);
    const double den = n * sxx - sx * sx;
    drift_ = (count_ >= 2 && den > 0) ? (n * sxy - sx * sy) / den : 0.0;
    const double mean_x = sx / n, mean_y = sy / n;
    ref_device_ = d0 + static_cast<int64_t>(mean_x);
    ref_host_ = h0 + static_cast<int64_t>(mean_x) + static_cast<int64_t>(mean_y);
  }

  static constexpr int64_t kMaxRttNs = 50000000;       // 50 ms
  static constexpr int64_t kRttToleranceNs = 2000000;  // 2 ms
  static constexpr size_t kWindow = 32;
  static constexpr size_t kRecent = 16;

  int64_t dev_[kWindow] = {}, host_[kWindow] = {};
  size_t count_ = 0, next_ = 0;
  double drift_ = 0;
  int64_t ref_device_ = 0, ref_host_ = 0;
  int64_t recent_rtt_[kRecent] = {};
  size_t recent_count_ = 0, recent_next_ = 0;
  uint32_t sequence_ = 0;
  int64_t host_send_ns_ = 0;
  bool pending_ = false;
};

}  // namespace laser_hhrp
