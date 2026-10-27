// Spatial trigger for the laser: one pulse every `counts_per_trigger` encoder counts of roller
// travel (either direction), but never faster than `min_period_us` (the laser's max profile rate).
#pragma once

#include <stdint.h>

class LaserTrigger {
 public:
  // Starts triggering; the first pulse comes one pitch away from `count`.
  void start(int32_t count, uint16_t counts_per_trigger, uint32_t min_period_us) {
    pitch_ = counts_per_trigger == 0 ? 1 : counts_per_trigger;
    min_period_us_ = min_period_us;
    anchor_ = count;
    index_ = 0;
    active_ = true;
    fired_once_ = false;
    held_back_ = false;
  }
  void stop() { active_ = false; }
  bool active() const { return active_; }
  uint32_t index() const { return index_; }  // pulses fired since start()

  // Call after every encoder read. Returns true when a pulse must be fired now; then
  // `direction` (+1/-1) and `overspeed` describe it.
  bool update(int32_t count, uint64_t now_us, int8_t& direction, bool& overspeed) {
    if (!active_) return false;
    const int32_t travel = count - anchor_;
    const int32_t distance = travel < 0 ? -travel : travel;
    if (distance < int32_t(pitch_)) return false;  // not one pitch yet

    // Too soon after the last pulse: the laser could not take it. Fire later, flagged.
    if (fired_once_ && now_us - last_us_ < min_period_us_) {
      held_back_ = true;
      return false;
    }

    direction = travel > 0 ? 1 : -1;
    // More than one pitch moved (too fast, or held back): one pulse, flagged overspeed.
    overspeed = held_back_ || distance >= 2 * int32_t(pitch_);
    // Move the anchor to the last pitch boundary we passed.
    anchor_ += direction * (distance / int32_t(pitch_)) * int32_t(pitch_);
    held_back_ = false;
    fired_once_ = true;
    last_us_ = now_us;
    ++index_;
    return true;
  }

 private:
  uint16_t pitch_ = 1;
  uint32_t min_period_us_ = 0;
  int32_t anchor_ = 0;
  uint32_t index_ = 0;
  bool active_ = false;
  bool fired_once_ = false;
  bool held_back_ = false;
  uint64_t last_us_ = 0;
};
