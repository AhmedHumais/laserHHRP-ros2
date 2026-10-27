# simulator_node

Stand-in for both layer-1 drivers when there is no hardware (`simulate:=true`). It publishes the
same topics as `nano_driver_node` and `laser_driver_node`, reacts to `nano/command` like the
firmware, and simulates a roller that rolls only while a scan is triggering, over a flat surface
with one rivet head. Used for development and for the end-to-end test
(`timed_scan.py` + `check_session.py`).

Source: `src/simulator_node.cpp`.

## Interfaces

| Kind | Name | Type / default | Notes |
| --- | --- | --- | --- |
| pub | `nano/encoder`, `nano/trigger`, `nano/imu`, `imu/data`, `nano/status`, `nano/clock` | as nano_driver_node | |
| pub | `laser/raw`, `laser/profile`, `laser/status` | as laser_driver_node | |
| sub | `nano/command` | `laser_hhrp/NanoCommand` | START / STOP / ZERO |
| param | `sim.speed_mps` | `0.02` | rolling speed while triggering |
| param | `encoder.roller_diameter_m`, `encoder.counts_per_turn` | `0.030`, `4096` | |
| param | `sync.hardware_trigger` | `false` | true: one profile per trigger |
| param | `sync.software_rate_hz` | `120.0` | free-running profile rate |
| param | `laser.live_view_every` | `10` | |

## Processing

```
startup:
    t0 = now; publish ClockMap once (latched):
        device_ns = device(t0), host_ns = t0, drift = 1/(1 − 20e-6) − 1
    1 ms timer -> step()

device(host) = (host − t0)·(1 − 20e-6) + 1000 ns        simulated Nano clock: 20 ppm slow, 1 µs offset

on nano/command:
    START: pitch = max(1, counts_per_trigger), min_period = min_trigger_period_us·1000,
           anchor = count, trigger_index = 0, triggering = true, fired_once = false
    STOP:  triggering = false
    ZERO:  count = anchor = 0

step():                                    every 1 ms
    if triggering: travel += speed·Δt
    target = floor(travel / (π·d) · counts_per_turn)
    for each new count i between prev and now:  count++, encoder_read(time interpolated in the step)
    software sync: for each free-run period passed: make_profile(that time)
    every 4.167 ms: publish_samples()
    every 500 ms:   publish_status()          (NanoStatus + LaserStatus connected)

encoder_read(t):                           same decision as arduino_nano/laser_trigger.h
    if not triggering or |count − anchor| < pitch: return
    if fired_once and t − last_trigger < min_period: held_back = true, return
    direction = sign(count − anchor)
    overspeed = held_back or |count − anchor| >= 2·pitch
    anchor += direction · floor(|count − anchor|/pitch) · pitch
    held_back = false, fired_once = true, last_trigger = t
    publish TriggerRecord{index++, count, direction, flags = overspeed, device(t)}
    hardware sync: make_profile(t)

make_profile(t):
    640 points, x from −5 mm to +5 mm, y = position = total_counts/counts_per_turn · π·d
    z = 52.5 mm − (20 µm if hypot(x, y − 20 mm) < 2.5 mm else 0)
    profile_counter++ (24 bit), shutter_open = device(t) mod 128 s, stamp = now
    publish RawProfile (+ PointCloud2 every Nth)

publish_samples():
    EncoderSample{count, raw = total_counts mod counts_per_turn}
    ImuSample{accel = (0, 0, 9.80665), gyro_i = bias_i + 0.002·noise}
        bias = (0.012, −0.021, 0.006) rad/s; noise = deterministic LCG, uniform, unit variance
```

## Notes

- Uses topics directly, not the serial protocol: the packet code is covered by the unit tests only.
- At 20 mm/s and 92 µm pitch the trigger rate would be 217 Hz, above the 120 Hz limit, so in
  hardware sync many triggers are flagged overspeed (as on the real device at that speed).
- The IMU is perfectly level and still, so the gyro calibration always succeeds.

## Where in the code

`SimulatorNode::device_ns/on_command/step/encoder_read/make_profile/publish_samples/publish_status/noise`.
