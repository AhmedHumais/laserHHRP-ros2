# Synchronization contract

How the encoder, the IMU and every laser profile are brought to the same instant, in both
synchronization modes. Code: `include/laser_hhrp/clock_sync.hpp`, `scanner_clock.hpp`,
`frame_pairer.hpp`, `software_pairer.hpp`, and the nodes `nano_driver_node`, `laser_driver_node`,
`synchronizer_node`.

**Rule:** pairing is always done in the clock of the device that measured the data. Pi time is
only used to stamp the finished frame.

## 1. The three clocks

| Clock | Comes from | Used for |
| --- | --- | --- |
| **Nano clock** | `micros()` extended to 64 bit (`now_us()` in `arduino_nano.ino`) | Encoder samples, trigger edges, IMU samples: all stamped by one clock |
| **Laser clock** | The scanner's own shutter timer (`shutter_open_s`, wraps every 128 s), plus a 24-bit profile counter | When each exposure happened |
| **Pi clock** | System time (= ROS time) | Header stamps on published messages |

Encoder, trigger and IMU share the Nano clock, so they are aligned with each other exactly, with no
transfer jitter.

## 2. Nano clock → Pi clock (`clock_sync.hpp`)

`nano_driver_node` runs a round-trip exchange: 10 Hz until it has 8 good points, then 2 Hz.

```
Pi  ── TimeRequest(seq, host_send) ──►  Nano
Pi  ◄── TimeReply(seq, host_send, nano_receive, nano_send) ── Nano
        host_receive = arrival time on the Pi
```

For each reply:

1. **Round trip** `rtt = host_receive − host_send`. Rejected if `rtt > 50 ms`, or more than 2 ms
   above the shortest of the last 16. A long round trip is usually delayed in one direction only,
   which would shift the offset by up to half of it.
2. **Midpoints:** Nano time `nano_receive + processing/2` is matched with Pi time
   `host_send + rtt/2` (`processing = nano_send − nano_receive`).
3. **Line fit:** least squares over the last 32 points (about 16 s):

   ```
   pi_time = host_ns + (nano_time − device_ns) · (1 + drift)
   ```

   This gives the offset and the crystal drift. A drift above 1 % is treated as nonsense and the
   fit restarts.

The result is published on `nano/clock` (`ClockMap`); every node that needs Pi time for a Nano
timestamp applies that formula. Accuracy is about RTT/2, roughly 1 ms with the FTDI low-latency
setting the driver turns on.

**This mapping is never used for pairing.** It only decides what Pi stamp a frame gets.

## 3. Laser clock → Pi clock (`scanner_clock.hpp`)

Each profile reaches the Pi some time after its exposure. The delay (network, SDK thread,
scheduling) is never negative and jitters by milliseconds. `laser_driver_node`:

1. takes the exposure midpoint in laser time, `(shutter_open + shutter_close) / 2`, unwrapping the
   128 s wrap;
2. computes `offset = arrival_pi − laser_time` for each profile;
3. keeps the **smallest** offset over a 2 s window (the true offset plus the shortest transfer
   delay);
4. stamps the profile `laser_time + min_offset`.

This removes the jitter (measured on the old system: 8.330 ± 0.002 ms between 120 Hz profiles).
The 2 s window follows the slow drift between the clocks (about 50 ppm, about 0.1 ms per window).
Result: `laser/raw` with `header.stamp` close to the exposure time in Pi time.

## 4. Hardware sync (`hardware_trigger:=true`, `frame_pairer.hpp`)

The wire Nano D3 → laser In1 makes **one exposure per trigger pulse**.

**On the Nano:** every `counts_per_trigger` encoder counts (default 4 = 92 µm pitch), never faster
than `min_trigger_period_us` (120 Hz), it:

- drives D3 high for 20 µs; the laser exposes on the rising edge;
- sends a `TriggerRecord`: trigger number *k*, count, direction, Nano time of the edge.

**On the Pi (synchronizer):** trigger *k* belongs to profile *k*. The only unknown is the offset
between trigger numbers and the laser's profile counter (it doesn't start at 0, and a first
profile or record may be lost).

1. **Locking the offset:** a hand-held scan has irregular intervals between pulses (9 ms, 11 ms,
   8.5 ms…). The pairer compares the intervals between the first ~9 trigger records (Nano clock)
   with the intervals between the first 6 profiles (laser clock) for offsets −3…+3. Only one
   offset makes the two sequences match (mean error ≤ 2 ms).
2. **Pairing by number:** after locking, profile counter *c* is paired with trigger
   `c − ref_counter + ref_index`. Lost profiles or records are detected (unpaired after 0.5 s
   → counted as missing) and never mis-paired.
3. **Timing check per pair:** the time since the last good pair must agree in the Nano clock and
   the laser clock within 2 ms, otherwise the frame is flagged `timing_mismatch`. The comparison is
   frame to frame, so drift between the two clocks never accumulates.

**The frame:**

```
position_m = sign · (count − count_at_scan_start) · metres_per_count
imu        = interpolated at the trigger's Nano time        (exact: same clock)
stamp      = Pi time of the trigger edge (ClockMap)
```

Profiles are **equally spaced in distance**, whatever the rolling speed.

## 5. Software sync (`hardware_trigger:=false`, `software_pairer.hpp`)

No trigger wire. The laser runs free at `sync.software_rate_hz` (120 Hz). The Nano still sends
**encoder samples at 240 Hz**, also while standing still.

1. **Position references:** each `EncoderSample` gives a pair of times: its Nano time and its Pi
   time (through the `ClockMap`).
2. **Profile time:** `t = profile Pi stamp (section 3) − sync.profile_latency_s` (2 ms).
3. **Interpolation** between the two encoder samples a, b around `t`:

   ```
   f         = (t − a.pi) / (b.pi − a.pi)
   count     = round(a.count + f · (b.count − a.count))       → position
   nano_time = a.nano + f · (b.nano − a.nano)                 → IMU at the same instant
   ```

4. **Edge cases:**
   - Profiles from before the scan's first encoder sample are counted as `unplaced`.
   - If no newer sample comes within 0.2 s (standing still), the last count is kept.

Profiles are **equally spaced in time**. Accuracy is limited by the jitter in the two clock
mappings, a few ms (position error ≈ speed × jitter, e.g. 20 µm at 10 mm/s). Hardware sync has no
such error.

## 6. Shared final step (synchronizer)

In both modes a frame waits up to 100 ms for the IMU sample (and the odometry pose, if the
extrinsics are calibrated) **after** its instant, so it can be interpolated, not extrapolated:

```
k   = (t − a.t) / (b.t − a.t)                 a, b: IMU samples around t (Nano clock)
imu = a + k · (b − a)                         accel and gyro, per axis
pose: position linear, orientation slerp
```

Then it publishes `scan/frame` (profile, position, direction, interpolated IMU, flags) and
`scan/pose` (the laser pose with the **same stamp** as the frame).

The only difference between the modes is where each profile's encoder count comes from: the
trigger record with the matching number (hardware) or interpolation between encoder samples
(software).

## Changing this contract

`ScanFrame` is what layers 4 and 5 and the recorded data rely on. If you change its fields or
their meaning, update `recorder_node`, `scripts/read_session.py`, `scripts/check_session.py` and
this file together.
