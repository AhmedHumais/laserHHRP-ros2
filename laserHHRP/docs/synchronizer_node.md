# synchronizer_node

Layer 3 (synchronization). Turns laser profiles into `ScanFrame`s: each profile gets its encoder
count (hence its position along the scan axis), the IMU sample and the laser pose at the same
instant, and a Pi-time stamp. Hardware sync pairs profiles with trigger records by number; software
sync interpolates the encoder at each profile's time. Frames are produced only while a scan runs.
The timing contract and the pairing algorithms are described in
[synchronization.md](synchronization.md); this page follows the node's own steps.

Source: `src/synchronizer_node.cpp`, `include/laser_hhrp/frame_pairer.hpp`,
`include/laser_hhrp/software_pairer.hpp`, `include/laser_hhrp/pose.hpp`,
`include/laser_hhrp/encoder_scale.hpp`.

## Interfaces

| Kind | Name | Type / default | Notes |
| --- | --- | --- | --- |
| sub | `nano/trigger` | `laser_hhrp/TriggerRecord` | hardware sync |
| sub | `nano/encoder` | `laser_hhrp/EncoderSample` | software sync references |
| sub | `laser/raw` | `laser_hhrp/RawProfile` | |
| sub | `imu/corrected` | `laser_hhrp/ImuSample` | kept 2 s |
| sub | `odometry/laser_pose` | `laser_hhrp/DevicePose` | kept 2 s |
| sub | `nano/clock` | `laser_hhrp/ClockMap` | latched |
| sub | `scan/status` | `laser_hhrp/ScanStatus` | start / pause / stop |
| sub | `scan/motion` | `laser_hhrp/Motion` | encoder count at scan start |
| sub | `calibration/status` | `laser_hhrp/CalibrationStatus` | measured metres per count |
| pub | `scan/frame` | `laser_hhrp/ScanFrame` | one per profile |
| pub | `scan/pose` | `geometry_msgs/PoseStamped` | same stamp as the frame, when a pose exists |
| pub | `scan/sync` | `laser_hhrp/SyncStats` | 10 Hz |
| param | `sync.hardware_trigger` | `false` | |
| param | `sync.profile_latency_s` | `0.002` | software sync |
| param | `encoder.*` | see EncoderScale | |

## Processing

```
on scan/status (state changes only):
    IDLE/ERROR -> SCANNING (new scan):
        clear ready queue, reset both pairers
        start_count = last encoder count (scan/motion)
        metres_per_count, sign = EncoderScale (fixed for the scan)
        frames = 0, active = true
    PAUSED -> SCANNING (resume): reset both pairers (Nano restarts trigger numbers; positions keep start_count)
    SCANNING/PAUSED -> IDLE/ERROR: publish everything still in the ready queue, active = false

on imu/corrected:        append; drop samples older than newest − 2 s
on odometry/laser_pose:  append (clear first if its time is not after the newest); drop older than newest − 2 s
on nano/trigger:         if active and hardware: frame_pairer.add_trigger(record, arrival = now)
on nano/encoder:         if active and software:
                             ref = {count, Nano time, direction = sign(count − previous count)}
                             software_pairer.add_record(ref, pi_time = to_host(Nano time))
on laser/raw:            if active: Raw{stamp, counter, shutter_open, msg} -> the mode's pairer

step():                                       every 2 ms
    if active: ready += (hardware ? frame_pairer : software_pairer).take(now)
    while ready not empty:
        pair = front
        imu_ready  = newest IMU time >= pair.time
        pose_ready = no poses, or newest pose time >= pair.time, or newest pose older than 200 ms
        if not (imu_ready and pose_ready) and now − pair.arrival < 100 ms: stop (wait)
        publish(pair); pop
    every 100 ms: publish SyncStats

publish(pair):
    stamp       = to_host(pair.time)
    index       = frames++                              (unique within the scan)
    position_m  = sign · (count − start_count) · metres_per_count
    direction   = sign · pair.direction
    imu         = linear interpolation at pair.time, imu_valid = bracketed
    flags       = bit0 overspeed (from the trigger) | bit1 timing mismatch (hardware pairing)
    x, z, valid = from the profile
    if a pose brackets pair.time: publish scan/pose (position linear, orientation slerp), same stamp
    publish scan/frame
```

### Formulas

- Nano → Pi: `to_host(t) = host_ns + (t − device_ns)·(1 + drift)` from the latest `ClockMap`
  (now, before the first map).
- IMU at `t` between samples `a`, `b`: `f = (t − t_a)/(t_b − t_a)`,
  `v(t) = v_a + f·(v_b − v_a)` per axis of accel and gyro.
- Pose at `t`: `p = p_a + f·(p_b − p_a)`, `q = slerp(q_a, q_b, f)`.
- Software sync (inside `SoftwarePairer::take`): profile time `t = stamp − latency`;
  `count = round(c_a + f·(c_b − c_a))`, `nano_time = n_a + f·(n_b − n_a)` with `f` from the Pi times
  of the two references around `t`; the same count is kept if no newer reference after 0.2 s.
- Hardware sync (inside `FramePairer`): offset lock, pairing by profile counter, timing check
  within 2 ms, give-up after 0.5 s: see [synchronization.md](synchronization.md).

### SyncStats

`locked` = hardware: pairer locked, software: at least one profile placed; `missing_profiles`,
`missing_triggers`, `timing_mismatches` from the frame pairer; `unplaced` from the software pairer;
`last_position_m`.

## Edge cases

- Between scans all incoming triggers, encoder samples and profiles are ignored.
- A frame never waits more than 100 ms for its IMU / pose; then `imu_valid = false`.
- Trigger records and profiles arriving before `scan/status` says SCANNING are lost; the pairer
  locks on the following ones.
- Speed or roller changes during a scan take effect at the next scan (scale fixed at start).

## Where in the code

`SynchronizerNode::follow_scan/add_software_reference/step/publish/imu_at/pose_at/pose_ready/
to_host/publish_stats`; `FramePairer::add_trigger/add_profile/take/try_lock/check_timing`;
`SoftwarePairer::add_record/add_profile/take`; `interpolate()` (pose.hpp).
