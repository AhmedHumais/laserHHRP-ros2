# laser_driver_node

Layer 1 (hardware interface). Connects to the Micro-Epsilon scanCONTROL 29xx through the
scanCONTROL SDK, configures its trigger (external In1 for hardware sync, internal free running for
software sync), and publishes every profile stamped with the Pi time of its exposure, computed
from the scanner's own shutter clock. It also publishes a thinned live view for RViz and the
connection state. Timing contract: [synchronization.md](synchronization.md).

Source: `src/laser_driver_node.cpp`, `src/scancontrol.cpp`, `include/laser_hhrp/scancontrol.hpp`,
`include/laser_hhrp/scanner_clock.hpp`, `include/laser_hhrp/profile_cloud.hpp`.

## Interfaces

| Kind | Name | Type / default | Notes |
| --- | --- | --- | --- |
| pub | `laser/raw` | `laser_hhrp/RawProfile` | every profile, QoS depth 200 |
| pub | `laser/profile` | `sensor_msgs/PointCloud2` | every `laser.live_view_every`-th profile, sensor-data QoS |
| pub | `laser/status` | `laser_hhrp/LaserStatus` | 1 Hz: connected + last message |
| param | `laser.interface` | `""` | scanner IP / SDK name; empty = first found |
| param | `laser.resolution` | `1280` | highest supported value ≤ this is used |
| param | `laser.exposure_time_us` | `100` | register in units of 10 µs |
| param | `sync.hardware_trigger` | `false` | true = external trigger on In1 |
| param | `sync.software_rate_hz` | `120.0` | only declared when `sync.hardware_trigger` is false |
| param | `laser.live_view_every` | `10` | live-view thinning |

## Processing

```
startup:
    read parameters into ScanControlSettings
    if software sync:
        expected_hz = max(1, software_rate_hz)
        idle_time_us = max(10, 1e6/expected_hz − exposure_time_us)     (period = exposure + idle)
    connect()
    timers: 1 ms -> poll(), 1 s -> publish_status()

connect():
    scanner_clock.reset()                      (a reconnected scanner may restart its clock)
    ScanControl::connect(settings):
        find interface (or first discovered) -> SetDeviceInterface -> Connect -> check type is 29xx
        resolution = highest supported <= requested
        read-modify-write registers (only our bit fields change):
            exposure (value/10, automatic off), idle (value/10),
            IO: TTL 5 V (or 24 V), pull-down, In1 = trigger input
            trigger: external -> ext-active | edge | digital input | rising;  internal -> clear ext-active
        partial profile: x and z of stripe 0 (8 bytes / point)
        register profile + control-lost callbacks, start transfer
    on failure: keep the message, warn (throttled 10 s)

SDK thread, on_profile(data):           stamped with arrival = system time first
    drop if size != expected (counter gap shows the loss)
    if queue full (256): drop oldest, lost++
    push {bytes, arrival}

poll():                                  every 1 ms
    if not connected: retry connect() every 2 s, return
    while ScanControl::read(profile):
        convert x, z (mm -> m), valid = (z != 0); last 16 bytes = timestamp
            -> shutter_open, shutter_close, profile_counter; counter gaps -> lost
        stamp = scanner_clock.map(shutter_open, shutter_close, arrival)
        publish RawProfile; every Nth also as PointCloud2 (x, 0, z; invalid dropped)
        count for the rate report
    if read reported an error (control lost / conversion): log, disconnect   (retried after 2 s)
    report_rate()
```

### Formulas

- Exposure midpoint in scanner time: `mid = (open + close)/2` (or `open` if close wrapped);
  wraps of the 128 s clock are counted: `scanner_ns = mid + wraps·128 s`.
- Scanner → Pi (`ScannerClock::map`): `offset_i = arrival_i − scanner_ns_i`; keep the offsets of the
  last 2 s; `stamp = scanner_ns + min(offset)`. A jump back > 2 s (scanner restart) or an offset
  2 s below the minimum resets the window.
- Rate report every 5 s: `received/s = n/T`; `made/s = (counter_span + 1)/T` with
  `counter_span = (last − first) & 0xFFFFFF`; error if software sync and `made < 0.8·expected_hz`
  (a 2900 asked for more than ~140 Hz falls to ~18/s).

## Edge cases

- Built without the SDK: `connect()` always fails with "built without the scanCONTROL SDK"; the
  node keeps running and publishes `connected: false`.
- Scanner unplugged / control lost: disconnect, retry every 2 s.
- Hardware sync with no pulses: "no profiles in 5 s" warning.
- scan_controller_node treats `laser/status` older than 3 s or `connected: false` as a fatal fault.

## Where in the code

`LaserDriverNode::connect/poll/count_rate/report_rate/publish_status`;
`ScanControl::connect/read/disconnect`, `Impl::on_profile/on_control_lost`, `merge()`;
`ScannerClock::map/min_offset`; `to_cloud()`.
