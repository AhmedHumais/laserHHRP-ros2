# nano_driver_node

Layer 1 (hardware interface). Owns the USB serial port to the Arduino Nano: it decodes the binary
packets into ROS 2 messages, converts IMU counts to SI units, sends scan commands to the Nano, and
keeps a Nano-clock → Pi-clock mapping up to date through clock exchanges. Every data message
carries both the Nano time (`device_time_ns`) and the mapped Pi time (`header.stamp`). Full timing
contract: [synchronization.md](synchronization.md).

Source: `src/nano_driver_node.cpp`, `include/laser_hhrp/serial_protocol.hpp`,
`include/laser_hhrp/clock_sync.hpp`.

## Interfaces

| Kind | Name | Type | Notes |
| --- | --- | --- | --- |
| pub | `nano/encoder` | `laser_hhrp/EncoderSample` | 240 Hz, QoS depth 500 |
| pub | `nano/trigger` | `laser_hhrp/TriggerRecord` | one per laser pulse |
| pub | `nano/imu` | `laser_hhrp/ImuSample` | 240 Hz, raw (not calibrated), SI units |
| pub | `imu/data` | `sensor_msgs/Imu` | same samples, sensor-data QoS, no orientation |
| pub | `nano/status` | `laser_hhrp/NanoStatus` | 2 Hz |
| pub | `nano/clock` | `laser_hhrp/ClockMap` | latched (transient local), after each accepted exchange |
| sub | `nano/command` | `laser_hhrp/NanoCommand` | from scan_controller_node |
| param | `nano.port` | string, `/dev/ttyUSB0` | set by launch argument `nano_port` |
| param | `nano.baud` | int, `1000000` | must equal `SERIAL_BAUD` in the firmware |

## Processing

```
startup:
    declare parameters, create publishers / subscription
    start a 1 ms wall timer -> poll()

poll():                                   every 1 ms
    if port closed:
        if less than 2 s since the last attempt: return
        fd = open_serial(port, baud)      raw mode, 8N1, ASYNC_LOW_LATENCY (FTDI 1 ms timer), flush
        if failed: warn (throttled 10 s), return
        clock.reset(); last_heard = now
    read all waiting bytes (non-blocking):
        for each byte: if reader.push(byte) -> handle_packet()
    if read() returned 0 or a real error: log "Nano disconnected", close port   (reopened by the 2 s rule)
    sync_clock(now)
    if nothing heard for 0.5 s: warn "Nano silent" (throttled 5 s)

sync_clock(now):
    period = 100 ms while the mapping has < 8 points, else 500 ms
    if period elapsed: send TimeRequest{++sequence, host_send_ns = now}; clock.sent(sequence, host_send_ns)

handle_packet():                          last_heard = now
    ENCODER  -> EncoderSample  {device_time_ns = time_us*1000, stamp = pi_time(device), count, raw, flags}
    TRIGGER  -> TriggerRecord  {device_time_ns, stamp, index, count, direction, flags}
    IMU      -> ImuSample      {accel = counts * 9.80665/16384  [m/s^2]
                                gyro  = counts * (pi/180)/131   [rad/s]}
                + sensor_msgs/Imu with the same values (orientation_covariance[0] = -1)
    STATUS   -> NanoStatus     {encoder_ok, imu_ok, triggering, triggers, overspeed}
    TIME_REPLY:
        if clock.reply(seq, host_send, nano_receive, nano_send, now) accepted and clock valid:
            publish ClockMap{device_ns = nano_send, host_ns = clock.to_host(nano_send), drift}

on nano/command:  write CommandPacket{command, counts_per_trigger, imu_period_ms, min_trigger_period_us}
on shutdown:      send STOP, close the port
```

### Formulas

- Pi time of a Nano time: `pi = to_host(t) = ref_host + (t − ref_device)·(1 + drift)`; before the
  first accepted exchange `pi = arrival time`.
- Clock exchange (`ClockSync::reply`):
  - `rtt = host_receive − host_send`, `processing = nano_send − nano_receive`
  - rejected if `rtt ≤ 0`, `rtt > 50 ms`, `processing < 0`, `processing > rtt`, or
    `rtt > min(rtt of the last 16) + 2 ms`
  - point: `(nano_receive + processing/2, host_send + rtt/2)`
  - least-squares line over the last 32 points of `y = (host − h0) − (nano − d0)` against
    `x = nano − d0`: `drift = (n·Σxy − Σx·Σy)/(n·Σx² − (Σx)²)`, `ref_device = d0 + mean(x)`,
    `ref_host = h0 + mean(x) + mean(y)`
  - if `|drift| > 0.01` (or not finite): reset.
- Packet: `AA 55 | type | length | payload | Fletcher-16(type, length, payload)` (little endian).
  Corrupt packets are counted (`PacketReader::errors()`) and dropped.

## Edge cases

- Nano unplugged: `read()` returns 0 / error → port closed, retried every 2 s; the clock mapping
  restarts from zero after reopening.
- Baud other than 1000000 / 500000 falls back to 115200 (`open_serial`).
- Before the first clock reply, stamps are arrival times (a few ms late).
- A write failure is only logged.

## Where in the code

`open_serial()`, `NanoDriverNode::poll()`, `sync_clock()`, `pi_time()`, `handle_packet()`,
`send_command()`, `write_packet()`; `ClockSync::sent/reply/to_host/add`; `protocol::encode`,
`protocol::PacketReader::push/payload`.
