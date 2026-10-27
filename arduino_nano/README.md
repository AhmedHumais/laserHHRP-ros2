# arduino_nano — firmware for the laser roller's Arduino Nano

The Nano sits between the roller's sensors and the Raspberry Pi 5:

- it counts the roller's travel with the **AMT222C-V encoder** (SPI),
- it fires the **laser profiler's trigger input** every few encoder counts,
- it reads the **MPU6050 IMU** (I2C),
- it sends everything to the Pi over **USB serial**, stamped with its own clock.

The Pi side (ROS 2 package `laserHHRP`, node `nano_driver`) talks to it. Nothing else may use the
serial port while the nodes run.

## Files

| File | Contents |
| --- | --- |
| `arduino_nano.ino` | Settings (pins, rates), `setup()` and `loop()`: reading the sensors, firing the trigger, sending and receiving packets |
| `encoder_amt22.h` | Reading the AMT22 over SPI (with its check bits) and turning the single-turn position into a multi-turn count |
| `imu_mpu6050.h` | Setting up the MPU6050 and reading raw accelerometer and gyro values |
| `laser_trigger.h` | Deciding when to fire the next laser pulse (every N counts, not faster than the laser allows) |
| `protocol.h` | Packet layout and checksum shared with the Pi (copy on the Pi: `laserHHRP/include/laser_hhrp/serial_protocol.hpp`) |
| `platformio.ini` | Optional PlatformIO build (the Arduino IDE needs nothing extra) |

## Wiring

| Nano pin | Connects to |
| --- | --- |
| D2 | AMT22 /CS |
| D11 / D12 / D13 | AMT22 MOSI / MISO / SCLK |
| 5V, GND | AMT22 +5 V, GND |
| A4 / A5 | MPU6050 SDA / SCL (GY-521 board; AD0 → GND for address 0x68) |
| 5V (or 3V3), GND | MPU6050 VCC, GND |
| D3 | Laser multi-function socket **In1** (pin 6, yellow) — only for hardware sync |
| GND | Laser **GND-In1** (pin 4, green) |
| USB | Raspberry Pi (power and data) |

## Build and upload

**Arduino IDE:** open `arduino_nano/arduino_nano.ino`, choose board *Arduino Nano*, processor
*ATmega328P* (or *ATmega328P (Old Bootloader)* for many clones), the port, then *Upload*.

**PlatformIO:** in this folder, `pio run` builds and `pio run -t upload` flashes.

Stop the ROS 2 nodes before uploading: they keep the serial port open.

## Main loop

`loop()` runs continuously (no delays, a few kHz) and does, in order:

1. **Receive** — commands and clock requests from the Pi (`receive_packets`).
2. **Encoder** — read the position, update the multi-turn count; if the roller has moved one
   *pitch* (`counts_per_trigger` counts) since the last pulse, fire the trigger (`poll_encoder`).
   An encoder sample goes to the Pi every 4167 µs (240 Hz), also when standing still.
3. **IMU** — every 4167 µs read one sample and send it (`poll_imu`). A sample identical to the
   previous one (read twice because the two clocks drift) is skipped.
4. **Status** — twice per second: encoder/IMU health and trigger counts (`report_status`).
5. **IMU retry** — once per second if the IMU stopped answering (`retry_imu`).

A **watchdog** restarts the Nano if the loop ever hangs for 0.5 s; the Pi reconnects by itself.

## Laser trigger

On `START` from the Pi the trigger counter restarts at 0. Each time the roller has moved
`counts_per_trigger` counts (either direction) since the last pulse, D3 goes high for 20 µs; the
laser starts one exposure on the rising edge. For every pulse a *trigger record* (number, encoder
count, direction, time of the edge) goes to the Pi, which pairs it with the profile that follows.

The pulse is never faster than `min_trigger_period_us` (the laser's maximum profile rate, sent by
the Pi). If the roller moves too fast, the pulse is delayed or a pitch is skipped and the record is
flagged *overspeed*.

**Hardware sync vs software sync:** the firmware behaves the same in both modes. With software sync
D3 is simply not wired; the Pi then places the free-running laser's profiles using the encoder
samples.

## Time stamps

Every packet carries the Nano's time in µs (`micros()` extended to 64 bit so it never wraps).
Because encoder, trigger and IMU share this one clock, the Pi can take the IMU exactly at each
trigger. To convert Nano time to Pi time, the Pi sends a time request twice per second; the Nano
answers at once with its clock (`handle_time_request`).

## Serial protocol

USB serial, 1 000 000 baud, binary packets only — never use `Serial.print()`:

```
0xAA 0x55 | type | length | payload | checksum (Fletcher-16, 2 bytes)
```

| Type | Direction | Payload (`protocol.h`) |
| --- | --- | --- |
| 0x01 | Nano → Pi | `EncoderPacket`: time, count, raw position, flags |
| 0x02 | Nano → Pi | `TriggerPacket`: time of the edge, trigger number, count, direction, flags |
| 0x03 | Nano → Pi | `ImuPacket`: time, raw accel and gyro (±2 g, ±250 °/s) |
| 0x04 | Nano → Pi | `StatusPacket`: encoder ok, IMU ok, triggering, trigger and overspeed counts |
| 0x05 | Nano → Pi | `TimeReplyPacket`: answer to a time request |
| 0x10 | Pi → Nano | `CommandPacket`: STOP / START (with counts per trigger and minimum period) / ZERO |
| 0x11 | Pi → Nano | `TimeRequestPacket` |

If you change a packet, change `laserHHRP/include/laser_hhrp/serial_protocol.hpp` the same way.

## Settings

At the top of `arduino_nano.ino`:

| Constant | Default | Meaning |
| --- | --- | --- |
| `ENCODER_CS_PIN` | 2 | AMT22 chip select |
| `TRIGGER_PIN` | 3 | Output to the laser's In1 |
| `TRIGGER_PULSE_US` | 20 | Pulse width (check the laser manual for its minimum) |
| `SERIAL_BAUD` | 1000000 | Must match `nano.baud` on the Pi |
| `ENCODER_BITS` | 12 | 12 for AMT222C-V, 14 for AMT223x (then also `encoder.counts_per_turn: 16384` on the Pi) |
| `SAMPLE_PERIOD_US` | 4167 | Encoder and IMU sample period (240 Hz) |
| `IMU_DLPF`, `IMU_SAMPLE_DIVIDER` | 2, 0 | MPU6050 low-pass filter (94 Hz) and internal rate (1 kHz) |

The number of counts per trigger and the maximum trigger rate are **not** set here: the Pi sends
them with every `START` (parameters `encoder.counts_per_trigger`, `laser.max_profile_rate_hz`).

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Upload fails (`not in sync`) | Choose the *Old Bootloader* processor; stop the ROS 2 nodes |
| Pi says "Nano silent" | Baud rate; another program on the port; a `Serial.print()` added to the code |
| Encoder status not ok | SPI wiring, 5 V supply, `ENCODER_BITS` matching the encoder |
| IMU status not ok | I2C wiring (A4/A5), AD0 to GND |
| No profiles in hardware sync | D3 → In1 and GND → GND-In1 wired; the roller must move while scanning |
