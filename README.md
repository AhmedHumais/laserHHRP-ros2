# laserHHRP — laser-based roller inspection handheld

A hand-held roller carrying a Micro-Epsilon **scanCONTROL LLT2900-10/BL** laser line profiler.
While the operator rolls it over a part, the roller's **AMT222C-V** encoder gives the position, an
**MPU6050** IMU the orientation and motion, and every laser profile is placed along the scan
axis. The profiles become a 3D point cloud for inspection (e.g. rivet flushness).

| Folder | Runs on | What it is |
| --- | --- | --- |
| [`arduino_nano/`](arduino_nano/README.md) | Arduino Nano | Firmware: encoder, IMU, laser trigger, USB link to the Pi |
| [`laserHHRP/`](laserHHRP/README.md) | Raspberry Pi 5 (ROS 2) | ROS 2 package `laser_hhrp`: drivers, synchronization, calibration, recording, RViz operator panel |

The two parts only share the serial protocol (`arduino_nano/protocol.h` ↔
`laserHHRP/include/laser_hhrp/serial_protocol.hpp`).

## Quick start

```bash
# Arduino: open arduino_nano/arduino_nano.ino in the Arduino IDE and upload (or: cd arduino_nano && pio run -t upload)

# Raspberry Pi 5 (Ubuntu + ROS 2):
#   install the scanCONTROL SDK once (laserHHRP/README.md, "scanCONTROL SDK")
mkdir -p ~/ros2_ws/src && cp -r laserHHRP ~/ros2_ws/src/
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch laser_hhrp hhrp.launch.py                         # software sync
ros2 launch laser_hhrp hhrp.launch.py hardware_trigger:=true  # hardware sync (D3 -> In1 wired)
ros2 launch laser_hhrp hhrp.launch.py simulate:=true          # without hardware

# Operator UI (on the Pi's screen, or on a PC on the same network with the package built there):
ros2 launch laser_hhrp ui.launch.py
```

A Pi without a screen: run the UI on a laptop/PC connected over Ethernet. Both machines need the
same `ROS_DOMAIN_ID` (default 0) and the `laser_hhrp` package built (for its messages and the panel).

## Hardware

| Component | Connection |
| --- | --- |
| Raspberry Pi 5 | Ethernet → PoE injector (laser data), USB → Arduino Nano |
| scanCONTROL LLT2900-10/BL | PoE (power + data); trigger input In1 ← Nano D3 (hardware sync only) |
| Arduino Nano (ATmega328P) | USB to the Pi (power + data, 1 Mbaud) |
| AMT222C-V encoder (12 bit, SPI) | SPI to the Nano (D2 = /CS) |
| MPU6050 IMU | I2C to the Nano (A4/A5) |
| Power | 48 V → switch → fuse → PoE injector (laser) and 48→5 V DC-DC (Pi, Nano) |

Pin-level wiring: [arduino_nano/README.md](arduino_nano/README.md).
