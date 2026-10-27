# Operator panel (RViz plugin)

Layer 7 (user interface). A panel inside RViz 2 with the scan state, calibration state, live
numbers, the operator message and large touch buttons. It contains no logic of its own: it calls
the nodes' `std_srvs/Trigger` services and shows `scan/status`. Because it only uses ROS 2, it
can run on any machine on the same network and `ROS_DOMAIN_ID` (e.g. a PC for a headless Pi).

Source: `src/gui/operator_panel.cpp`, registered by `plugins.xml` (class
`laser_hhrp/OperatorPanel`, library `laser_hhrp_gui`), layout `config/hhrp.rviz`, started by
`ros2 launch laser_hhrp ui.launch.py`.

## Interfaces

| Kind | Name | Type |
| --- | --- | --- |
| sub | `/scan/status` | `laser_hhrp/ScanStatus` |
| client | `/scan/start`, `/scan/pause`, `/scan/stop`, `/scan/zero` | `std_srvs/Trigger` |
| client | `/calibration/start`, `/calibration/reset`, `/calibration/extrinsics` | `std_srvs/Trigger` |

## Layout

```
 State (large)            Idle / Scanning / Paused / Error / Unknown
 Calibration              Calibrated [ (nominal extrinsics) ] / Calibrating: follow… / Calibration required
 Details                  frames N | speed v / vmax mm/s | tilt d deg | position p mm | faults n
 Message                  red + bold when state is Error or any fault is active
 [Start scan] [Pause / resume] [Stop scan]
 [Zero position] [Calibrate gyro] [Reset calibration]
 [        Calibrate extrinsics          ]
 Reply                    the last service answer (green ok / red refused)
```

## Processing

```
constructor:  build labels and buttons (min height 56 px), all buttons disabled

onInitialize():
    node = RViz's ROS node
    one Trigger client per button
    subscribe /scan/status → copy the message to the Qt thread (QueuedConnection) → show_status
    timer 1 s → check_link

button clicked → call(service):
    if the service is not ready: reply "<service>: not reachable"
    else reply "<service> ...", async request; the answer arrives on the ROS thread → signal → show_reply(message, success)

show_status(s):
    remember the time
    state text, calibration text (calibration 2 = CALIBRATED, 1 = CALIBRATING, else required;
        "(nominal extrinsics)" if calibrated without extrinsics)
    details: speed and max ×1000 (mm/s), tilt ×180/π (deg), position ×1000 (mm)
    message = s.message, red when state == ERROR or active_faults > 0
    active = SCANNING or PAUSED
    enable:  Start          = not active and CALIBRATED
             Stop, Pause    = active
             Zero, Reset    = always
             Calibrate gyro, Calibrate extrinsics = not active and not CALIBRATING

check_link() every 1 s:
    no status for 2 s → "Waiting for the laser roller ...", all buttons disabled
```

The panel never decides whether an action is allowed: the services re-check and refuse with a
message (shown in the reply line). The enabled buttons are only a convenience.

## Where in the code

`OperatorPanel::OperatorPanel`, `add_button`, `onInitialize`, `call`, `show_status`, `show_reply`,
`check_link`.
