# recorder_node

Layer 6 (storage). Writes one folder per scan: every synchronized frame, the inspection results
and a session summary with all parameters. It also reports the free disk space, which the scan
controller monitors.

Source: `src/recorder_node.cpp`. Reader: `scripts/read_session.py`.

## Interfaces

| Kind | Name | Type | Notes |
| --- | --- | --- | --- |
| sub | `scan/status` | `laser_hhrp/ScanStatus` | opens / closes the session |
| sub | `scan/frame` | `laser_hhrp/ScanFrame` | queue 200 |
| sub | `inspection/result` | `laser_hhrp/InspectionResult` | |
| sub | `scan/sync` | `laser_hhrp/SyncStats` | summary in session.json |
| pub | `scan/storage` | `laser_hhrp/StorageStatus` | every 0.5 s |

| Parameter | Default | Use |
| --- | --- | --- |
| `system.storage_path` | `~/laser_hhrp/scans` | `~` expanded to `$HOME` |

The node is created with `automatically_declare_parameters_from_overrides(true)`: it accepts every
parameter of `config/hhrp.yaml`, so `session.json` can list them all.

## Processing

```
scan/status:
    on = state ∈ {SCANNING, PAUSED}
    if on and not scanning_:  begin()
    if not on and scanning_:  end()
    scanning_ = on                       # pause keeps the same session

begin():
    end()  (a session still open is closed)
    dir = storage_path / localtime("%Y%m%d-%H%M%S")
    create dir; open frames.bin (truncate); clear results; frames = 0
    write_failed = (file did not open)

scan/frame → write(f):      (ignored when no session is open)
    bytes = rclcpp serialization of f (CDR, as in ROS 2 bags)
    append  [uint32 length, little endian][bytes]
    if the stream failed: write_failed = true
    frames += 1

inspection/result → add_result(r):   (only while a session is open)
    keep one JSON line: feature_type, position_m[3], flushness_m, countersink_depth_m,
    surface_deviation_m, passed, detail

end():  (only if frames.bin is open)
    close frames.bin
    results.json = [ all kept result lines ]
    session.json = {
        "start": ISO UTC, "stop": ISO UTC, "frames": n,
        "summary": { missing_profiles, missing_triggers, timing_mismatches,
                     length_m = last_position_m },          # from the last scan/sync
        "parameters": { every parameter: value }            # strings quoted, use_sim_time skipped
    }

timer 0.5 s → publish_status():
    create storage_path if needed
    free_mb = f_bavail · f_frsize / 1024²     (statvfs; −1 if it fails)
    publish recording, session_dir (current or last), free_mb, write_failed
```

The destructor calls `end()`, so stopping the launch (Ctrl-C) during a scan still closes the files.

## frames.bin

```
repeat until end of file:
    uint32  length            (little endian)
    byte[length]  serialized laser_hhrp/msg/ScanFrame
```

```python
from rclpy.serialization import deserialize_message
from laser_hhrp.msg import ScanFrame
# read 4 bytes → length, read length bytes → deserialize_message(data, ScanFrame)
```

## Edge cases

- Frames that arrive before `scan/status` says SCANNING are dropped (no session open yet). The
  synchronizer only makes frames after the same status, so normally none are lost.
- `write_failed` stays true for the rest of the session; the scan controller raises a FATAL fault
  and stops the scan.

## Where in the code

`RecorderNode::begin`, `end`, `write`, `add_result`, `parameters_json`, `publish_status`,
`json_escape`, `iso_time`.
