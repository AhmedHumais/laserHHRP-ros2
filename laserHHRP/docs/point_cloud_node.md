# point_cloud_node

Layer 4 (point cloud generation). **Placeholder to be developed.** It currently builds the
simplest possible cloud: every valid profile point placed at its scan-axis position, all frames of
the current scan merged, published once per second for the inspection node and for RViz.

Source: `src/point_cloud_node.cpp`. Input contract: [synchronization.md](synchronization.md).

## Interfaces

| Kind | Name | Type | Notes |
| --- | --- | --- | --- |
| sub | `scan/frame` | `laser_hhrp/ScanFrame` | queue 200 |
| pub | `scan/cloud` | `sensor_msgs/PointCloud2` | for `inspection_node` |
| pub | `scan/cloud_view` | `sensor_msgs/PointCloud2` | same cloud, RViz "3D scan" display |

No parameters.

## Processing

```
scan/frame → add(f):
    if f.index == 0: points.clear()            # first frame of a new scan
    for each point i with f.valid[i]:
        append (x, y, z) = (f.x_m[i], f.position_m, f.z_m[i])
    changed = true

timer 1 s → publish():
    if not changed: return
    changed = false
    PointCloud2: frame "laser", height 1, width = points/3,
                 fields x, y, z float32 (offsets 0, 4, 8), point_step 12, dense
    publish on scan/cloud and scan/cloud_view
```

Frame of the cloud: x along the laser line, y = roller travel since the scan start (encoder), z =
distance from the laser (larger = farther away).

## Not done yet (planned steps from the architecture document)

- IMU tilt compensation or a tilt quality check (`accel_mps2`, `gyro_radps` of each frame, or the
  laser pose on `scan/pose`, which has the same stamp as the frame once the extrinsics exist).
- Filtering of outliers beyond `valid`; a "poor point cloud" warning.
- Memory: the cloud grows for the whole scan (about 1.8 MB/s at 120 Hz with 1278 points).

## Where in the code

`PointCloudNode::add`, `publish`.
