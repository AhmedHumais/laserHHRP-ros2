# acquisition_node

Layer 2 (data acquisition). Applies the IMU calibration to every raw IMU sample, and computes the
device's motion state (roller speed, tilt, vibration) for monitoring and the operator panel.

Source: `src/acquisition_node.cpp`, `include/laser_hhrp/encoder_scale.hpp`,
`include/laser_hhrp/rotation.hpp`.

## Interfaces

| Kind | Name | Type / default | Notes |
| --- | --- | --- | --- |
| sub | `nano/encoder` | `laser_hhrp/EncoderSample` | |
| sub | `nano/imu` | `laser_hhrp/ImuSample` | raw |
| sub | `calibration/status` | `laser_hhrp/CalibrationStatus` | latched |
| pub | `imu/corrected` | `laser_hhrp/ImuSample` | calibrated, Nano time kept |
| pub | `scan/motion` | `laser_hhrp/Motion` | after every encoder and IMU sample |
| param | `encoder.roller_diameter_m`, `encoder.counts_per_turn`, `encoder.direction_sign` | `0.030`, `4096`, `1` | via EncoderScale |

## Processing

```
on calibration/status:
    keep it (bias, scale, gravity reference, state); pass it to EncoderScale

on nano/encoder e:
    if a previous sample exists:
        dt = (e.device_time − prev.device_time)·1e-9
        if dt > 0: speed = (e.count − prev.count) · metres_per_count / dt
    motion.encoder_count = e.count; publish motion

on nano/imu m:
    gyro_i  = gyro_i − gyro_bias_i
    accel_i = (accel_i − accel_bias_i) · accel_scale_i
    publish imu/corrected
    g_ref = (0, 0, 9.80665) if CALIBRATION_REQUIRED else calibration.gravity_imu_mps2
    tilt  = acos( clamp( accel·g_ref / (|accel|·|g_ref|), −1, 1 ) )
    rotation = |gyro|                                    (vibration indicator)
    publish motion
```

### Formulas

- `metres_per_count` (EncoderScale): the calibrated value if
  `extrinsics_calibrated && metres_per_count > 0`, else `π · roller_diameter_m / counts_per_turn`.
- Speed is signed in the encoder's counting direction (not multiplied by `direction_sign`).
- Tilt is relative to the rest pose measured during the gyro calibration, so any IMU mounting works.

## Edge cases

- Before any calibration message: bias 0, scale 1, gravity reference = IMU z axis.
- Speed stays 0 until two encoder samples arrived; `dt ≤ 0` keeps the old speed.

## Where in the code

`AcquisitionNode::on_encoder/on_imu/publish_motion`; `EncoderScale::metres_per_count`;
`angle_between`, `norm` (rotation.hpp).
