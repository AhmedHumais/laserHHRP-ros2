# calibration_node

Owns the device's IMU calibration. It loads the calibration file at start, runs the **gyro
calibration** (hold still ~2 s → gyro bias) and the **extrinsic calibration** (a guided rolling
procedure → IMU ↔ base rotation, metres per encoder count, encoder direction), saves the result to
the YAML file and publishes the current state on `calibration/status`. Every other node takes the
calibration from that topic (acquisition: bias; synchronizer / scan controller: metres per count;
odometry: reloads the file).

Source: `src/calibration_node.cpp`, logic in `include/laser_hhrp/gyro_calibrator.hpp`,
`extrinsic_calibrator.hpp`, `calibration_data.hpp`.

## Interfaces

| Kind | Name | Type | Notes |
| --- | --- | --- | --- |
| sub | `nano/encoder` | `laser_hhrp/EncoderSample` | roller must stand still (gyro) / roll detection (extrinsics) |
| sub | `nano/imu` | `laser_hhrp/ImuSample` | **raw** samples (bias not removed) |
| pub | `calibration/status` | `laser_hhrp/CalibrationStatus` | latched (transient local), on every change and every 0.5 s |
| srv | `calibration/start` | `std_srvs/Trigger` | gyro calibration |
| srv | `calibration/reset` | `std_srvs/Trigger` | delete the calibration |
| srv | `calibration/extrinsics` | `std_srvs/Trigger` | rolling procedure (needs the gyro calibration) |

| Parameter | Default | Use |
| --- | --- | --- |
| `calibration.file` | `~/laser_hhrp/calibration.yaml` | `~` expanded to `$HOME` |
| `calibration.duration_s` | 2.0 | stillness needed for the gyro bias |
| `calibration.max_gyro_std_radps` | 0.02 | larger spread = moving |
| `encoder.roller_diameter_m` | 0.030 | only for the roll reports in the log (nominal scale) |
| `encoder.counts_per_turn` | 4096 | roll reports, effective diameter |

## State

```
state ∈ { CALIBRATION_REQUIRED, CALIBRATING, CALIBRATED }
extrinsics_running   true while the rolling procedure runs (state is CALIBRATING then)
data_                CalibrationData: gyro bias/noise, accel bias/scale, rest gravity, Extrinsics
```

```
CALIBRATION_REQUIRED ──start──► CALIBRATING ──done──► CALIBRATED
        ▲                          │ failed (no previous calibration)
        └──────────────────────────┘
CALIBRATED ──start──► CALIBRATING ──done / failed──► CALIBRATED   (old values kept on failure)
CALIBRATED ──extrinsics──► CALIBRATING (extrinsics_running) ──done / failed──► CALIBRATED
any ──reset──► CALIBRATION_REQUIRED   (file deleted)
```

## Start-up — `load()`

```
read calibration.file
if it loads and gyro_calibrated:
    data_ = file; state = CALIBRATED; message = "calibration loaded (<date>)"
else:
    state = CALIBRATION_REQUIRED; message = "calibration required: ..."
publish()
```

## Callbacks

### `nano/encoder`
```
if extrinsics_running:          extrinsic.encoder(count)
elif state == CALIBRATING:      gyro.encoder(count, now)
```

### `nano/imu` — `on_imu()`
```
g = gyro_radps, a = accel_mps2   (raw)
if extrinsics_running:  extrinsic.imu(g, a, device_time_ns)   # Nano time, same clock as the encoder
                        follow_extrinsics()
elif state == CALIBRATING:
                        gyro.imu(g, a, now)                   # Pi time is fine: only durations matter
                        follow_gyro()
```

### Timer 0.5 s
`publish()`: state, message, gyro bias, accel bias/scale, rest gravity, `extrinsics_calibrated`,
`metres_per_count` (0 if extrinsics not calibrated), `encoder_direction_sign`.

## Gyro calibration — `GyroCalibrator`

```
start(now): collecting, clear sums
encoder(count):  if |count − first_count| > 2  → "roller moved", restart sums
imu(gyro, accel, t):
    if t − start > 20 s                      → failed "device never still long enough"
    if | |accel| − 9.80665 | > 0.8 m/s²      → "device accelerating", restart sums
    n += 1;  Σg += gyro;  Σg² += gyro²;  Σa += accel
    wait until collected time ≥ duration_s (2 s) and n ≥ 20
    mean_i = Σg_i / n
    std_i  = sqrt( max(0, Σg²_i / n − mean_i²) )
    if any std_i > max_gyro_std_radps (0.02) or |mean_i| > 0.2 rad/s
                                             → "moving or vibrating", restart sums
    bias = mean, noise = std, gravity_imu = Σa / n   → done
```

`follow_gyro()`:
```
done:   data_.gyro_* = result, date = now (ISO), save file → state CALIBRATED
        (existing extrinsics are kept: the IMU mounting did not change)
failed: state = CALIBRATED if an earlier gyro calibration exists, else CALIBRATION_REQUIRED
else:   message = "calibrating: <instruction>"
```

## Extrinsic calibration — `ExtrinsicCalibrator`

Frames: **base** = the device on the surface, x forward (rolling direction), z up, y = z × x.

Prompts (the status message): `1/4` hold still · `2/4` roll forward 10 cm, hold still · `3/4`
roll backward 20 cm, hold still · `4/4` roll forward 30 cm, hold still.

```
per IMU sample (gyro minus the gyro bias, dt from Nano time; gaps > 0.1 s skipped):
    q = q ⊗ rotvec(gyro·dt)                     # IMU orientation in the reference frame
    f = R(q) · accel                            # specific force in the reference frame
    imu_still = |gyro| < 0.05 rad/s  and  | |accel| − g_rest | < 0.3 m/s²
    still     = imu_still and |count − rest_count| ≤ 2

step 1 (reference): still for 2 s →
    gravity_raw = mean(accel)  (IMU frame);  g_rest = |gravity_raw|
    fail if |g_rest − 9.80665| > 1.5
    reference frame := the IMU now (q = identity)

steps 2..4 (rolls):
    at rest ≥ 0.3 s → "armed": gravity_ref = mean(f) of this rest, v = p = 0
    roll starts when |count − count_at_rest| > 20
    while rolling:  a = f − gravity_ref
                    p += v·dt + ½·a·dt²;   v += a·dt
                    store (t, v, count)
    roll ends after 1 s of rest (imu_still and count unchanged ±2)
    T = roll duration
    displacement = p_end − v_end·T/2            # zero velocity at both rests: remove linear drift
    counts_i     = count_end − count_start      (fail if < 40: "encoder hardly moved")
    correlation:  for each sample k (encoder speed by central difference over ±25 ms)
        v_enc = (c[k+w] − c[k−w]) / (t[k+w] − t[k−w])
        v_imu = v[k] − (v_end/T)·t[k]           # drift-free IMU velocity
        corr_i += v_imu · v_enc · h ;  enc2_i += v_enc² · h       (h = sample spacing)
```

`solve()` after the third roll:
```
k  = Σ counts_i·d_i / Σ counts_i²               # metres per count, least squares through 0
                                                # d = (+0.10, −0.20, +0.30) m
z  = normalize(gravity_raw)                     # base z (up) in IMU coordinates
h  = Σcorr − z·(z·Σcorr)                         # horizontal part of the correlation
fail if Σenc2 ≤ 0 or |h|/Σenc2 < 1e-9           "the IMU saw no horizontal motion"
x  = sign(k) · normalize(h);   y = z × x
checks per roll i:
    |k·counts_i − d_i| ≤ 15 mm                  else "roll i: … cm instead of … cm"
    (corr_i · x)·k > 0                          else "roll i: the IMU moved the other way"
ratio = Σ|corr_i·x| / Σ|k|·enc2_i               # IMU speed / encoder speed
    0.5 ≤ ratio ≤ 2                             else "roller turned without moving, or slipping"
result:
    base_from_imu        = quaternion of the matrix with rows (x, y, z)
    metres_per_count     = |k|
    encoder_direction_sign = sign(k)
    residual_m           = max_i |k·counts_i − d_i|
    imu_distance_ratio   = ratio
    accel_bias           = z · (g_rest − 9.80665)   # bias along gravity only
any step longer than 60 s → failed "step timed out"
```

`follow_extrinsics()`:
```
done:   copy result into data_.extrinsics,
        roller_effective_diameter_m = metres_per_count · counts_per_turn / π
        data_.accel_bias = accel_bias, date = now, save → state CALIBRATED, publish
failed: log every roll report, state = CALIBRATED (gyro stays valid), message = "... failed: <why>"
else:   log each finished roll ("roll n (asked +10 cm): … counts = … cm, IMU … cm, … s");
        when the prompt changes: message = prompt, publish
```

## Services

| Service | Refused when | Action |
| --- | --- | --- |
| `calibration/start` | already CALIBRATING or extrinsics running | `gyro.start(now)`, state CALIBRATING |
| `calibration/extrinsics` | no gyro calibration, or anything running | `extrinsic.start(gyro_bias)`, state CALIBRATING |
| `calibration/reset` | never | cancel both, delete the file, `data_ = defaults`, CALIBRATION_REQUIRED (the scan controller stops a running scan) |

## The file

Written by `calibration_file::save()` (flat YAML, `version: 1`): device, date, `gyro_calibrated`,
`gyro_bias_radps`, `gyro_noise_radps`, `accel_bias_mps2`, `accel_scale`, `gravity_imu_mps2`,
`extrinsics_calibrated`, `base_from_imu_wxyz`, `metres_per_count`, `encoder_direction_sign`,
`gravity_mps2`, `residual_m`, `imu_distance_ratio`, laser extrinsics (still nominal),
`roller_effective_diameter_m`. The parent folder is created if needed; a failed write is
reported in the message ("could not write …") but the values are still used.

## Where in the code

`CalibrationNode::load`, `on_imu`, `follow_gyro`, `follow_extrinsics`, `log_rolls`, `save`,
`publish`, `on_start`, `on_reset`, `on_extrinsics` · `GyroCalibrator::imu/encoder` ·
`ExtrinsicCalibrator::imu`, `finish_reference`, `finish_roll`, `correlate`, `solve` ·
`calibration_file::load/save`.
