# One synchronized scan frame: a profile with its scan-axis position and the IMU at the same
# instant. Input of the point cloud layer; recorded in frames.bin.
std_msgs/Header header   # stamp: Pi time of the exposure (trigger)
uint32 index             # frame number in the scan
int32 encoder_count
float64 position_m       # scan-axis position since scan start
int8 direction
float32[3] accel_mps2    # IMU interpolated at the exposure (calibrated)
float32[3] gyro_radps
bool imu_valid
uint8 flags              # bit 0: overspeed, bit 1: profile/trigger timing mismatch
float32[] x_m
float32[] z_m
bool[] valid
