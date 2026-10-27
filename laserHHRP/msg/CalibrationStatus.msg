# Device calibration (published latched, and 2 Hz).
uint8 CALIBRATION_REQUIRED=0
uint8 CALIBRATING=1
uint8 CALIBRATED=2
std_msgs/Header header
uint8 state
string message
float64[3] gyro_bias_radps
float64[3] accel_bias_mps2
float64[3] accel_scale
float64[3] gravity_imu_mps2  # gravity in the IMU frame at rest (tilt reference)
bool extrinsics_calibrated
float64 metres_per_count     # measured by the extrinsic calibration (0 = use the parameters)
int8 encoder_direction_sign
