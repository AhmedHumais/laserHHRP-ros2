# Motion of the device (acquisition): for monitoring and the UI.
std_msgs/Header header
bool has_encoder
int32 encoder_count
float32 speed_mps        # roller speed (signed)
float32 tilt_rad         # tilt from the rest pose of the gyro calibration
float32 rotation_radps   # |gyro| (vibration indicator)
