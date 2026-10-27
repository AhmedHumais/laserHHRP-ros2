# IMU sample (m/s^2, rad/s) with its Nano time. imu/data (sensor_msgs/Imu) carries the same
# samples in Pi time for RViz and other tools.
std_msgs/Header header   # stamp: Pi time
int64 device_time_ns     # Nano clock
float32[3] accel_mps2
float32[3] gyro_radps
