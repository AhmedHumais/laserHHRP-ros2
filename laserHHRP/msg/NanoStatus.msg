# Nano health (2 Hz).
std_msgs/Header header
bool encoder_ok
bool imu_ok
bool triggering          # between scan start and stop
uint32 triggers          # pulses since scan start
uint32 overspeed         # pulses flagged overspeed since scan start
