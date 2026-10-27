# One trigger pulse the Nano sent to the laser (input In1): one profile follows (hardware sync).
std_msgs/Header header   # stamp: Pi time of the pulse
int64 device_time_ns     # Nano clock at the rising edge
uint32 index             # trigger number since scan start (0, 1, 2, ...)
int32 count              # encoder count at the pulse
int8 direction           # +1 forward, -1 backward
uint8 flags              # bit 0: overspeed
