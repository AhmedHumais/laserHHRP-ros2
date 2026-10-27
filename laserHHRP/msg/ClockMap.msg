# Nano clock -> Pi clock: pi_ns = host_ns + (device_ns_of_sample - device_ns) * (1 + drift).
std_msgs/Header header
int64 device_ns
int64 host_ns
float64 drift
