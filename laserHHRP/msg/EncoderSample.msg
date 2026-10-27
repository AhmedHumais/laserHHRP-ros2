# Encoder sample from the Nano (240 Hz, also when standing still).
std_msgs/Header header   # stamp: Pi time (Nano time mapped by the clock sync)
int64 device_time_ns     # Nano clock
int32 count              # multi-turn count since the last zero
uint16 raw               # single-turn position
uint8 flags              # bit 0: read error, bit 1: jump of half a turn (count unreliable)
