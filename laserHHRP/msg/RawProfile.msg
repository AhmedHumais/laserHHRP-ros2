# One laser profile as received from the scanner (before synchronization).
std_msgs/Header header   # stamp: Pi time of the exposure (from the scanner's own clock)
uint32 profile_counter   # scanner profile counter (24 bit, wraps)
float64 shutter_open_s   # scanner clock at shutter open (wraps every 128 s)
float32[] x_m            # along the laser line
float32[] z_m            # distance
bool[] valid
