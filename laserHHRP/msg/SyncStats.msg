# Synchronization statistics of the current scan.
std_msgs/Header header
bool hardware_trigger
bool locked              # hardware: numbering locked; software: placing profiles
uint32 frames
uint32 missing_profiles  # hardware: triggers without a profile
uint32 missing_triggers  # hardware: profiles without a trigger record
uint32 timing_mismatches
uint32 unplaced          # software: profiles before the first encoder sample
float32 last_position_m
