# Scan state for the operator (5 Hz).
uint8 IDLE=0
uint8 SCANNING=1
uint8 PAUSED=2
uint8 ERROR=3
std_msgs/Header header
uint8 state
uint32 frames
uint32 missing_profiles
uint32 missing_triggers
float32 speed_mps
float32 max_speed_mps
float32 tilt_rad
float32 position_m
uint32 active_faults
uint8 calibration        # CalibrationStatus.state
bool extrinsics_calibrated
string message           # most important instruction / warning
string[] faults          # all active faults, "severity source: message"
