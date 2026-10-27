# Command to the Nano (from the scan controller).
uint8 STOP=0             # stop triggering (samples keep flowing)
uint8 START=1            # restart trigger numbering, trigger every counts_per_trigger counts
uint8 ZERO=2             # encoder count = 0 here
uint8 command
uint16 counts_per_trigger
uint16 imu_period_ms         # 0 = firmware default (240 Hz)
uint16 min_trigger_period_us # never trigger faster than the laser can measure
