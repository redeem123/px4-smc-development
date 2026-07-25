
uint8 ALLOCATION_POLICY_CONFIGURED = 0
uint8 ALLOCATION_POLICY_ROLL_PITCH_HEADROOM = 1

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)
uint8 allocation_policy # per-command control-allocation policy

# TOPICS vehicle_torque_setpoint
# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc
