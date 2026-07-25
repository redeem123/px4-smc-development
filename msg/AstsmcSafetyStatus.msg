uint32 RUNTIME_FAULT_STATE_RECOVERY = 1 # stale transformed state reversed large-error reaching action
uint32 RUNTIME_FAULT_TOTAL_BOUND = 2    # exact total-torque bound invariant failed
uint32 RUNTIME_FAULT_CONFIGURATION = 4  # active type-3 configuration became invalid

uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # gyro sample timestamp used by the controller

bool runtime_fault_latched              # sticky until reboot after any hard type-3 runtime intervention
uint32 runtime_fault_reason             # bitmask of RUNTIME_FAULT_* reasons
bool[3] state_recovery_active           # retained state is currently being removed to restore corrective reaching action
uint32[3] state_recovery_count          # cumulative stale-state recovery episodes per axis
float32[3] recovery_trigger_sliding     # sliding value at the last recovery episode entry (rad/s)
float32[3] recovery_trigger_state       # transformed state before the last recovery episode entry (rad/s^2)
float32[3] recovery_trigger_reaching    # retained-state reaching input before the last recovery episode entry (rad/s^2)
bool[2] selective_release_active
bool[2] quiet_anchor_active
bool[2] deep_quiet_active
bool[2] trim_valid
uint32[2] selective_release_count
uint32[2] quiet_anchor_count
float32[2] selective_trigger_sliding
float32[2] selective_trigger_state
float32[2] selective_trigger_trim
float32[2] selective_trigger_reaching
float32[2] trim_state
float32[2] trim_confidence
bool ground_containment_active          # armed type-3 ground-contact plus low-throttle containment is active
uint32 ground_containment_count         # cumulative containment episodes
