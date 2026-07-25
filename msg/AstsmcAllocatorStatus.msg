uint16 STATE_RECEIVED = 1
uint16 STATE_USABLE = 2
uint16 STATE_TORQUE_ACHIEVED = 4
uint16 STATE_THRUST_ACHIEVED = 8
uint16 STATE_ACTUATOR_BOUND = 16
uint16 STATE_BACKOFF_ACTIVE = 32
uint16 STATE_RELEASE_ACTIVE = 64
uint16 STATE_HEADROOM_REQUESTED = 128
uint16 STATE_HEADROOM_APPLIED = 256
uint16 STATE_ROLL_CONDITIONING = 512
uint16 STATE_PITCH_CONDITIONING = 1024

uint64 timestamp
uint64 timestamp_sample
uint64 allocator_timestamp_sample
float32 publication_age_s
float32 sample_age_s
float32 allocation_residual_norm
float32[3] yaw_residual_authority # base, maximum governed, effective
float32[2] roll_pitch_torque_norm # requested, allocated
float32 roll_pitch_achieved_fraction
uint32[6] event_count # accepted, rejected, yaw backoff, stale/invalid fallback, headroom requests, roll/pitch miss episodes
uint8 requested_allocation_policy
uint8 applied_allocation_policy
uint16 rejection_flags
uint16 state_flags
