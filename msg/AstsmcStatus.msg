uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # gyro sample timestamp used by the controller

bool configuration_valid                # true when the complete type-3 parameter card is valid and acknowledged
bool dt_valid                           # true when the reference and implicit state advanced on this cycle
bool total_bound_valid                  # true when the exact total torque stayed within the configured total bound
float32 raw_dt                          # controller input interval before integration clamping (s)
float32 accepted_dt                     # interval used for this accepted update, NaN on a held update (s)
float32 dt_min                          # configured minimum accepted update interval (s)
float32 dt_max                          # configured maximum accepted update interval (s)
float32 max_airborne_dt                 # maximum raw airborne interval since the last full reset (s)
uint32 valid_update_count               # cumulative accepted airborne update count
uint32 invalid_dt_hold_count            # cumulative airborne hold count caused by invalid timing or inputs
uint32 consecutive_invalid_dt_hold_count # cumulative invalid holds that immediately followed another invalid hold
uint32 reference_reset_count            # cumulative realizable-reference initialization count
uint32[3] nominal_limit_count           # cumulative nominal-torque limit count per axis
uint32[3] residual_limit_count          # cumulative residual-controller limit count per axis
uint32 total_bound_violation_count      # cumulative total-bound invariant violation count

float32[3] rate_setpoint_raw            # unshaped angular-rate target (rad/s)
float32[3] rate_setpoint_shaped         # realizable angular-rate target (rad/s)
float32[3] reference_acceleration       # realizable reference acceleration (rad/s^2)
float32[3] reference_jerk               # realizable reference jerk (rad/s^3)
float32[3] sliding_variable_raw         # raw rate setpoint minus measured rate (rad/s)
float32[3] sliding_variable             # conditioned tracking target minus measured rate (rad/s)
float32[3] integral_state               # transformed super-twisting state q (rad/s^2)
float32[3] k1                           # configured first super-twisting gain
float32[3] k2                           # fixed second super-twisting gain
float32[3] reaching_input_raw           # residual reaching command before conditioning (rad/s^2)
float32[3] reaching_input_limited       # residual reaching command after conditioning (rad/s^2)
float32[3] nominal_torque_raw           # normalized model torque before nominal limiting
float32[3] nominal_torque_limited       # normalized model torque after nominal limiting
float32[3] residual_torque_raw          # normalized residual torque before conditioning
float32[3] residual_torque_limited      # normalized residual torque after conditioning
float32[3] residual_authority           # symmetric normalized torque reserved for the residual controller
float32[3] actuator_time_constant       # first-order normalized torque actuator time constant (s)
float32[3] torque_slew_rate             # internal normalized torque-command slew bound (1/s)
float32[3] variation_weight             # proximal applied-torque variation penalty
float32[3] variation_regularization     # regularized minus desired applied torque
float32[3] actuator_state               # predicted normalized applied torque
float32[3] applied_torque_target        # conditioned achievable next-step applied torque
float32[3] command_lower                # current lower normalized torque-command bound
float32[3] command_upper                # current upper normalized torque-command bound
bool[3] actuator_constraint             # desired applied torque was projected into the achievable interval
bool[3] nominal_saturation              # per-axis nominal torque was limited
bool[3] internal_saturation             # per-axis residual reaching command was limited
float32[3] torque_raw                   # nominal raw plus residual raw normalized torque
float32[3] torque_limited               # exact total normalized torque published to allocation

bool allocator_feedback_valid           # true after at least one allocator status update was received
bool allocator_feedback_stale           # true when allocator feedback is older than the fixed 20 ms diagnostic limit
bool allocator_torque_setpoint_achieved # allocator-model feasibility flag, not a physical torque measurement
uint64 allocator_timestamp_sample       # torque-setpoint sample timestamp associated with the allocator result
float32 allocator_feedback_age_s        # age of the latest allocator feedback at publication time
float32[3] allocated_torque              # allocator-model feasible virtual torque
float32[3] allocation_residual           # desired minus allocated virtual torque
