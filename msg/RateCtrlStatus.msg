uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ

# Active rate controller: 0 = PID, 1 = MPC, 2 = model-based SMC, 3 = implicit super-twisting SMC
uint8 controller_type

# True when the active model-based SMC parameter card is valid and acknowledged
bool model_valid

# True when the implicit super-twisting parameter card is valid and acknowledged
bool astsmc_valid

# Model-based SMC diagnostics. Torques are normalized actuator commands.
float32[3] smc_surface
float32[3] smc_surface_filtered
float32[3] smc_torque_raw
float32[3] smc_torque_limited
