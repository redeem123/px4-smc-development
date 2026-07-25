/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rate_control.hpp
 *
 * PID, constrained MPC, model-based SMC, and implicit super-twisting 3-axis angular-rate control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <motion_planning/VelocitySmoothing.hpp>
#include <uORB/topics/astsmc_safety_status.h>
#include <uORB/topics/astsmc_status.h>
#include <uORB/topics/rate_ctrl_status.h>

class RateControl
{
public:
	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control PID gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setPidGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum absolute value of the PID integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
				 const matrix::Vector3<bool> &saturation_negative);

	/**
	 * Set individual saturation flags
	 * @param axis 0 roll, 1 pitch, 2 yaw
	 * @param is_saturated value to update the flag with
	 */
	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	struct AstsmcAllocatorFeedback {
		matrix::Vector3f allocated_torque{};
		matrix::Vector3f unallocated_torque{};
		bool usable{false};
		bool new_sample{false};
		bool torque_setpoint_achieved{false};
		bool thrust_setpoint_achieved{false};
		bool actuator_bound{false};
	};

	void setAstsmcAllocatorFeedback(const AstsmcAllocatorFeedback &feedback);
	void resetAstsmcAllocatorFeasibility();

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed,
				const float raw_dt = NAN, const bool ground_containment = false);

	/**
	 * Set controller type (0 = PID, 1 = MPC, 2 = model-based SMC, 3 = implicit super-twisting SMC)
	 * @return true when the requested controller was accepted
	 */
	bool setControllerType(int type);
	int getControllerType() const { return _controller_type; }

	/**
	 * Atomically set the complete model-based SMC parameter card.
	 * Invalid cards are rejected without changing the last valid configuration.
	 * @return true when the complete card was accepted
	 */
	bool setModelBasedSmcParameters(const matrix::Vector3f &inertia,
					const matrix::Vector3f &control_effectiveness,
					const matrix::Vector3f &c, const matrix::Vector3f &eta,
					const matrix::Vector3f &bnd, const matrix::Vector3f &ks,
					float rate_sp_derivative_limit,
					const matrix::Vector3f &integrator_limit,
					const matrix::Vector3f &torque_limit,
					float cutoff, float slew);
	bool modelBasedSmcParametersValid() const { return _msmc_model_valid; }

	/**
	 * Set the Bouabdallah/Siegwart (ICRA 2005) augmentation constants for the
	 * model-based SMC. Both terms are disabled by zero, which reproduces the
	 * plain rate-loop law exactly.
	 * @param attitude_surface_slope per-axis alpha1 of eq. 30; enables the -alpha1^2*z1 term of eq. 33/34
	 * @param rotor_inertia propulsion-group inertia J_R of eq. 5 in kg m^2
	 * @return true when the constants were accepted
	 */
	bool setModelBasedSmcPaperConstants(const matrix::Vector3f &attitude_surface_slope, float rotor_inertia);

	/**
	 * Provide the per-cycle vehicle state required by the paper augmentation.
	 * Invalid or unset state zeroes the corresponding term for that cycle.
	 * @param net_rotor_speed Omega of eq. 7 in rad/s, signed by propeller direction
	 */
	void setModelBasedSmcPaperState(float net_rotor_speed, bool net_rotor_speed_valid);

	/**
	 * Atomically set the complete implicit super-twisting parameter card.
	 * Gains are expressed in rate-acceleration coordinates and output limits are normalized torque.
	 * Invalid cards are rejected without changing the last valid configuration.
	 */
	bool setAstsmcParameters(const matrix::Vector3f &inertia,
				 const matrix::Vector3f &control_effectiveness,
				 const matrix::Vector3f &k1, const matrix::Vector3f &k2,
				 const matrix::Vector3f &torque_limit,
				 const matrix::Vector3f &reference_acceleration_limit,
				 const matrix::Vector3f &reference_jerk_limit,
				 const matrix::Vector3f &residual_torque_limit,
				 const matrix::Vector3f &actuator_time_constant,
				 const matrix::Vector3f &torque_slew_rate,
				 const matrix::Vector3f &variation_weight,
				 const matrix::Vector3f &sliding_boundary,
				 float tracking_blend, float reference_feedforward,
				 float reference_feedforward_rp, float gyro_compensation,
				 float dt_min, float dt_max, float recovery_error_threshold,
				 float quiet_k1_error_threshold,
				 float selective_release_error_threshold, float selective_release_state_rate,
				 float trim_command_threshold, float trim_error_threshold,
				 float trim_acceleration_threshold, float trim_confidence_time,
				 float trim_time_constant, float roll_pitch_residual_extension,
				 float roll_pitch_k1_recovery_boost, float yaw_residual_extension);
	bool astsmcParametersValid() const { return _astsmc_model_valid; }
	void latchAstsmcRuntimeFault(uint32_t reason)
	{
		_astsmc_runtime_fault_latched = true;
		_astsmc_runtime_fault_reason |= reason;
	}

	/**
	 * Set rate-level MPC parameters
	 */
	void setMpcGains(const matrix::Vector3f &inertia, const matrix::Vector3f &rate_weight,
			 const matrix::Vector3f &control_effectiveness, const matrix::Vector3f &torque_weight,
			 const matrix::Vector3f &torque_rate_weight, int horizon, float torque_slew_rate);

	/**
	 * Set rate-level MPC integral bias adaptation gains.
	 */
	void setMpcIntegralGain(const matrix::Vector3f &I);
	void setMpcIntegralLimit(const matrix::Vector3f &integrator_limit);

	/**
	 * Set rate-level MPC normalized torque constraints.
	 */
	void setMpcTorqueLimit(const matrix::Vector3f &torque_limit);

	/**
	 * Set rate-level MPC rate-setpoint derivative feed-forward limit.
	 */
	void setMpcRateSetpointDerivativeLimit(float rate_sp_derivative_limit);

	/**
	 * Set rate MPC rigid-body gyroscopic compensation blend.
	 */
	void setMpcGyroCompensation(float gyro_compensation_weight);

	/**
	 * Set the first-order normalized torque actuator time constant.
	 */
	void setMpcActuatorTimeConstant(float time_constant);

	/**
	 * Reset model-based SMC rate setpoint feed-forward history.
	 */
	void resetModelBasedSmcSetpoint()
	{
		_smc_last_rate_sp.zero();
		_smc_rate_sp_prev_valid = false;
	}

	void resetMpcSetpoint()
	{
		_mpc_last_rate_sp.zero();
		_mpc_rate_sp_prev_valid = false;
	}

	/**
	 * Reset setpoint-derived state after an active source or navigation-context change.
	 * Type 3 preserves the last bounded output and reseeds its conditioned state on the
	 * next valid sample.
	 */
	void resetSetpointHistory();

	/**
	 * Reset setpoint-derived state while rate control is disabled.
	 * Type 3 clears its output seed so stale airborne torque cannot be replayed.
	 */
	void resetSetpointHistoryForControlDisable();

	/**
	 * Reset SMC-only dynamic state.
	 */
	void resetSmcState()
	{
		_smc_rate_int.zero();
		_smc_surface.zero();
		_smc_s_filtered.zero();
		_smc_torque_raw.zero();
		_smc_torque_limited.zero();
		_smc_last_torque.zero();
		resetModelBasedSmcSetpoint();
	}

	void resetAstsmcReference(bool preserve_output)
	{
		if (preserve_output && _last_output_valid) {
			_astsmc_seed_torque = _last_output;
			_astsmc_last_torque = _last_output;
			_astsmc_torque_raw = _last_output;
			_astsmc_torque_limited = _last_output;
			_astsmc_actuator_state = _last_output;
			_astsmc_seed_pending = true;

		} else {
			_astsmc_seed_torque.zero();
			_astsmc_last_torque.zero();
			_astsmc_torque_raw.zero();
			_astsmc_torque_limited.zero();
			_astsmc_actuator_state.zero();
			_astsmc_seed_pending = false;
		}

		_astsmc_reference_valid = false;
		_astsmc_reference_reset_pending = true;
		_astsmc_trim_confidence.zero();
		_astsmc_trim_valid.zero();
		_astsmc_selective_release_active.zero();
		_astsmc_selective_release_completed.zero();
		_astsmc_quiet_anchor_active.zero();
		_astsmc_deep_quiet_active.zero();
	}

	void resetAstsmcState(bool reset_counters = false, bool count_reference_reset = true)
	{
		_astsmc_surface.zero();
		_astsmc_surface_raw.zero();
		_astsmc_tracking_setpoint.zero();
		_astsmc_rate_sp_raw.zero();
		_astsmc_rate_sp_shaped.zero();
		_astsmc_reference_acceleration.zero();
		_astsmc_reference_jerk.zero();
		_astsmc_trim_state.zero();
		_astsmc_trim_confidence.zero();
		_astsmc_trim_valid.zero();
		_astsmc_selective_release_active.zero();
		_astsmc_selective_release_completed.zero();
		_astsmc_quiet_anchor_active.zero();
		_astsmc_deep_quiet_active.zero();
		_astsmc_nominal_torque_raw.zero();
		_astsmc_nominal_torque_limited.zero();
		_astsmc_residual_torque_raw.zero();
		_astsmc_residual_torque_limited.zero();
		_astsmc_integral_state.zero();
		_astsmc_reaching_input_raw.zero();
		_astsmc_reaching_input_limited.zero();
		_astsmc_torque_raw.zero();
		_astsmc_torque_limited.zero();
		_astsmc_last_torque.zero();
		_astsmc_actuator_state.zero();
		_astsmc_applied_torque_target.zero();
		_astsmc_command_lower.zero();
		_astsmc_command_upper.zero();
		_astsmc_actuator_constraint.zero();
		_astsmc_variation_regularization.zero();
		_astsmc_internal_saturation.zero();
		_astsmc_nominal_saturation.zero();
		_astsmc_state_recovery_active.zero();
		_astsmc_selective_trigger_sliding.zero();
		_astsmc_selective_trigger_state.zero();
		_astsmc_selective_trigger_trim.zero();
		_astsmc_selective_trigger_reaching.zero();
		_astsmc_ground_containment_active = false;
		_astsmc_total_bound_valid = true;
		_astsmc_seed_torque.zero();
		_astsmc_seed_pending = false;
		_astsmc_reference_valid = false;
		_astsmc_reference_reset_pending = count_reference_reset;
		_astsmc_dt_valid = false;
		_astsmc_previous_airborne_update_invalid = false;
		_astsmc_raw_dt = NAN;
		_astsmc_accepted_dt = NAN;
		resetAstsmcAllocatorFeasibility();

		if (reset_counters) {
			_astsmc_max_airborne_dt = 0.f;
			_astsmc_valid_update_count = 0;
			_astsmc_invalid_dt_hold_count = 0;
			_astsmc_consecutive_invalid_dt_hold_count = 0;
			_astsmc_reference_reset_count = 0;
			_astsmc_nominal_limit_count[0] = 0;
			_astsmc_nominal_limit_count[1] = 0;
			_astsmc_nominal_limit_count[2] = 0;
			_astsmc_residual_limit_count[0] = 0;
			_astsmc_residual_limit_count[1] = 0;
			_astsmc_residual_limit_count[2] = 0;
			_astsmc_selective_release_count[0] = 0;
			_astsmc_selective_release_count[1] = 0;
			_astsmc_selective_release_count[2] = 0;
			_astsmc_quiet_anchor_count[0] = 0;
			_astsmc_quiet_anchor_count[1] = 0;
			_astsmc_quiet_anchor_count[2] = 0;
			_astsmc_yaw_authority_backoff_count = 0;
			_astsmc_yaw_authority_fallback_count = 0;
			_astsmc_total_bound_violation_count = 0;
		}
	}

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral()
	{
		_rate_int.zero();
		resetSmcState();
		resetAstsmcState(true);
		_mpc_rate_int.zero();
		_mpc_last_torque.zero();
		_mpc_actuator_state.zero();
		_mpc_warm_start.zero();
		resetMpcSetpoint();
		_mpc_last_torque_valid = false;
		_last_output.zero();
		_last_output_valid = false;
	}

	/**
	 * Set the integral term to 0 for specific axes
	 * @param  axis roll 0 / pitch 1 / yaw 2
	 * @see _rate_int
	 */
	void resetIntegral(size_t axis)
	{
		if (axis < 3) {
			_rate_int(axis) = 0.f;
			_smc_rate_int(axis) = 0.f;
			_smc_s_filtered(axis) = 0.f;
			_smc_last_torque(axis) = 0.f;
			_smc_last_rate_sp(axis) = 0.f;
			_smc_rate_sp_prev_valid = false;
			_astsmc_surface(axis) = 0.f;
			_astsmc_surface_raw(axis) = 0.f;
			_astsmc_tracking_setpoint(axis) = 0.f;
			_astsmc_rate_sp_raw(axis) = 0.f;
			_astsmc_rate_sp_shaped(axis) = 0.f;
			_astsmc_reference_acceleration(axis) = 0.f;
			_astsmc_reference_jerk(axis) = 0.f;
			_astsmc_trim_state(axis) = 0.f;
			_astsmc_trim_confidence(axis) = 0.f;
			_astsmc_trim_valid(axis) = false;
			_astsmc_selective_release_active(axis) = false;
			_astsmc_selective_release_completed(axis) = false;
			_astsmc_quiet_anchor_active(axis) = false;
			_astsmc_deep_quiet_active(axis) = false;
			_astsmc_nominal_torque_raw(axis) = 0.f;
			_astsmc_nominal_torque_limited(axis) = 0.f;
			_astsmc_residual_torque_raw(axis) = 0.f;
			_astsmc_residual_torque_limited(axis) = 0.f;
			_astsmc_integral_state(axis) = 0.f;
			_astsmc_reaching_input_raw(axis) = 0.f;
			_astsmc_reaching_input_limited(axis) = 0.f;
			_astsmc_torque_raw(axis) = 0.f;
			_astsmc_torque_limited(axis) = 0.f;
			_astsmc_last_torque(axis) = 0.f;
			_astsmc_actuator_state(axis) = 0.f;
			_astsmc_applied_torque_target(axis) = 0.f;
			_astsmc_command_lower(axis) = 0.f;
			_astsmc_command_upper(axis) = 0.f;
			_astsmc_actuator_constraint(axis) = false;
			_astsmc_variation_regularization(axis) = 0.f;
			_astsmc_internal_saturation(axis) = false;
			_astsmc_nominal_saturation(axis) = false;
			_astsmc_seed_torque(axis) = 0.f;
			_astsmc_seed_pending = false;
			_astsmc_reference_valid = false;
			_astsmc_reference_reset_pending = true;
			_astsmc_dt_valid = false;
			_mpc_rate_int(axis) = 0.f;
			_mpc_last_torque(axis) = 0.f;
			_mpc_actuator_state(axis) = 0.f;

			for (int step = 0; step < MPC_MAX_HORIZON; step++) {
				_mpc_warm_start(axis, step) = 0.f;
			}

			_mpc_last_rate_sp(axis) = 0.f;
			_mpc_rate_sp_prev_valid = false;
			_mpc_last_torque_valid = false;
			_last_output(axis) = 0.f;
			_last_output_valid = false;
		}
	}

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);
	void getAstsmcStatus(astsmc_status_s &astsmc_status) const;
	void getAstsmcSafetyStatus(astsmc_safety_status_s &astsmc_safety_status) const;
	float astsmcYawResidualAuthorityBase() const { return _astsmc_residual_torque_limit(2); }
	float astsmcYawResidualAuthorityMaximum() const
	{
		return math::min(_astsmc_torque_limit(2),
				 _astsmc_residual_torque_limit(2) + _astsmc_yaw_residual_extension);
	}
	float astsmcYawResidualAuthorityEffective() const { return _astsmc_yaw_residual_authority_effective; }
	float astsmcYawAuthorityHealthyDwell() const { return _astsmc_yaw_authority_healthy_dwell; }
	bool astsmcAllocatorFeedbackUsable() const { return _astsmc_allocator_feedback_usable; }
	bool astsmcAllocatorConditioningActive(int axis) const
	{
		return axis >= 0 && axis < 2 && _astsmc_allocator_conditioning_active(axis);
	}
	bool astsmcYawAuthorityBackoffActive() const { return _astsmc_yaw_authority_backoff_active; }
	bool astsmcYawAuthorityReleaseActive() const { return _astsmc_yaw_authority_release_active; }
	uint32_t astsmcYawAuthorityBackoffCount() const { return _astsmc_yaw_authority_backoff_count; }
	uint32_t astsmcYawAuthorityFallbackCount() const { return _astsmc_yaw_authority_fallback_count; }

	struct AstsmcScalarStep {
		float reaching_input_raw;
		float reaching_input_limited;
		float state_next;
	};

	static AstsmcScalarStep implicitSuperTwistingStep(float sliding, float state, float k1, float k2,
			float reaching_limit, float dt);
	static float variationRegularizedAppliedTorque(float desired_torque, float previous_torque,
			float residual_authority, float variation_weight);
	static float scheduledAstsmcK1(float configured_k1, float sliding, float quiet_error_threshold,
			bool schedule_enabled, bool deep_quiet_enabled = false,
			float recovery_boost = 0.f, float recovery_error_threshold = 0.f);
	static float astsmcRollPitchResidualAuthority(float base_authority, float extension,
			float sliding, float quiet_error_threshold);

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);
	matrix::Vector3f updateMPC(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				   const float dt, const bool landed);
	matrix::Vector3f updateModelBasedSMC(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
					     const float dt, const bool landed);
	matrix::Vector3f updateImplicitSuperTwisting(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
			const float dt, const bool landed, const float raw_dt, const bool ground_containment);
	void updateSMCIntegral(const matrix::Vector3f &rate_error, const float dt);

	// Controller Type
	int _controller_type{0};

	// Model-based SMC parameters
	matrix::Vector3f _msmc_inertia{0.01f, 0.01f, 0.02f};
	matrix::Vector3f _msmc_control_effectiveness{1.f, 1.f, 1.f};
	matrix::Vector3f _msmc_c{0.8f, 0.8f, 0.5f};
	matrix::Vector3f _msmc_eta{0.5f, 0.5f, 0.3f};
	matrix::Vector3f _msmc_bnd{0.25f, 0.25f, 0.25f};
	matrix::Vector3f _msmc_ks{0.05f, 0.05f, 0.03f};
	matrix::Vector3f _msmc_integral_limit{0.3f, 0.3f, 0.3f};
	matrix::Vector3f _msmc_torque_limit{1.f, 1.f, 1.f};
	float _msmc_rate_sp_derivative_limit{0.f};
	bool _msmc_model_valid{false};

	// Paper augmentation (Bouabdallah/Siegwart ICRA 2005), disabled by zero
	matrix::Vector3f _msmc_attitude_surface_slope; ///< alpha1 per axis
	float _msmc_rotor_inertia{0.f}; ///< propulsion-group inertia J_R
	float _msmc_net_rotor_speed{0.f}; ///< Omega for the current cycle
	bool _msmc_net_rotor_speed_valid{false};

	// Implicit super-twisting parameters
	matrix::Vector3f _astsmc_inertia{0.01f, 0.01f, 0.02f};
	matrix::Vector3f _astsmc_control_effectiveness{1.f, 1.f, 1.f};
	matrix::Vector3f _astsmc_k1{3.f, 3.f, 1.5f};
	matrix::Vector3f _astsmc_k2{4.5f, 4.5f, 1.5f};
	matrix::Vector3f _astsmc_torque_limit{0.2f, 0.2f, 0.17f};
	matrix::Vector3f _astsmc_reference_acceleration_limit{20.f, 20.f, 5.f};
	matrix::Vector3f _astsmc_reference_jerk_limit{100.f, 100.f, 40.f};
	matrix::Vector3f _astsmc_residual_torque_limit{0.1f, 0.1f, 0.05f};
	matrix::Vector3f _astsmc_actuator_time_constant;
	matrix::Vector3f _astsmc_torque_slew_rate;
	matrix::Vector3f _astsmc_variation_weight;
	matrix::Vector3f _astsmc_sliding_boundary;
	float _astsmc_tracking_blend{0.f};
	float _astsmc_reference_feedforward{1.f};
	float _astsmc_reference_feedforward_rp{1.f};
	float _astsmc_gyro_compensation{1.f};
	float _astsmc_dt_min{0.0005f};
	float _astsmc_dt_max{0.005f};
	float _astsmc_recovery_error_threshold{1.f};
	float _astsmc_quiet_k1_error_threshold{0.25f};
	float _astsmc_selective_release_error_threshold{0.2f};
	float _astsmc_selective_release_state_rate{20.f};
	float _astsmc_trim_command_threshold{0.15f};
	float _astsmc_trim_error_threshold{0.2f};
	float _astsmc_trim_acceleration_threshold{1.f};
	float _astsmc_trim_confidence_time{0.5f};
	float _astsmc_trim_time_constant{20.f};
	float _astsmc_roll_pitch_residual_extension{0.f};
	float _astsmc_roll_pitch_k1_recovery_boost{0.f};
	float _astsmc_yaw_residual_extension{0.f};
	float _astsmc_yaw_residual_authority_effective{0.05f};
	float _astsmc_yaw_authority_healthy_dwell{0.f};
	matrix::Vector3f _astsmc_allocator_allocated_torque;
	matrix::Vector3f _astsmc_allocator_unallocated_torque;
	matrix::Vector<bool, 3> _astsmc_allocator_conditioning_active;
	bool _astsmc_allocator_feedback_usable{false};
	bool _astsmc_allocator_tightly_feasible{false};
	bool _astsmc_yaw_authority_backoff_active{false};
	bool _astsmc_yaw_authority_release_active{false};
	uint32_t _astsmc_yaw_authority_backoff_count{0};
	uint32_t _astsmc_yaw_authority_fallback_count{0};
	bool _astsmc_model_valid{false};

	// Rate MPC parameters
	static constexpr int MPC_MAX_HORIZON = 8;
	matrix::Vector3f _mpc_inertia{0.01f, 0.01f, 0.02f};
	matrix::Vector3f _mpc_rate_weight{1.f, 1.f, 1.f};
	matrix::Vector3f _mpc_control_effectiveness{1.f, 1.f, 1.f};
	matrix::Vector3f _mpc_torque_weight{1.f, 1.f, 1.f};
	matrix::Vector3f _mpc_torque_rate_weight;
	matrix::Vector3f _mpc_integral_gain;
	matrix::Vector3f _mpc_integral_limit{0.3f, 0.3f, 0.3f};
	matrix::Vector3f _mpc_torque_limit{1.f, 1.f, 1.f};
	matrix::Vector3f _mpc_rate_int;
	matrix::Vector3f _mpc_last_torque;
	matrix::Vector3f _mpc_actuator_state;
	matrix::Vector3f _mpc_last_rate_sp;
	matrix::Matrix<float, 3, MPC_MAX_HORIZON> _mpc_warm_start;
	int _mpc_horizon{6};
	float _mpc_torque_slew_rate{10.f};
	float _mpc_rate_sp_derivative_limit{0.f};
	float _mpc_gyro_compensation_weight{0.f};
	float _mpc_actuator_time_constant{0.02f};
	bool _mpc_last_torque_valid{false};
	bool _mpc_rate_sp_prev_valid{false};

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller
	matrix::Vector3f _smc_rate_int; ///< integral term of the SMC controller
	matrix::Vector3f _smc_surface; ///< unfiltered model-based SMC sliding surface
	matrix::Vector3f _smc_s_filtered; ///< filtered sliding surface state
	matrix::Vector3f _smc_torque_raw; ///< normalized SMC torque before limits
	matrix::Vector3f _smc_torque_limited; ///< normalized SMC torque after limits
	matrix::Vector3f _smc_last_torque; ///< last output torque state
	matrix::Vector3f _smc_last_rate_sp; ///< last rate setpoint for model-based SMC feed-forward acceleration
	bool _smc_rate_sp_prev_valid{false};
	matrix::Vector3f _astsmc_surface;
	matrix::Vector3f _astsmc_surface_raw;
	matrix::Vector3f _astsmc_tracking_setpoint;
	matrix::Vector3f _astsmc_rate_sp_raw;
	matrix::Vector3f _astsmc_rate_sp_shaped;
	matrix::Vector3f _astsmc_reference_acceleration;
	matrix::Vector3f _astsmc_reference_jerk;
	matrix::Vector3f _astsmc_trim_state;
	matrix::Vector3f _astsmc_trim_confidence;
	matrix::Vector<bool, 3> _astsmc_trim_valid;
	matrix::Vector<bool, 3> _astsmc_selective_release_active;
	matrix::Vector<bool, 3> _astsmc_selective_release_completed;
	matrix::Vector<bool, 3> _astsmc_quiet_anchor_active;
	matrix::Vector<bool, 3> _astsmc_deep_quiet_active;
	matrix::Vector3f _astsmc_nominal_torque_raw;
	matrix::Vector3f _astsmc_nominal_torque_limited;
	matrix::Vector3f _astsmc_residual_torque_raw;
	matrix::Vector3f _astsmc_residual_torque_limited;
	matrix::Vector3f _astsmc_integral_state;
	matrix::Vector3f _astsmc_reaching_input_raw;
	matrix::Vector3f _astsmc_reaching_input_limited;
	matrix::Vector3f _astsmc_torque_raw;
	matrix::Vector3f _astsmc_torque_limited;
	matrix::Vector3f _astsmc_last_torque;
	matrix::Vector3f _astsmc_actuator_state;
	matrix::Vector3f _astsmc_applied_torque_target;
	matrix::Vector3f _astsmc_command_lower;
	matrix::Vector3f _astsmc_command_upper;
	matrix::Vector3f _astsmc_variation_regularization;
	matrix::Vector<bool, 3> _astsmc_actuator_constraint;
	matrix::Vector<bool, 3> _astsmc_internal_saturation;
	matrix::Vector<bool, 3> _astsmc_nominal_saturation;
	matrix::Vector<bool, 3> _astsmc_state_recovery_active;
	matrix::Vector3f _astsmc_recovery_trigger_sliding;
	matrix::Vector3f _astsmc_recovery_trigger_state;
	matrix::Vector3f _astsmc_recovery_trigger_reaching;
	matrix::Vector3f _astsmc_seed_torque;
	VelocitySmoothing _astsmc_reference[3];
	bool _astsmc_seed_pending{false};
	bool _astsmc_reference_valid{false};
	bool _astsmc_reference_reset_pending{true};
	bool _astsmc_dt_valid{false};
	bool _astsmc_previous_airborne_update_invalid{false};
	bool _astsmc_total_bound_valid{true};
	float _astsmc_raw_dt{NAN};
	float _astsmc_accepted_dt{NAN};
	float _astsmc_max_airborne_dt{0.f};
	uint32_t _astsmc_valid_update_count{0};
	uint32_t _astsmc_invalid_dt_hold_count{0};
	uint32_t _astsmc_consecutive_invalid_dt_hold_count{0};
	uint32_t _astsmc_reference_reset_count{0};
	uint32_t _astsmc_nominal_limit_count[3] {};
	uint32_t _astsmc_residual_limit_count[3] {};
	uint32_t _astsmc_total_bound_violation_count{0};
	uint32_t _astsmc_selective_release_count[3] {};
	uint32_t _astsmc_quiet_anchor_count[3] {};
	uint32_t _astsmc_state_recovery_count[3] {};
	matrix::Vector3f _astsmc_selective_trigger_sliding;
	matrix::Vector3f _astsmc_selective_trigger_state;
	matrix::Vector3f _astsmc_selective_trigger_trim;
	matrix::Vector3f _astsmc_selective_trigger_reaching;
	uint32_t _astsmc_ground_containment_count{0};
	uint32_t _astsmc_runtime_fault_reason{0};
	bool _astsmc_ground_containment_active{false};
	bool _astsmc_runtime_fault_latched{false};
	matrix::Vector3f _last_output;
	bool _last_output_valid{false};

	// Safeguards configurations
	float _smc_lpf_cutoff{20.0f}; ///< Cutoff frequency for sliding surface LPF (Hz)
	float _smc_slew_max{10.0f}; ///< Maximum torque rate of change per second

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
