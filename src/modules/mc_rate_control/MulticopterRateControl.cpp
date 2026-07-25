/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"
#include "RateControlLandingPolicy.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

ModuleBase::Descriptor MulticopterRateControl::desc{task_spawn, custom_command, print_usage};

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_is_vtol = vtol;

	parameters_updated();
	_astsmc_safety_status_pub.advertise();
	_astsmc_status_pub.advertise();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

MulticopterRateControl::RateSetpointSource
MulticopterRateControl::getRateSetpointSource() const
{
	if (!_vehicle_control_mode.flag_control_rates_enabled) {
		return RateSetpointSource::Disabled;
	}

	if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
		return RateSetpointSource::ManualRate;
	}

	if (_vehicle_control_mode.flag_control_position_enabled) {
		return RateSetpointSource::Position;
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled) {
		return RateSetpointSource::Velocity;
	}

	if (_vehicle_control_mode.flag_control_altitude_enabled || _vehicle_control_mode.flag_control_climb_rate_enabled) {
		return RateSetpointSource::Altitude;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		return RateSetpointSource::Attitude;
	}

	return RateSetpointSource::Topic;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

	_rate_control.setMpcGains(
		Vector3f(_param_mc_mpc_j_roll.get(), _param_mc_mpc_j_pitch.get(), _param_mc_mpc_j_yaw.get()),
		Vector3f(_param_mc_mpc_q_roll.get(), _param_mc_mpc_q_pitch.get(), _param_mc_mpc_q_yaw.get()),
		Vector3f(_param_mc_mpc_eff_roll.get(), _param_mc_mpc_eff_pitch.get(), _param_mc_mpc_eff_yaw.get()),
		Vector3f(_param_mc_mpc_r_roll.get(), _param_mc_mpc_r_pitch.get(), _param_mc_mpc_r_yaw.get()),
		Vector3f(_param_mc_mpc_du_roll.get(), _param_mc_mpc_du_pitch.get(), _param_mc_mpc_du_yaw.get()),
		_param_mc_mpc_horizon.get(),
		_param_mc_mpc_slew.get()
	);

	_rate_control.setMpcIntegralGain(
		Vector3f(_param_mc_mpc_i_roll.get(), _param_mc_mpc_i_pitch.get(), _param_mc_mpc_i_yaw.get()));
	_rate_control.setMpcIntegralLimit(
		Vector3f(_param_mc_mpc_ilim_roll.get(), _param_mc_mpc_ilim_pitch.get(), _param_mc_mpc_ilim_yaw.get()));

	_rate_control.setMpcTorqueLimit(
		Vector3f(_param_mc_mpc_tmax_roll.get(), _param_mc_mpc_tmax_pitch.get(), _param_mc_mpc_tmax_yaw.get()));

	_rate_control.setMpcRateSetpointDerivativeLimit(_param_mc_mpc_rate_sp_deriv_lim.get());

	_rate_control.setMpcGyroCompensation(_param_mc_mpc_gyro.get());
	_rate_control.setMpcActuatorTimeConstant(_param_mc_mpc_tau.get());

	const bool smc_card_acknowledged = _param_mc_msmc_cfg.get() == 1;
	const bool smc_parameters_valid = smc_card_acknowledged
					  && _rate_control.setModelBasedSmcParameters(
							  Vector3f(_param_mc_msmc_j_roll.get(), _param_mc_msmc_j_pitch.get(),
									  _param_mc_msmc_j_yaw.get()),
							  Vector3f(_param_mc_msmc_eff_roll.get(), _param_mc_msmc_eff_pitch.get(),
									  _param_mc_msmc_eff_yaw.get()),
							  Vector3f(_param_mc_msmc_c_roll.get(), _param_mc_msmc_c_pitch.get(),
									  _param_mc_msmc_c_yaw.get()),
							  Vector3f(_param_mc_msmc_eta_roll.get(), _param_mc_msmc_eta_pitch.get(),
									  _param_mc_msmc_eta_yaw.get()),
							  Vector3f(_param_mc_msmc_bnd_roll.get(), _param_mc_msmc_bnd_pitch.get(),
									  _param_mc_msmc_bnd_yaw.get()),
							  Vector3f(_param_mc_msmc_ks_roll.get(), _param_mc_msmc_ks_pitch.get(),
									  _param_mc_msmc_ks_yaw.get()),
							  _param_mc_msmc_rate_sp_deriv_lim.get(),
							  Vector3f(_param_mc_msmc_ilim_roll.get(), _param_mc_msmc_ilim_pitch.get(),
									  _param_mc_msmc_ilim_yaw.get()),
							  Vector3f(_param_mc_msmc_tmax_roll.get(), _param_mc_msmc_tmax_pitch.get(),
									  _param_mc_msmc_tmax_yaw.get()),
							  _param_mc_smc_lpf.get(), _param_mc_smc_slew.get());

	// The paper augmentation is an addition to the acknowledged card, so a
	// rejected constant leaves the plain rate-loop law running rather than
	// invalidating the card.
	const bool smc_paper_constants_valid = _rate_control.setModelBasedSmcPaperConstants(
			Vector3f(_param_mc_msmc_a1_roll.get(), _param_mc_msmc_a1_pitch.get(),
				 _param_mc_msmc_a1_yaw.get()),
			_param_mc_msmc_rotor_inertia.get());

	if (!smc_paper_constants_valid) {
		_rate_control.setModelBasedSmcPaperConstants(Vector3f(), 0.f);
	}

	_smc_configuration_valid = smc_parameters_valid;

	const bool astsmc_card_acknowledged = _param_mc_ast_cfg.get() == 1;
	// VTOL attitude control scales/blends the virtual multicopter torque after this module.
	const bool astsmc_runtime_compatible = !_param_mc_bat_scale_en.get() && !_is_vtol;
	const int32_t astsmc_roll_pitch_airmode = _param_mc_ast_roll_pitch_airmode.get();
	const bool astsmc_allocation_policy_valid = astsmc_roll_pitch_airmode == 0 || astsmc_roll_pitch_airmode == 1;
	const bool astsmc_parameters_valid = astsmc_card_acknowledged && astsmc_runtime_compatible
					     && astsmc_allocation_policy_valid
					     && _rate_control.setAstsmcParameters(
							     Vector3f(_param_mc_ast_j_roll.get(), _param_mc_ast_j_pitch.get(),
									     _param_mc_ast_j_yaw.get()),
							     Vector3f(_param_mc_ast_eff_roll.get(), _param_mc_ast_eff_pitch.get(),
									     _param_mc_ast_eff_yaw.get()),
							     Vector3f(_param_mc_ast_k1_roll.get(), _param_mc_ast_k1_pitch.get(),
									     _param_mc_ast_k1_yaw.get()),
							     Vector3f(_param_mc_ast_k2_roll.get(), _param_mc_ast_k2_pitch.get(),
									     _param_mc_ast_k2_yaw.get()),
							     Vector3f(_param_mc_ast_tmax_roll.get(), _param_mc_ast_tmax_pitch.get(),
									     _param_mc_ast_tmax_yaw.get()),
							     Vector3f(_param_mc_ast_racc_roll.get(), _param_mc_ast_racc_pitch.get(),
									     _param_mc_ast_racc_yaw.get()),
							     Vector3f(_param_mc_ast_rjerk_roll.get(), _param_mc_ast_rjerk_pitch.get(),
									     _param_mc_ast_rjerk_yaw.get()),
							     Vector3f(_param_mc_ast_tres_roll.get(), _param_mc_ast_tres_pitch.get(),
									     _param_mc_ast_tres_yaw.get()),
							     Vector3f(_param_mc_ast_tau_roll.get(), _param_mc_ast_tau_pitch.get(),
								     _param_mc_ast_tau_yaw.get()),
							     Vector3f(_param_mc_ast_slew_roll.get(), _param_mc_ast_slew_pitch.get(),
								     _param_mc_ast_slew_yaw.get()),
							     Vector3f(_param_mc_ast_du_roll.get(), _param_mc_ast_du_pitch.get(),
								     _param_mc_ast_du_yaw.get()),
							     Vector3f(_param_mc_ast_sbd_roll.get(), _param_mc_ast_sbd_pitch.get(),
								     _param_mc_ast_sbd_yaw.get()),
							     _param_mc_ast_tracking_blend.get(), _param_mc_ast_rff.get(),
							     _param_mc_ast_rff_rp.get(), _param_mc_ast_gyro.get(),
							     _param_mc_ast_dt_min.get(), _param_mc_ast_dt_max.get(),
							     _param_mc_ast_recovery_error.get(),
							     _param_mc_ast_quiet_k1_error.get(),
							     _param_mc_ast_selective_release_error.get(),
							     _param_mc_ast_selective_release_rate.get(),
							     _param_mc_ast_trim_command.get(), _param_mc_ast_trim_error.get(),
							     _param_mc_ast_trim_acceleration.get(),
							     _param_mc_ast_trim_confidence_time.get(),
							     _param_mc_ast_trim_time_constant.get(),
							     _param_mc_ast_roll_pitch_extension.get(),
							     _param_mc_ast_roll_pitch_k1_recovery_boost.get(),
							     _param_mc_ast_yaw_extension.get());

	resetAstsmcAllocatorFeedbackEpoch();
	_astsmc_configuration_valid = astsmc_parameters_valid;
	const int requested_controller = _param_mc_rate_ctrl_t.get();

	if (requested_controller == 3 && _rate_control.getControllerType() == 3 && !astsmc_parameters_valid) {
		_rate_control.latchAstsmcRuntimeFault(astsmc_safety_status_s::RUNTIME_FAULT_CONFIGURATION);
	}

	if (requested_controller == 2 && !_smc_configuration_valid) {
		PX4_ERR("SMC configuration rejected; set MC_MSMC_CFG=1 after model review");

	} else if (requested_controller == 3 && !_astsmc_configuration_valid) {
		PX4_ERR("implicit super-twisting configuration rejected; review MC_AST_CFG, MC_AST_RP_AIR, battery scaling, and vehicle type");

	} else if (!_rate_control.setControllerType(requested_controller)) {
		PX4_ERR("rate controller selection rejected: %d", requested_controller);
	}

	if (_param_mc_rate_ctrl_t.get() == 1
	    && (Vector3f(_param_mc_mpc_j_roll.get(), _param_mc_mpc_j_pitch.get(), _param_mc_mpc_j_yaw.get()).min() <= 0.f
		|| Vector3f(_param_mc_mpc_eff_roll.get(), _param_mc_mpc_eff_pitch.get(), _param_mc_mpc_eff_yaw.get()).min() <= 0.f)) {
		PX4_WARN("invalid MPC model update rejected; retaining last valid J/EFF");
	}

	if (smc_card_acknowledged && !smc_parameters_valid) {
		PX4_ERR("invalid SMC card rejected; retaining last valid model");
	}

	if (astsmc_card_acknowledged && astsmc_runtime_compatible && !astsmc_parameters_valid) {
		PX4_ERR("invalid implicit super-twisting card rejected; retaining last valid model");
	}

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

void MulticopterRateControl::resetAstsmcAllocatorFeedbackEpoch()
{
	_astsmc_allocator_epoch_active = false;
	_astsmc_allocator_feedback_usable = false;
	_astsmc_allocator_actuator_bound = false;
	_astsmc_first_command_sample = 0;
	_astsmc_last_command_sample = 0;
	_astsmc_last_accepted_allocator_sample = 0;
	_astsmc_allocator_publication_age_s = NAN;
	_astsmc_allocator_sample_age_s = NAN;
	_astsmc_allocator_rejection_flags = 0;
	_astsmc_roll_pitch_miss_active = false;
	_astsmc_roll_pitch_miss_started = 0;
	_astsmc_roll_pitch_miss_reported = false;
	_rate_control.resetAstsmcAllocatorFeasibility();
}

void MulticopterRateControl::updateModelBasedSmcPaperState(uint64_t current_sample)
{
	// The rotor term fails closed: without a fresh motor command the controller
	// runs the plain rate-loop law instead of extrapolating a stale speed. The
	// attitude-surface term needs no state, because z1 is taken from the rate
	// setpoint the loop is already tracking.
	static constexpr uint64_t kMaximumStateAge = 50_ms;

	_actuator_motors_sub.update(&_actuator_motors);

	float net_rotor_speed = 0.f;
	bool net_rotor_speed_valid = false;

	const float rotor_speed_gain = _param_mc_msmc_rotor_speed_gain.get();
	const bool rotor_term_enabled = _param_mc_msmc_rotor_inertia.get() > FLT_EPSILON
					&& rotor_speed_gain > FLT_EPSILON;
	const bool motors_fresh = _actuator_motors.timestamp != 0
				  && current_sample >= _actuator_motors.timestamp
				  && current_sample - _actuator_motors.timestamp <= kMaximumStateAge;

	if (rotor_term_enabled && motors_fresh) {
		const int32_t reversed = _param_mc_msmc_rotor_directions.get();
		net_rotor_speed_valid = true;

		for (int motor = 0; motor < actuator_motors_s::NUM_CONTROLS; motor++) {
			const float command = _actuator_motors.control[motor];

			if (!PX4_ISFINITE(command)) {
				continue;
			}

			// Thrust is proportional to the squared propeller speed, so a
			// normalized command maps to speed through a square root.
			const float speed = rotor_speed_gain * sqrtf(math::constrain(command, 0.f, 1.f));
			net_rotor_speed += ((reversed >> motor) & 1) ? -speed : speed;
		}
	}

	_rate_control.setModelBasedSmcPaperState(net_rotor_speed, net_rotor_speed_valid);
}

RateControl::AstsmcAllocatorFeedback MulticopterRateControl::updateAstsmcAllocatorFeedback(
		uint64_t current_sample, bool eligible, bool new_status)
{
	RateControl::AstsmcAllocatorFeedback feedback{};

	if (eligible && !_astsmc_allocator_epoch_active) {
		resetAstsmcAllocatorFeedbackEpoch();
		_astsmc_allocator_epoch_active = true;

	} else if (!eligible && _astsmc_allocator_epoch_active) {
		resetAstsmcAllocatorFeedbackEpoch();
	}

	AstsmcAllocatorFeedbackSample sample{};
	sample.timestamp = _control_allocator_status.timestamp;
	sample.timestamp_sample = _control_allocator_status.timestamp_sample;
	sample.torque_setpoint_achieved = _control_allocator_status.torque_setpoint_achieved;
	sample.thrust_setpoint_achieved = _control_allocator_status.thrust_setpoint_achieved;
	sample.preflight_active = _control_allocator_status.actuator_group_preflight_check_active;
	sample.handled_motor_failure_mask = _control_allocator_status.handled_motor_failure_mask;
	sample.motor_stop_mask = _control_allocator_status.motor_stop_mask;

	for (int axis = 0; axis < 3; axis++) {
		sample.allocated_torque[axis] = _control_allocator_status.allocated_torque[axis];
		sample.unallocated_torque[axis] = _control_allocator_status.unallocated_torque[axis];
		sample.allocated_thrust[axis] = _control_allocator_status.allocated_thrust[axis];
		sample.unallocated_thrust[axis] = _control_allocator_status.unallocated_thrust[axis];
	}

	for (int actuator = 0; actuator < 16; actuator++) {
		if (_control_allocator_status.actuator_saturation[actuator]
		    != control_allocator_status_s::ACTUATOR_SATURATION_OK) {
			sample.actuator_bound = true;
			break;
		}
	}

	AstsmcAllocatorFeedbackContext context{};
	context.eligible = eligible;
	context.require_new_sample = new_status;
	context.now = hrt_absolute_time();
	context.current_sample = current_sample;
	context.first_command_sample = _astsmc_first_command_sample;
	context.last_command_sample = _astsmc_last_command_sample;
	context.last_accepted_sample = _astsmc_last_accepted_allocator_sample;
	const AstsmcAllocatorFeedbackGateResult gate = astsmcAllocatorFeedbackGate(sample, context);
	bool usable = gate.usable;

	if (!new_status && sample.timestamp_sample != _astsmc_last_accepted_allocator_sample) {
		usable = false;
	}

	_astsmc_allocator_feedback_usable = usable;
	_astsmc_allocator_actuator_bound = sample.actuator_bound;
	_astsmc_allocator_publication_age_s = gate.publication_age_s;
	_astsmc_allocator_sample_age_s = gate.sample_age_s;
	_astsmc_allocator_rejection_flags = usable ? 0 : gate.rejection_flags;

	if (new_status && eligible) {
		if (usable) {
			_astsmc_last_accepted_allocator_sample = sample.timestamp_sample;
			_astsmc_allocator_accepted_count++;

		} else {
			_astsmc_allocator_rejected_count++;
		}
	}

	feedback.usable = usable;
	feedback.new_sample = new_status && usable;
	feedback.torque_setpoint_achieved = sample.torque_setpoint_achieved;
	feedback.thrust_setpoint_achieved = sample.thrust_setpoint_achieved;
	feedback.actuator_bound = sample.actuator_bound;
	feedback.allocated_torque = Vector3f(sample.allocated_torque);
	feedback.unallocated_torque = Vector3f(sample.unallocated_torque);
	return feedback;
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		const float raw_dt = (now - _last_run) * 1e-6f;
		// Guard non-type-3 controllers against too small (< 0.125ms) and too large (> 20ms) integration steps.
		const float dt = math::constrain(raw_dt, 0.000125f, 0.02f);
		const float controller_dt = _rate_control.getControllerType() == 3 ? raw_dt : dt;
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);

		const RateSetpointSource rate_setpoint_source = getRateSetpointSource();

		if (!_rate_setpoint_context_valid
		    || _rate_setpoint_source != rate_setpoint_source
		    || _rate_setpoint_nav_state != _vehicle_status.nav_state) {
			if (_rate_setpoint_context_valid) {
				_rate_control.resetSetpointHistory();
			}

			_rate_setpoint_source = rate_setpoint_source;
			_rate_setpoint_nav_state = _vehicle_status.nav_state;
			_rate_setpoint_context_valid = true;
		}

		if (!_vehicle_control_mode.flag_armed) {
			_astsmc_released_since_arming = false;
			_astsmc_no_ground_contact_since = 0;
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
				_ground_contact = vehicle_land_detected.ground_contact;
				_has_low_throttle = vehicle_land_detected.has_low_throttle;

				if (_vehicle_control_mode.flag_armed && !_landed && !_maybe_landed && !_ground_contact) {
					if (_astsmc_no_ground_contact_since == 0) {
						_astsmc_no_ground_contact_since = vehicle_land_detected.timestamp;
					}

				} else {
					_astsmc_no_ground_contact_since = 0;
				}

				if (rateControlReleaseEvidence(_vehicle_control_mode.flag_armed, _landed, _maybe_landed,
							_ground_contact, _has_low_throttle, false)) {
					_astsmc_released_since_arming = true;
				}
			}
		}

		constexpr hrt_abstime astsmc_low_throttle_release_delay = 500_ms;
		const bool low_throttle_release_confirmed = rateControlLowThrottleReleaseConfirmed(
					now, _astsmc_no_ground_contact_since, astsmc_low_throttle_release_delay);

		if (rateControlReleaseEvidence(_vehicle_control_mode.flag_armed, _landed, _maybe_landed,
					_ground_contact, _has_low_throttle, low_throttle_release_confirmed)) {
			_astsmc_released_since_arming = true;
		}

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
				resetAstsmcAllocatorFeedbackEpoch();
			}

			// update saturation status from control allocation feedback
			const bool new_allocator_status = _control_allocator_status_sub.update(&_control_allocator_status);

			if (new_allocator_status) {
				_control_allocator_feedback_valid = true;
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				saturation_positive.zero();
				saturation_negative.zero();

				if (!_control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (_control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (_control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			if (_rate_control.getControllerType() == 2) {
				updateModelBasedSmcPaperState(angular_velocity.timestamp_sample);
			}

			// run rate controller
			const bool controller_landed = rateControlLanded(_rate_control.getControllerType(), _landed, _maybe_landed);
			const bool ground_containment = rateControlGroundContained(_rate_control.getControllerType(),
							      _vehicle_control_mode.flag_armed, _astsmc_released_since_arming, _landed,
							      _maybe_landed, _ground_contact, _has_low_throttle);
			const bool allocator_feedback_eligible = _rate_control.getControllerType() == 3
					&& _vehicle_control_mode.flag_armed && !controller_landed && !ground_containment;
			_rate_control.setAstsmcAllocatorFeedback(updateAstsmcAllocatorFeedback(
						angular_velocity.timestamp_sample, allocator_feedback_eligible, new_allocator_status));
			Vector3f torque_setpoint = _rate_control.update(rates, _rates_setpoint, angular_accel, controller_dt,
							 controller_landed, raw_dt, ground_containment);

			// Type 3 conditions its state on the command limit and must not have an unobserved post-controller filter.
			if (_rate_control.getControllerType() == 3) {
				_output_lpf_yaw.reset(torque_setpoint(2));

			} else {
				torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);
			}

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			const bool headroom_requested = _rate_control.getControllerType() == 3 && _astsmc_configuration_valid
					&& _param_mc_ast_roll_pitch_airmode.get() && allocator_feedback_eligible;

			if (headroom_requested) {
				vehicle_torque_setpoint.allocation_policy =
					vehicle_torque_setpoint_s::ALLOCATION_POLICY_ROLL_PITCH_HEADROOM;
			}

			if (headroom_requested) {
				_astsmc_headroom_request_count++;
			}

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get() && _rate_control.getControllerType() != 3) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			if (_rate_control.getControllerType() == 3 && allocator_feedback_eligible) {
				if (_astsmc_first_command_sample == 0) {
					_astsmc_first_command_sample = vehicle_torque_setpoint.timestamp_sample;
				}

				_astsmc_last_command_sample = vehicle_torque_setpoint.timestamp_sample;
			}

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		} else {
			_rate_control.resetSetpointHistoryForControlDisable();
			resetAstsmcAllocatorFeedbackEpoch();
		}

		// Publish configuration validity even while rate control is disabled.
		rate_ctrl_status_s rate_ctrl_status{};
		_rate_control.getRateControlStatus(rate_ctrl_status);
		rate_ctrl_status.model_valid = rate_ctrl_status.model_valid && _smc_configuration_valid;
		rate_ctrl_status.astsmc_valid = rate_ctrl_status.astsmc_valid && _astsmc_configuration_valid;
		rate_ctrl_status.timestamp = hrt_absolute_time();
		_controller_status_pub.publish(rate_ctrl_status);

		if (_param_mc_rate_ctrl_t.get() == 3 || _rate_control.getControllerType() == 3) {
			astsmc_safety_status_s astsmc_safety_status{};
			astsmc_status_s astsmc_status{};
			_rate_control.getAstsmcSafetyStatus(astsmc_safety_status);
			_rate_control.getAstsmcStatus(astsmc_status);

			for (int axis = 0; axis < 3; axis++) {
				if (axis < 2 && astsmc_safety_status.selective_release_count[axis]
				    > _astsmc_reported_selective_release_count[axis]) {
					events::send<uint8_t, float, float, float>(events::ID("mc_rate_control_astsmc_selective_release"),
						{events::Log::Info, events::LogInternal::Info},
						"ASTSMC selective release axis {1}: s={2:.2}, q={3:.2}, trim={4:.2}",
						static_cast<uint8_t>(axis), astsmc_safety_status.selective_trigger_sliding[axis],
						astsmc_safety_status.selective_trigger_state[axis],
						astsmc_safety_status.selective_trigger_trim[axis]);
					_astsmc_reported_selective_release_count[axis] = astsmc_safety_status.selective_release_count[axis];
				}

				if (astsmc_safety_status.state_recovery_count[axis] > _astsmc_reported_recovery_count[axis]) {
					events::send<uint8_t, float, float, float>(events::ID("mc_rate_control_astsmc_state_recovery"),
						{events::Log::Critical, events::LogInternal::Warning},
						"ASTSMC state recovery on axis {1}: s={2:.2}, q={3:.2}, r={4:.2}",
						static_cast<uint8_t>(axis), astsmc_safety_status.recovery_trigger_sliding[axis],
						astsmc_safety_status.recovery_trigger_state[axis], astsmc_safety_status.recovery_trigger_reaching[axis]);
					_astsmc_reported_recovery_count[axis] = astsmc_safety_status.state_recovery_count[axis];
				}
			}

			const uint32_t new_fault_reasons = astsmc_safety_status.runtime_fault_reason & ~_astsmc_reported_fault_reason;

			if (new_fault_reasons != 0) {
				events::send<uint32_t>(events::ID("mc_rate_control_astsmc_runtime_fault"),
					{events::Log::Critical, events::LogInternal::Warning},
					"ASTSMC runtime fault latched: {1}", new_fault_reasons);
				_astsmc_reported_fault_reason |= new_fault_reasons;
			}
			astsmc_status.configuration_valid = astsmc_status.configuration_valid && _astsmc_configuration_valid;
			astsmc_status.timestamp_sample = angular_velocity.timestamp_sample;
			astsmc_status.timestamp = hrt_absolute_time();
			astsmc_safety_status.timestamp_sample = angular_velocity.timestamp_sample;
			astsmc_safety_status.timestamp = astsmc_status.timestamp;
			astsmc_status.allocator_feedback_valid = _control_allocator_feedback_valid;

			if (_control_allocator_feedback_valid) {
				const hrt_abstime feedback_age = astsmc_status.timestamp >= _control_allocator_status.timestamp ?
								 astsmc_status.timestamp - _control_allocator_status.timestamp : 0;
				astsmc_status.allocator_feedback_age_s = static_cast<float>(feedback_age) * 1e-6f;
				astsmc_status.allocator_feedback_stale = feedback_age > 20_ms;
				astsmc_status.allocator_torque_setpoint_achieved = _control_allocator_status.torque_setpoint_achieved;
				astsmc_status.allocator_timestamp_sample = _control_allocator_status.timestamp_sample;

				for (int axis = 0; axis < 3; axis++) {
					astsmc_status.allocated_torque[axis] = _control_allocator_status.allocated_torque[axis];
					astsmc_status.allocation_residual[axis] = _control_allocator_status.unallocated_torque[axis];
				}

			} else {
				astsmc_status.allocator_feedback_age_s = NAN;
				astsmc_status.allocator_feedback_stale = true;
			}

			astsmc_allocator_status_s astsmc_allocator_status{};
			astsmc_allocator_status.timestamp = astsmc_status.timestamp;
			astsmc_allocator_status.timestamp_sample = angular_velocity.timestamp_sample;
			astsmc_allocator_status.allocator_timestamp_sample = _control_allocator_status.timestamp_sample;
			astsmc_allocator_status.publication_age_s = _astsmc_allocator_publication_age_s;
			astsmc_allocator_status.sample_age_s = _astsmc_allocator_sample_age_s;
			astsmc_allocator_status.allocation_residual_norm = Vector3f(
					_control_allocator_status.unallocated_torque).norm();
			const Vector2f requested_roll_pitch_torque = Vector2f(_control_allocator_status.allocated_torque)
					+ Vector2f(_control_allocator_status.unallocated_torque);
			const Vector2f allocated_roll_pitch_torque(_control_allocator_status.allocated_torque);
			const float requested_roll_pitch_norm = _astsmc_allocator_feedback_usable
					? requested_roll_pitch_torque.norm() : NAN;
			const float allocated_roll_pitch_norm = _astsmc_allocator_feedback_usable
					? allocated_roll_pitch_torque.norm() : NAN;
			astsmc_allocator_status.roll_pitch_torque_norm[0] = requested_roll_pitch_norm;
			astsmc_allocator_status.roll_pitch_torque_norm[1] = allocated_roll_pitch_norm;
			astsmc_allocator_status.roll_pitch_achieved_fraction = PX4_ISFINITE(requested_roll_pitch_norm)
					? (requested_roll_pitch_norm > 0.001f
					   ? math::constrain(allocated_roll_pitch_norm / requested_roll_pitch_norm, 0.f, 1.f) : 1.f)
					: NAN;
			astsmc_allocator_status.requested_allocation_policy =
					_control_allocator_status.requested_allocation_policy;
			astsmc_allocator_status.applied_allocation_policy =
					_control_allocator_status.applied_allocation_policy;
			astsmc_allocator_status.yaw_residual_authority[0] = _rate_control.astsmcYawResidualAuthorityBase();
			astsmc_allocator_status.yaw_residual_authority[1] = _rate_control.astsmcYawResidualAuthorityMaximum();
			astsmc_allocator_status.yaw_residual_authority[2] = _rate_control.astsmcYawResidualAuthorityEffective();
			astsmc_allocator_status.event_count[0] = _astsmc_allocator_accepted_count;
			astsmc_allocator_status.event_count[1] = _astsmc_allocator_rejected_count;
			astsmc_allocator_status.event_count[2] = _rate_control.astsmcYawAuthorityBackoffCount();
			astsmc_allocator_status.event_count[3] = _rate_control.astsmcYawAuthorityFallbackCount();
			astsmc_allocator_status.event_count[4] = _astsmc_headroom_request_count;
			astsmc_allocator_status.rejection_flags = _astsmc_allocator_rejection_flags;

			const bool headroom_was_requested = astsmc_allocator_status.requested_allocation_policy
					== control_allocator_status_s::ALLOCATION_POLICY_ROLL_PITCH_HEADROOM;
			const bool headroom_was_applied = astsmc_allocator_status.applied_allocation_policy
					== control_allocator_status_s::ALLOCATION_POLICY_ROLL_PITCH_HEADROOM;
			const bool roll_pitch_miss = _astsmc_allocator_feedback_usable
					&& (fabsf(_control_allocator_status.unallocated_torque[0]) >= 0.001f
					    || fabsf(_control_allocator_status.unallocated_torque[1]) >= 0.001f);

			if (roll_pitch_miss && !_astsmc_roll_pitch_miss_active) {
				_astsmc_roll_pitch_miss_count++;
				_astsmc_roll_pitch_miss_started = astsmc_status.timestamp;
				_astsmc_roll_pitch_miss_reported = false;
			}

			if (roll_pitch_miss && !headroom_was_requested) {
				_astsmc_roll_pitch_miss_started = astsmc_status.timestamp;
				_astsmc_roll_pitch_miss_reported = false;
			}

			if (roll_pitch_miss && headroom_was_requested && !_astsmc_roll_pitch_miss_reported
			    && astsmc_status.timestamp - _astsmc_roll_pitch_miss_started >= 250_ms) {
				events::send<float, float>(events::ID("mc_rate_control_astsmc_roll_pitch_allocation_miss"),
					{events::Log::Info, events::LogInternal::Info},
					"ASTSMC roll/pitch allocation miss: requested={1:.2}, achieved={2:.2}",
					requested_roll_pitch_norm, allocated_roll_pitch_norm);
				_astsmc_roll_pitch_miss_reported = true;
			}

			_astsmc_roll_pitch_miss_active = roll_pitch_miss;
			astsmc_allocator_status.event_count[5] = _astsmc_roll_pitch_miss_count;

			if (_control_allocator_feedback_valid) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_RECEIVED;
			}

			if (_astsmc_allocator_feedback_usable) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_USABLE;
			}

			if (_control_allocator_status.torque_setpoint_achieved) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_TORQUE_ACHIEVED;
			}

			if (_control_allocator_status.thrust_setpoint_achieved) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_THRUST_ACHIEVED;
			}

			if (_astsmc_allocator_actuator_bound) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_ACTUATOR_BOUND;
			}

			if (_rate_control.astsmcYawAuthorityBackoffActive()) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_BACKOFF_ACTIVE;
			}

			if (_rate_control.astsmcYawAuthorityReleaseActive()) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_RELEASE_ACTIVE;
			}

			if (headroom_was_requested) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_HEADROOM_REQUESTED;
			}

			if (headroom_was_applied) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_HEADROOM_APPLIED;
			}

			if (_rate_control.astsmcAllocatorConditioningActive(0)) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_ROLL_CONDITIONING;
			}

			if (_rate_control.astsmcAllocatorConditioningActive(1)) {
				astsmc_allocator_status.state_flags |= astsmc_allocator_status_s::STATE_PITCH_CONDITIONING;
			}

			_astsmc_allocator_status_pub.publish(astsmc_allocator_status);
			_astsmc_safety_status_pub.publish(astsmc_safety_status);
			_astsmc_status_pub.publish(astsmc_status);
		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The selected controller is PID, constrained finite-horizon MPC, model-based SMC,
or proper-implicit super-twisting SMC.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(MulticopterRateControl::desc, argc, argv);
}
