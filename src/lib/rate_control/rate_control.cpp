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
 * @file rate_control.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{
static constexpr int MPC_MAX_HORIZON = 8;
static constexpr int MPC_MAX_CONSTRAINTS = 2 * MPC_MAX_HORIZON;

Vector3f sanitizeNonNegativeVector(const Vector3f &value)
{
	Vector3f sanitized;

	for (int i = 0; i < 3; i++) {
		sanitized(i) = PX4_ISFINITE(value(i)) ? math::max(value(i), 0.f) : 0.f;
	}

	return sanitized;
}

Vector3f retainPositiveVector(const Vector3f &value, const Vector3f &current, const Vector3f &fallback)
{
	Vector3f sanitized;

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(value(i)) && value(i) > FLT_EPSILON) {
			sanitized(i) = value(i);

		} else if (PX4_ISFINITE(current(i)) && current(i) > FLT_EPSILON) {
			sanitized(i) = current(i);

		} else {
			sanitized(i) = fallback(i);
		}
	}

	return sanitized;
}

bool changed(const Vector3f &previous, const Vector3f &next)
{
	return (previous - next).abs().max() > FLT_EPSILON;
}

bool allInRange(const Vector3f &value, float minimum, float maximum)
{
	for (int axis = 0; axis < 3; axis++) {
		if (!PX4_ISFINITE(value(axis)) || value(axis) < minimum || value(axis) > maximum) {
			return false;
		}
	}

	return true;
}

bool inRange(float value, float minimum, float maximum)
{
	return PX4_ISFINITE(value) && value >= minimum && value <= maximum;
}

float sanitizeNonNegative(float value)
{
	return PX4_ISFINITE(value) ? math::max(value, 0.f) : 0.f;
}

Vector3f constrainNormalizedTorque(const Vector3f &torque)
{
	Vector3f normalized_torque;

	for (int i = 0; i < 3; i++) {
		normalized_torque(i) = PX4_ISFINITE(torque(i)) ? math::constrain(torque(i), -1.f, 1.f) : 0.f;
	}

	return normalized_torque;
}

Vector3f retainNormalizedTorqueLimit(const Vector3f &value, const Vector3f &current)
{
	Vector3f sanitized;

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(value(i)) && value(i) > FLT_EPSILON) {
			sanitized(i) = math::constrain(value(i), FLT_EPSILON, 1.f);

		} else {
			sanitized(i) = math::constrain(current(i), FLT_EPSILON, 1.f);
		}
	}

	return sanitized;
}

bool choleskyDecomposition(const float matrix[MPC_MAX_HORIZON][MPC_MAX_HORIZON],
			   float lower[MPC_MAX_HORIZON][MPC_MAX_HORIZON], int size)
{
	for (int row = 0; row < size; row++) {
		for (int column = 0; column <= row; column++) {
			float value = matrix[row][column];

			for (int index = 0; index < column; index++) {
				value -= lower[row][index] * lower[column][index];
			}

			if (row == column) {
				if (!PX4_ISFINITE(value) || value <= 1e-9f) {
					return false;
				}

				lower[row][column] = sqrtf(value);

			} else {
				lower[row][column] = value / lower[column][column];
			}
		}
	}

	return true;
}

void solveCholesky(const float lower[MPC_MAX_HORIZON][MPC_MAX_HORIZON], const float right_hand_side[MPC_MAX_HORIZON],
		   float solution[MPC_MAX_HORIZON], int size)
{
	float intermediate[MPC_MAX_HORIZON] {};

	for (int row = 0; row < size; row++) {
		float value = right_hand_side[row];

		for (int column = 0; column < row; column++) {
			value -= lower[row][column] * intermediate[column];
		}

		intermediate[row] = value / lower[row][row];
	}

	for (int row = size - 1; row >= 0; row--) {
		float value = intermediate[row];

		for (int column = row + 1; column < size; column++) {
			value -= lower[column][row] * solution[column];
		}

		solution[row] = value / lower[row][row];
	}
}

bool solveMpcQuadraticProgram(const float response[MPC_MAX_HORIZON][MPC_MAX_HORIZON],
			      const float base_prediction[MPC_MAX_HORIZON], const float reference[MPC_MAX_HORIZON],
			      float rate_weight, float torque_weight, float torque_rate_weight, float previous_torque,
			      float torque_limit, float slew_delta, bool constrain_slew, int horizon,
			      float warm_start[MPC_MAX_HORIZON], float solution[MPC_MAX_HORIZON])
{
	float hessian[MPC_MAX_HORIZON][MPC_MAX_HORIZON] {};
	float gradient[MPC_MAX_HORIZON] {};

	for (int row = 0; row < horizon; row++) {
		for (int column = 0; column < horizon; column++) {
			float value = 0.f;

			for (int prediction = 0; prediction < horizon; prediction++) {
				value += response[prediction][row] * response[prediction][column];
			}

			hessian[row][column] = 2.f * rate_weight * value;
		}

		for (int prediction = 0; prediction < horizon; prediction++) {
			gradient[row] += 2.f * rate_weight * response[prediction][row]
					 * (base_prediction[prediction] - reference[prediction]);
		}

		hessian[row][row] += 2.f * math::max(torque_weight, 1e-6f);
	}

	if (torque_rate_weight > FLT_EPSILON) {
		hessian[0][0] += 2.f * torque_rate_weight;
		gradient[0] -= 2.f * torque_rate_weight * previous_torque;

		for (int step = 1; step < horizon; step++) {
			hessian[step][step] += 2.f * torque_rate_weight;
			hessian[step - 1][step - 1] += 2.f * torque_rate_weight;
			hessian[step][step - 1] -= 2.f * torque_rate_weight;
			hessian[step - 1][step] -= 2.f * torque_rate_weight;
		}
	}

	float constraint[MPC_MAX_CONSTRAINTS][MPC_MAX_HORIZON] {};
	float lower_bound[MPC_MAX_CONSTRAINTS] {};
	float upper_bound[MPC_MAX_CONSTRAINTS] {};

	for (int step = 0; step < horizon; step++) {
		constraint[step][step] = 1.f;
		lower_bound[step] = -torque_limit;
		upper_bound[step] = torque_limit;

		const int difference_row = horizon + step;
		constraint[difference_row][step] = 1.f;

		if (step == 0) {
			lower_bound[difference_row] = constrain_slew ? previous_torque - slew_delta : -2.f;
			upper_bound[difference_row] = constrain_slew ? previous_torque + slew_delta : 2.f;

		} else {
			constraint[difference_row][step - 1] = -1.f;
			lower_bound[difference_row] = constrain_slew ? -slew_delta : -2.f;
			upper_bound[difference_row] = constrain_slew ? slew_delta : 2.f;
		}
	}

	const int constraint_count = 2 * horizon;
	float mean_hessian_diagonal = 0.f;

	for (int step = 0; step < horizon; step++) {
		mean_hessian_diagonal += hessian[step][step];
	}

	mean_hessian_diagonal /= horizon;
	const float rho = math::constrain(0.1f * mean_hessian_diagonal, 1e-4f, 100.f);
	float system_matrix[MPC_MAX_HORIZON][MPC_MAX_HORIZON] {};

	for (int row = 0; row < horizon; row++) {
		for (int column = 0; column < horizon; column++) {
			system_matrix[row][column] = hessian[row][column];

			for (int constraint_index = 0; constraint_index < constraint_count; constraint_index++) {
				system_matrix[row][column] += rho * constraint[constraint_index][row]
							      * constraint[constraint_index][column];
			}
		}
	}

	float cholesky[MPC_MAX_HORIZON][MPC_MAX_HORIZON] {};

	if (!choleskyDecomposition(system_matrix, cholesky, horizon)) {
		return false;
	}

	float projected[MPC_MAX_CONSTRAINTS] {};
	float dual[MPC_MAX_CONSTRAINTS] {};

	for (int step = 0; step < horizon; step++) {
		solution[step] = PX4_ISFINITE(warm_start[step]) ? warm_start[step] : previous_torque;
	}

	for (int row = 0; row < constraint_count; row++) {
		float value = 0.f;

		for (int column = 0; column < horizon; column++) {
			value += constraint[row][column] * solution[column];
		}

		projected[row] = math::constrain(value, lower_bound[row], upper_bound[row]);
	}

	for (int iteration = 0; iteration < 12; iteration++) {
		float right_hand_side[MPC_MAX_HORIZON] {};

		for (int column = 0; column < horizon; column++) {
			right_hand_side[column] = -gradient[column];

			for (int row = 0; row < constraint_count; row++) {
				right_hand_side[column] += rho * constraint[row][column] * (projected[row] - dual[row]);
			}
		}

		solveCholesky(cholesky, right_hand_side, solution, horizon);

		for (int row = 0; row < constraint_count; row++) {
			float value = dual[row];

			for (int column = 0; column < horizon; column++) {
				value += constraint[row][column] * solution[column];
			}

			projected[row] = math::constrain(value, lower_bound[row], upper_bound[row]);
			dual[row] = value - projected[row];
		}
	}

	float preceding = previous_torque;

	for (int step = 0; step < horizon; step++) {
		solution[step] = projected[step];

		if (!PX4_ISFINITE(solution[step])) {
			return false;
		}

		float lower = -torque_limit;
		float upper = torque_limit;

		if (constrain_slew) {
			lower = math::max(lower, preceding - slew_delta);
			upper = math::min(upper, preceding + slew_delta);
		}

		solution[step] = math::constrain(solution[step], lower, upper);
		preceding = solution[step];
	}

	for (int step = 0; step < horizon; step++) {
		warm_start[step] = solution[math::min(step + 1, horizon - 1)];
	}

	return true;
}
}

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

void RateControl::resetAstsmcAllocatorFeasibility()
{
	const float base_authority = _astsmc_residual_torque_limit(2);

	if (_astsmc_allocator_feedback_usable) {
		_astsmc_yaw_authority_fallback_count++;
	}

	_astsmc_yaw_residual_authority_effective = base_authority;
	_astsmc_yaw_authority_healthy_dwell = 0.f;
	_astsmc_allocator_allocated_torque.zero();
	_astsmc_allocator_unallocated_torque.zero();
	_astsmc_allocator_conditioning_active.zero();
	_astsmc_allocator_feedback_usable = false;
	_astsmc_allocator_tightly_feasible = false;
	_astsmc_yaw_authority_backoff_active = false;
	_astsmc_yaw_authority_release_active = false;
}

void RateControl::setAstsmcAllocatorFeedback(const AstsmcAllocatorFeedback &feedback)
{
	const float base_authority = _astsmc_residual_torque_limit(2);
	const float maximum_authority = math::min(_astsmc_torque_limit(2), base_authority + _astsmc_yaw_residual_extension);

	if (feedback.usable && feedback.new_sample) {
		_astsmc_allocator_allocated_torque = feedback.allocated_torque;
		_astsmc_allocator_unallocated_torque = feedback.unallocated_torque;

	} else {
		_astsmc_allocator_allocated_torque.zero();
		_astsmc_allocator_unallocated_torque.zero();
	}

	if (_astsmc_yaw_residual_extension <= FLT_EPSILON) {
		_astsmc_yaw_residual_authority_effective = base_authority;
		_astsmc_yaw_authority_healthy_dwell = 0.f;
		_astsmc_allocator_feedback_usable = feedback.usable;
		_astsmc_allocator_tightly_feasible = false;
		_astsmc_yaw_authority_backoff_active = false;
		_astsmc_yaw_authority_release_active = false;
		return;
	}

	if (!feedback.usable) {
		if (_astsmc_allocator_feedback_usable) {
			_astsmc_yaw_authority_fallback_count++;
		}

		_astsmc_yaw_residual_authority_effective = base_authority;
		_astsmc_yaw_authority_healthy_dwell = 0.f;
		_astsmc_allocator_feedback_usable = false;
		_astsmc_allocator_tightly_feasible = false;
		_astsmc_yaw_authority_backoff_active = false;
		_astsmc_yaw_authority_release_active = false;
		return;
	}

	_astsmc_allocator_feedback_usable = true;

	if (!feedback.new_sample) {
		return;
	}

	constexpr float allocation_miss_threshold = 0.001f;
	constexpr float tightly_feasible_threshold = 0.0005f;
	const float residual_norm = feedback.unallocated_torque.norm();
	const bool allocation_miss = !feedback.torque_setpoint_achieved || !feedback.thrust_setpoint_achieved
				     || feedback.actuator_bound || residual_norm >= allocation_miss_threshold;
	const bool tightly_feasible = feedback.torque_setpoint_achieved && feedback.thrust_setpoint_achieved
				      && !feedback.actuator_bound && residual_norm < tightly_feasible_threshold;

	if (allocation_miss) {
		if (!_astsmc_yaw_authority_backoff_active) {
			_astsmc_yaw_authority_backoff_count++;
		}

		_astsmc_yaw_residual_authority_effective = base_authority;
		_astsmc_yaw_authority_healthy_dwell = 0.f;
		_astsmc_allocator_tightly_feasible = false;
		_astsmc_yaw_authority_backoff_active = true;
		_astsmc_yaw_authority_release_active = false;

	} else if (tightly_feasible) {
		_astsmc_allocator_tightly_feasible = true;
		_astsmc_yaw_authority_backoff_active = false;

	} else {
		_astsmc_yaw_authority_healthy_dwell = 0.f;
		_astsmc_allocator_tightly_feasible = false;
		_astsmc_yaw_authority_backoff_active = false;
		_astsmc_yaw_authority_release_active = false;
	}

	_astsmc_yaw_residual_authority_effective = math::constrain(
			_astsmc_yaw_residual_authority_effective, base_authority, maximum_authority);
}

bool RateControl::setControllerType(int type)
{
	if (type < 0 || type > 3 || (type == 2 && !_msmc_model_valid) || (type == 3 && !_astsmc_model_valid)) {
		return false;
	}

	if (_controller_type != type) {
		const Vector3f previous_output = _last_output;
		const bool previous_output_valid = _last_output_valid;
		resetIntegral();

		if (previous_output_valid) {
			if (type == 1) {
				_mpc_last_torque = previous_output;
				_mpc_actuator_state = previous_output;
				_mpc_last_torque_valid = true;

				for (int axis = 0; axis < 3; axis++) {
					for (int step = 0; step < MPC_MAX_HORIZON; step++) {
						_mpc_warm_start(axis, step) = previous_output(axis);
					}
				}

			} else if (type == 2) {
				_smc_last_torque = previous_output;

			} else if (type == 3) {
				_astsmc_seed_torque = previous_output;
				_astsmc_last_torque = previous_output;
				_astsmc_torque_raw = previous_output;
				_astsmc_torque_limited = previous_output;
				_astsmc_seed_pending = true;
			}
		}
	}

	_controller_type = type;
	return true;
}

void RateControl::resetSetpointHistory()
{
	resetModelBasedSmcSetpoint();
	resetMpcSetpoint();
	resetAstsmcReference(true);
}

void RateControl::resetSetpointHistoryForControlDisable()
{
	resetModelBasedSmcSetpoint();
	resetMpcSetpoint();
	resetAstsmcState(false);
	_last_output.zero();
	_last_output_valid = false;
}

bool RateControl::setAstsmcParameters(const Vector3f &inertia, const Vector3f &control_effectiveness,
				      const Vector3f &k1, const Vector3f &k2, const Vector3f &torque_limit,
				      const Vector3f &reference_acceleration_limit,
				      const Vector3f &reference_jerk_limit,
				      const Vector3f &residual_torque_limit,
				      const Vector3f &actuator_time_constant,
				      const Vector3f &torque_slew_rate,
				      const Vector3f &variation_weight,
				      const Vector3f &sliding_boundary,
				      float tracking_blend, float reference_feedforward,
				      float reference_feedforward_rp, float gyro_compensation,
				      float dt_min, float dt_max, float recovery_error_threshold,
				      float quiet_k1_error_threshold,
				      float selective_release_error_threshold, float selective_release_state_rate,
				      float trim_command_threshold, float trim_error_threshold,
				      float trim_acceleration_threshold, float trim_confidence_time,
				      float trim_time_constant, float roll_pitch_residual_extension,
				      float roll_pitch_k1_recovery_boost, float yaw_residual_extension)
{
	const bool valid = allInRange(inertia, 0.0001f, 5.f)
			   && allInRange(control_effectiveness, 0.0001f, 10.f)
			   && allInRange(k1, 0.01f, 100.f)
			   && allInRange(k2, 0.01f, 1000.f)
			   && allInRange(torque_limit, 0.01f, 1.f)
			   && allInRange(reference_acceleration_limit, 0.01f, 1000.f)
			   && allInRange(reference_jerk_limit, 0.01f, 10000.f)
			   && allInRange(residual_torque_limit, 0.001f, 1.f)
			   && allInRange(actuator_time_constant, 0.f, 0.2f)
			   && allInRange(torque_slew_rate, 0.f, 100.f)
			   && allInRange(variation_weight, 0.f, 100.f)
			   && allInRange(sliding_boundary, 0.f, 10.f)
			   && residual_torque_limit(0) <= torque_limit(0)
			   && residual_torque_limit(1) <= torque_limit(1)
			   && residual_torque_limit(2) <= torque_limit(2)
			   && inRange(tracking_blend, 0.f, 1.f)
			   && inRange(reference_feedforward, 0.f, 1.f)
			   && inRange(reference_feedforward_rp, 0.f, 1.f)
			   && inRange(gyro_compensation, 0.f, 1.f)
			   && inRange(dt_min, 0.000125f, 0.02f)
				   && inRange(dt_max, 0.0005f, 0.02f)
				   && dt_min <= dt_max
				   && inRange(recovery_error_threshold, 0.05f, 5.f)
				   && inRange(quiet_k1_error_threshold, 0.05f, recovery_error_threshold)
				   && inRange(selective_release_error_threshold, 0.05f, recovery_error_threshold)
				   && inRange(selective_release_state_rate, 0.f, 100.f)
				   && inRange(trim_command_threshold, 0.02f, 0.5f)
				   && inRange(trim_error_threshold, 0.02f, 0.5f)
				   && inRange(trim_acceleration_threshold, 0.1f, 10.f)
				   && inRange(trim_confidence_time, 0.1f, 5.f)
				   && inRange(trim_time_constant, 1.f, 100.f)
				   && inRange(roll_pitch_residual_extension, 0.f, 1.f)
				   && residual_torque_limit(0) + roll_pitch_residual_extension <= torque_limit(0)
				   && residual_torque_limit(1) + roll_pitch_residual_extension <= torque_limit(1)
				   && inRange(roll_pitch_k1_recovery_boost, 0.f, 10.f)
				   && (roll_pitch_k1_recovery_boost <= FLT_EPSILON
				       || quiet_k1_error_threshold < recovery_error_threshold)
				   && inRange(yaw_residual_extension, 0.f, 1.f)
				   && residual_torque_limit(2) + yaw_residual_extension <= torque_limit(2);

	if (!valid) {
		return false;
	}

	const bool parameters_changed = changed(_astsmc_inertia, inertia)
					|| changed(_astsmc_control_effectiveness, control_effectiveness)
					|| changed(_astsmc_k1, k1)
					|| changed(_astsmc_k2, k2)
					|| changed(_astsmc_torque_limit, torque_limit)
					|| changed(_astsmc_reference_acceleration_limit, reference_acceleration_limit)
					|| changed(_astsmc_reference_jerk_limit, reference_jerk_limit)
					|| changed(_astsmc_residual_torque_limit, residual_torque_limit)
					|| changed(_astsmc_actuator_time_constant, actuator_time_constant)
					|| changed(_astsmc_torque_slew_rate, torque_slew_rate)
					|| changed(_astsmc_variation_weight, variation_weight)
					|| changed(_astsmc_sliding_boundary, sliding_boundary)
					|| fabsf(_astsmc_tracking_blend - tracking_blend) > FLT_EPSILON
					|| fabsf(_astsmc_reference_feedforward - reference_feedforward) > FLT_EPSILON
					|| fabsf(_astsmc_reference_feedforward_rp - reference_feedforward_rp) > FLT_EPSILON
					|| fabsf(_astsmc_gyro_compensation - gyro_compensation) > FLT_EPSILON
					|| fabsf(_astsmc_dt_min - dt_min) > FLT_EPSILON
					|| fabsf(_astsmc_dt_max - dt_max) > FLT_EPSILON
					|| fabsf(_astsmc_recovery_error_threshold - recovery_error_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_quiet_k1_error_threshold - quiet_k1_error_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_selective_release_error_threshold - selective_release_error_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_selective_release_state_rate - selective_release_state_rate) > FLT_EPSILON
					|| fabsf(_astsmc_trim_command_threshold - trim_command_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_trim_error_threshold - trim_error_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_trim_acceleration_threshold - trim_acceleration_threshold) > FLT_EPSILON
					|| fabsf(_astsmc_trim_confidence_time - trim_confidence_time) > FLT_EPSILON
					|| fabsf(_astsmc_trim_time_constant - trim_time_constant) > FLT_EPSILON
					|| fabsf(_astsmc_roll_pitch_residual_extension - roll_pitch_residual_extension) > FLT_EPSILON
					|| fabsf(_astsmc_roll_pitch_k1_recovery_boost - roll_pitch_k1_recovery_boost) > FLT_EPSILON
					|| fabsf(_astsmc_yaw_residual_extension - yaw_residual_extension) > FLT_EPSILON;

	_astsmc_inertia = inertia;
	_astsmc_control_effectiveness = control_effectiveness;
	_astsmc_k1 = k1;
	_astsmc_k2 = k2;
	_astsmc_torque_limit = torque_limit;
	_astsmc_reference_acceleration_limit = reference_acceleration_limit;
	_astsmc_reference_jerk_limit = reference_jerk_limit;
	_astsmc_residual_torque_limit = residual_torque_limit;
	_astsmc_actuator_time_constant = actuator_time_constant;
	_astsmc_torque_slew_rate = torque_slew_rate;
	_astsmc_variation_weight = variation_weight;
	_astsmc_sliding_boundary = sliding_boundary;
	_astsmc_tracking_blend = tracking_blend;
	_astsmc_reference_feedforward = reference_feedforward;
	_astsmc_reference_feedforward_rp = reference_feedforward_rp;
	_astsmc_gyro_compensation = gyro_compensation;
	_astsmc_dt_min = dt_min;
	_astsmc_dt_max = dt_max;
	_astsmc_recovery_error_threshold = recovery_error_threshold;
	_astsmc_quiet_k1_error_threshold = quiet_k1_error_threshold;
	_astsmc_selective_release_error_threshold = selective_release_error_threshold;
	_astsmc_selective_release_state_rate = selective_release_state_rate;
	_astsmc_trim_command_threshold = trim_command_threshold;
	_astsmc_trim_error_threshold = trim_error_threshold;
	_astsmc_trim_acceleration_threshold = trim_acceleration_threshold;
	_astsmc_trim_confidence_time = trim_confidence_time;
	_astsmc_trim_time_constant = trim_time_constant;
	_astsmc_roll_pitch_residual_extension = roll_pitch_residual_extension;
	_astsmc_roll_pitch_k1_recovery_boost = roll_pitch_k1_recovery_boost;
	_astsmc_yaw_residual_extension = yaw_residual_extension;
	_astsmc_yaw_residual_authority_effective = math::constrain(
			_astsmc_yaw_residual_authority_effective, residual_torque_limit(2),
			residual_torque_limit(2) + yaw_residual_extension);

	for (int axis = 0; axis < 3; axis++) {
		_astsmc_reference[axis].setMaxAccel(reference_acceleration_limit(axis));
		_astsmc_reference[axis].setMaxJerk(reference_jerk_limit(axis));
		_astsmc_reference[axis].setMaxVel(FLT_MAX);
	}

	_astsmc_model_valid = true;

	if (parameters_changed) {
		const Vector3f previous_output = _last_output;
		const bool preserve_output = _controller_type == 3 && _last_output_valid;
		resetAstsmcState(false);

		if (preserve_output) {
			_astsmc_seed_torque = previous_output;
			_astsmc_last_torque = previous_output;
			_astsmc_torque_raw = previous_output;
			_astsmc_torque_limited = previous_output;
			_astsmc_actuator_state = previous_output;
			_astsmc_seed_pending = true;
		}
	}

	return true;
}

bool RateControl::setModelBasedSmcParameters(const Vector3f &inertia, const Vector3f &control_effectiveness,
		const Vector3f &c, const Vector3f &eta, const Vector3f &bnd, const Vector3f &ks,
		float rate_sp_derivative_limit, const Vector3f &integrator_limit, const Vector3f &torque_limit,
		float cutoff, float slew)
{
	const bool valid = allInRange(inertia, 0.0001f, 5.f)
			   && allInRange(control_effectiveness, 0.0001f, 10.f)
			   && allInRange(c, 0.1f, 10.f)
			   && allInRange(eta, 0.f, 10.f)
			   && allInRange(bnd, 0.01f, 1.f)
			   && allInRange(ks, 0.f, 5.f)
			   && inRange(rate_sp_derivative_limit, 0.f, 1000.f)
			   && allInRange(integrator_limit, 0.f, 5.f)
			   && allInRange(torque_limit, 0.01f, 1.f)
			   && inRange(cutoff, 0.f, 100.f)
			   && inRange(slew, 0.f, 100.f);

	if (!valid) {
		return false;
	}

	const bool parameters_changed = changed(_msmc_inertia, inertia)
					|| changed(_msmc_control_effectiveness, control_effectiveness)
					|| changed(_msmc_c, c)
					|| changed(_msmc_eta, eta)
					|| changed(_msmc_bnd, bnd)
					|| changed(_msmc_ks, ks)
					|| changed(_msmc_integral_limit, integrator_limit)
					|| changed(_msmc_torque_limit, torque_limit)
					|| fabsf(_msmc_rate_sp_derivative_limit - rate_sp_derivative_limit) > FLT_EPSILON
					|| fabsf(_smc_lpf_cutoff - cutoff) > FLT_EPSILON
					|| fabsf(_smc_slew_max - slew) > FLT_EPSILON;

	_msmc_inertia = inertia;
	_msmc_control_effectiveness = control_effectiveness;
	_msmc_c = c;
	_msmc_eta = eta;
	_msmc_bnd = bnd;
	_msmc_ks = ks;
	_msmc_rate_sp_derivative_limit = rate_sp_derivative_limit;
	_msmc_integral_limit = integrator_limit;
	_msmc_torque_limit = torque_limit;
	_smc_lpf_cutoff = cutoff;
	_smc_slew_max = slew;
	_msmc_model_valid = true;

	if (parameters_changed) {
		const Vector3f previous_output = _last_output;
		const bool preserve_output = _controller_type == 2 && _last_output_valid;
		resetSmcState();

		if (preserve_output) {
			_smc_last_torque = previous_output;
		}
	}

	return true;
}

bool RateControl::setModelBasedSmcPaperConstants(const Vector3f &attitude_surface_slope, float rotor_inertia)
{
	if (!allInRange(attitude_surface_slope, 0.f, 20.f) || !inRange(rotor_inertia, 0.f, 0.1f)) {
		return false;
	}

	_msmc_attitude_surface_slope = attitude_surface_slope;
	_msmc_rotor_inertia = rotor_inertia;
	return true;
}

void RateControl::setModelBasedSmcPaperState(float net_rotor_speed, bool net_rotor_speed_valid)
{
	_msmc_net_rotor_speed = net_rotor_speed;
	_msmc_net_rotor_speed_valid = net_rotor_speed_valid && PX4_ISFINITE(net_rotor_speed);
}

void RateControl::setMpcGains(const Vector3f &inertia, const Vector3f &rate_weight,
			      const Vector3f &control_effectiveness, const Vector3f &torque_weight,
			      const Vector3f &torque_rate_weight, int horizon, float torque_slew_rate)
{
	const Vector3f sanitized_inertia = retainPositiveVector(inertia, _mpc_inertia, Vector3f(0.01f, 0.01f, 0.02f));
	const Vector3f sanitized_rate_weight = sanitizeNonNegativeVector(rate_weight);
	const Vector3f sanitized_control_effectiveness = retainPositiveVector(control_effectiveness, _mpc_control_effectiveness,
			Vector3f(1.f, 1.f, 1.f));
	const Vector3f sanitized_torque_weight = sanitizeNonNegativeVector(torque_weight);
	const Vector3f sanitized_torque_rate_weight = sanitizeNonNegativeVector(torque_rate_weight);
	const int sanitized_horizon = math::constrain(horizon, 1, MPC_MAX_HORIZON);
	const float sanitized_torque_slew_rate = sanitizeNonNegative(torque_slew_rate);

	const bool params_changed = changed(_mpc_inertia, sanitized_inertia)
				    || changed(_mpc_rate_weight, sanitized_rate_weight)
				    || changed(_mpc_control_effectiveness, sanitized_control_effectiveness)
				    || changed(_mpc_torque_weight, sanitized_torque_weight)
				    || changed(_mpc_torque_rate_weight, sanitized_torque_rate_weight)
				    || _mpc_horizon != sanitized_horizon
				    || fabsf(_mpc_torque_slew_rate - sanitized_torque_slew_rate) > FLT_EPSILON;

	_mpc_inertia = sanitized_inertia;
	_mpc_rate_weight = sanitized_rate_weight;
	_mpc_control_effectiveness = sanitized_control_effectiveness;
	_mpc_torque_weight = sanitized_torque_weight;
	_mpc_torque_rate_weight = sanitized_torque_rate_weight;
	_mpc_horizon = sanitized_horizon;
	_mpc_torque_slew_rate = sanitized_torque_slew_rate;

	if (params_changed) {
		_mpc_warm_start.zero();
	}
}

void RateControl::setMpcIntegralGain(const Vector3f &I)
{
	const Vector3f sanitized_I = sanitizeNonNegativeVector(I);

	if (changed(_mpc_integral_gain, sanitized_I)) {
		_mpc_rate_int.zero();
	}

	_mpc_integral_gain = sanitized_I;
}

void RateControl::setMpcIntegralLimit(const Vector3f &integrator_limit)
{
	_mpc_integral_limit = sanitizeNonNegativeVector(integrator_limit);
}

void RateControl::setMpcTorqueLimit(const Vector3f &torque_limit)
{
	_mpc_torque_limit = retainNormalizedTorqueLimit(torque_limit, _mpc_torque_limit);
}

void RateControl::setMpcRateSetpointDerivativeLimit(float rate_sp_derivative_limit)
{
	const float sanitized_rate_sp_derivative_limit = sanitizeNonNegative(rate_sp_derivative_limit);

	if (fabsf(_mpc_rate_sp_derivative_limit - sanitized_rate_sp_derivative_limit) > FLT_EPSILON) {
		_mpc_last_rate_sp.zero();
		_mpc_rate_sp_prev_valid = false;
	}

	_mpc_rate_sp_derivative_limit = sanitized_rate_sp_derivative_limit;
}

void RateControl::setMpcGyroCompensation(float gyro_compensation_weight)
{
	_mpc_gyro_compensation_weight = PX4_ISFINITE(gyro_compensation_weight) ?
					math::constrain(gyro_compensation_weight, 0.f, 1.f) : 0.f;
}

void RateControl::setMpcActuatorTimeConstant(float time_constant)
{
	const float sanitized = PX4_ISFINITE(time_constant) ? math::constrain(time_constant, 0.f, 0.2f) :
				_mpc_actuator_time_constant;

	if (fabsf(_mpc_actuator_time_constant - sanitized) > FLT_EPSILON) {
		_mpc_actuator_time_constant = sanitized;
		_mpc_actuator_state = _mpc_last_torque_valid ? _mpc_last_torque : Vector3f();
		_mpc_warm_start.zero();
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed, const float raw_dt, const bool ground_containment)
{
	Vector3f torque;

	if (_controller_type == 1) {
		torque = updateMPC(rate, rate_sp, dt, landed);

	} else if (_controller_type == 2) {
		torque = updateModelBasedSMC(rate, rate_sp, dt, landed);

	} else if (_controller_type == 3) {
		torque = updateImplicitSuperTwisting(rate, rate_sp, dt, landed, raw_dt, ground_containment);

	} else {
		// angular rates error
		Vector3f rate_error = rate_sp - rate;

		// PID control with feed forward
		torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

		// update integral only if we are not landed
		if (!landed) {
			updateIntegral(rate_error, dt);
		}
	}

	if (!landed && torque.isAllFinite()) {
		_last_output = constrainNormalizedTorque(torque);
		_last_output_valid = true;

	} else if (landed) {
		_last_output_valid = false;
	}

	return torque;
}

Vector3f RateControl::updateMPC(const Vector3f &rate, const Vector3f &rate_sp, const float dt, const bool landed)
{
	if (landed) {
		_mpc_rate_int.zero();
		_mpc_last_torque.zero();
		_mpc_actuator_state.zero();
		_mpc_warm_start.zero();
		resetMpcSetpoint();
		_mpc_last_torque_valid = false;
	}

	Vector3f rate_sp_derivative{};

	if (!landed && _mpc_rate_sp_derivative_limit > FLT_EPSILON && _mpc_rate_sp_prev_valid && dt > FLT_EPSILON) {
		rate_sp_derivative = (rate_sp - _mpc_last_rate_sp) * (1.f / dt);

		for (int axis = 0; axis < 3; axis++) {
			rate_sp_derivative(axis) = math::constrain(rate_sp_derivative(axis),
						   -_mpc_rate_sp_derivative_limit, _mpc_rate_sp_derivative_limit);
		}
	}

	if (!landed) {
		_mpc_last_rate_sp = rate_sp;
		_mpc_rate_sp_prev_valid = true;
	}

	const Vector3f angular_momentum = _mpc_inertia.emult(rate);
	const Vector3f gyro_compensation = rate.cross(angular_momentum);
	Vector3f torque;

	for (int axis = 0; axis < 3; axis++) {
		const float inertia = _mpc_inertia(axis);
		const float control_effectiveness = _mpc_control_effectiveness(axis);

		if (inertia <= FLT_EPSILON || control_effectiveness <= FLT_EPSILON
		    || !PX4_ISFINITE(rate(axis)) || !PX4_ISFINITE(rate_sp(axis)) || dt <= FLT_EPSILON) {
			torque(axis) = _mpc_last_torque_valid ? _mpc_last_torque(axis) : 0.f;
			continue;
		}

		const float rate_error = rate_sp(axis) - rate(axis);
		const float torque_limit = math::constrain(_mpc_torque_limit(axis), FLT_EPSILON, 1.f);
		const float previous_torque = _mpc_last_torque_valid ? _mpc_last_torque(axis) : 0.f;
		const float last_torque = math::constrain(previous_torque, -torque_limit, torque_limit);
		const int horizon = math::constrain(_mpc_horizon, 1, MPC_MAX_HORIZON);
		const bool previous_torque_within_limit = fabsf(previous_torque) <= torque_limit + FLT_EPSILON;
		const bool constrain_slew = _mpc_torque_slew_rate > FLT_EPSILON && _mpc_last_torque_valid && !landed
					    && previous_torque_within_limit;
		const float slew_delta = constrain_slew ? _mpc_torque_slew_rate * dt : 0.f;
		float alpha = 0.f;
		float actuator_state_gain = 0.f;

		if (_mpc_actuator_time_constant > FLT_EPSILON) {
			alpha = expf(-dt / _mpc_actuator_time_constant);
			actuator_state_gain = _mpc_actuator_time_constant * (1.f - alpha);
		}

		const float command_gain = dt - actuator_state_gain;
		const float actuator_to_rate = control_effectiveness / inertia;

		if (_mpc_last_torque_valid && !landed) {
			_mpc_actuator_state(axis) = alpha * _mpc_actuator_state(axis) + (1.f - alpha) * previous_torque;
		}

		const float gyro = PX4_ISFINITE(gyro_compensation(axis)) ? gyro_compensation(axis) : 0.f;
		const float model_offset = -dt * (_mpc_gyro_compensation_weight * gyro
						  + control_effectiveness * _mpc_rate_int(axis)) / inertia;
		float base_prediction[MPC_MAX_HORIZON] {};
		float reference[MPC_MAX_HORIZON] {};
		float response[MPC_MAX_HORIZON][MPC_MAX_HORIZON] {};
		float predicted_rate = rate(axis);
		float predicted_actuator = _mpc_actuator_state(axis);

		for (int step = 0; step < horizon; step++) {
			predicted_rate += actuator_to_rate * actuator_state_gain * predicted_actuator + model_offset;
			predicted_actuator *= alpha;
			base_prediction[step] = predicted_rate;
			reference[step] = rate_sp(axis) + (step + 1) * dt * rate_sp_derivative(axis);
		}

		for (int input_step = 0; input_step < horizon; input_step++) {
			float impulse_rate = 0.f;
			float impulse_actuator = 0.f;

			for (int prediction_step = 0; prediction_step < horizon; prediction_step++) {
				const float command = prediction_step == input_step ? 1.f : 0.f;
				impulse_rate += actuator_to_rate * (actuator_state_gain * impulse_actuator + command_gain * command);
				impulse_actuator = alpha * impulse_actuator + (1.f - alpha) * command;
				response[prediction_step][input_step] = impulse_rate;
			}
		}

		float warm_start[MPC_MAX_HORIZON] {};
		float solution[MPC_MAX_HORIZON] {};

		for (int step = 0; step < horizon; step++) {
			warm_start[step] = _mpc_warm_start(axis, step);
		}

		const bool solved = solveMpcQuadraticProgram(response, base_prediction, reference, _mpc_rate_weight(axis),
				    _mpc_torque_weight(axis), _mpc_torque_rate_weight(axis), last_torque, torque_limit,
				    slew_delta, constrain_slew, horizon, warm_start, solution);

		if (!solved) {
			solution[0] = math::constrain(last_torque, -torque_limit, torque_limit);
		}

		torque(axis) = solution[0];

		for (int step = 0; step < horizon; step++) {
			_mpc_warm_start(axis, step) = solved ? warm_start[step] : torque(axis);
		}

		if (!landed) {
			float integrator_error = rate_error;

			if (_control_allocator_saturation_positive(axis) || torque(axis) >= torque_limit - FLT_EPSILON) {
				integrator_error = math::min(integrator_error, 0.f);
			}

			if (_control_allocator_saturation_negative(axis) || torque(axis) <= -torque_limit + FLT_EPSILON) {
				integrator_error = math::max(integrator_error, 0.f);
			}

			if (_mpc_integral_gain(axis) > FLT_EPSILON && _mpc_integral_limit(axis) > FLT_EPSILON) {
				const float rate_i = _mpc_rate_int(axis) + _mpc_integral_gain(axis) * integrator_error * dt;

				if (PX4_ISFINITE(rate_i)) {
					const float integral_limit = math::min(_mpc_integral_limit(axis), torque_limit);
					_mpc_rate_int(axis) = math::constrain(rate_i, -integral_limit, integral_limit);
				}

			} else {
				_mpc_rate_int(axis) = 0.f;
			}
		}
	}

	torque = constrainNormalizedTorque(torque);

	if (!landed) {
		_mpc_last_torque = torque;
		_mpc_last_torque_valid = true;
	}

	return torque;
}

Vector3f RateControl::updateModelBasedSMC(const Vector3f &rate, const Vector3f &rate_sp, const float dt,
		const bool landed)
{
	// angular rates error: e = rate_sp - rate
	const Vector3f rate_error = rate_sp - rate;

	Vector3f rate_sp_derivative{};

	// update integral only if we are not landed
	if (!landed) {
		if (_smc_rate_sp_prev_valid && dt > FLT_EPSILON && _msmc_rate_sp_derivative_limit > FLT_EPSILON) {
			rate_sp_derivative = (rate_sp - _smc_last_rate_sp) * (1.f / dt);
		}

		_smc_last_rate_sp = rate_sp;
		_smc_rate_sp_prev_valid = true;

		updateSMCIntegral(rate_error, dt);

	} else {
		_smc_rate_int.zero();
		_smc_s_filtered.zero();
		_smc_last_torque.zero();
		resetModelBasedSmcSetpoint();
	}

	const Vector3f angular_momentum = _msmc_inertia.emult(rate);
	Vector3f gyro_compensation = rate.cross(angular_momentum);

	// Propulsion-group gyroscopic torque of eq. 6: -J_R*q*Omega on roll and
	// +J_R*p*Omega on pitch. Cancelling it is the -a2*x4*Omega / -a4*x2*Omega
	// term of eq. 33. Yaw carries no rotor gyroscopic term.
	if (_msmc_rotor_inertia > FLT_EPSILON && _msmc_net_rotor_speed_valid) {
		const float rotor_momentum = _msmc_rotor_inertia * _msmc_net_rotor_speed;
		gyro_compensation += Vector3f(rotor_momentum * rate(1), -rotor_momentum * rate(0), 0.f);
	}

	_smc_surface.zero();
	_smc_torque_raw.zero();
	Vector3f torque;

	for (int i = 0; i < 3; i++) {
		const float inertia = _msmc_inertia(i);
		const float control_effectiveness = _msmc_control_effectiveness(i);

		if (inertia <= FLT_EPSILON || control_effectiveness <= FLT_EPSILON
		    || !PX4_ISFINITE(rate(i)) || !PX4_ISFINITE(rate_sp(i)) || dt <= FLT_EPSILON) {
			torque(i) = _smc_last_torque(i);
			continue;
		}

		// Sliding surface s_i = e_i + c_i * e_int_i
		const float s_i = rate_error(i) + _msmc_c(i) * _smc_rate_int(i);
		_smc_surface(i) = s_i;

		// Apply low-pass filter on the sliding surface
		float s_filtered = s_i;

		if (_smc_lpf_cutoff > FLT_EPSILON && !landed) {
			const float tau = 1.f / (2.f * M_PI_F * _smc_lpf_cutoff);
			const float alpha = dt / (tau + dt);

			if (!PX4_ISFINITE(_smc_s_filtered(i))) {
				_smc_s_filtered(i) = s_i;
			}

			s_filtered = alpha * s_i + (1.f - alpha) * _smc_s_filtered(i);
			_smc_s_filtered(i) = s_filtered;

		} else if (!landed) {
			_smc_s_filtered(i) = s_i;
		}

		// Saturation function sat(s_filtered / phi_i)
		float sat_s = s_filtered;

		if (_msmc_bnd(i) > 1e-4f) {
			sat_s = math::constrain(s_filtered / _msmc_bnd(i), -1.f, 1.f);

		} else {
			sat_s = (s_filtered > 0.f) ? 1.f : ((s_filtered < 0.f) ? -1.f : 0.f);
		}

		float rate_sp_dot = 0.f;

		if (_msmc_rate_sp_derivative_limit > FLT_EPSILON) {
			rate_sp_dot = math::constrain(rate_sp_derivative(i), -_msmc_rate_sp_derivative_limit,
						      _msmc_rate_sp_derivative_limit);
		}

		// Attitude-surface term -alpha1^2*z1 of eq. 33/34, which follows from
		// differentiating the surface of eq. 30. The surface is only the s2 of
		// eq. 30 if rate_sp = x1d_dot + alpha1*z1, so z1 is taken as
		// rate_sp/alpha1 rather than measured: that identity then holds under
		// every attitude-loop nonlinearity, and the term reduces to
		// -alpha1*rate_sp. Zero slope disables it.
		float attitude_surface = 0.f;

		if (_msmc_attitude_surface_slope(i) > FLT_EPSILON) {
			attitude_surface = -_msmc_attitude_surface_slope(i) * rate_sp(i);
		}

		// Euler rigid-body model: J*w_dot + w x (J*w) = effectiveness*u.
		const float reaching_rate = rate_sp_dot + attitude_surface + _msmc_c(i) * rate_error(i) + _msmc_eta(i) * sat_s
					    + _msmc_ks(i) * s_filtered;
		const float gyro = PX4_ISFINITE(gyro_compensation(i)) ? gyro_compensation(i) : 0.f;
		const float physical_torque = gyro + inertia * reaching_rate;
		torque(i) = physical_torque / control_effectiveness;
		_smc_torque_raw(i) = torque(i);
	}

	// Apply torque output slew rate limiting if enabled
	if (!landed) {
		if (_smc_slew_max > FLT_EPSILON) {
			const float max_delta = _smc_slew_max * dt;

			for (int i = 0; i < 3; i++) {
				if (PX4_ISFINITE(_smc_last_torque(i))) {
					torque(i) = math::constrain(torque(i), _smc_last_torque(i) - max_delta, _smc_last_torque(i) + max_delta);
				}
			}
		}

		for (int i = 0; i < 3; i++) {
			torque(i) = math::constrain(torque(i), -_msmc_torque_limit(i), _msmc_torque_limit(i));
		}

		torque = constrainNormalizedTorque(torque);
		_smc_last_torque = torque;
	}

	for (int i = 0; i < 3; i++) {
		torque(i) = math::constrain(torque(i), -_msmc_torque_limit(i), _msmc_torque_limit(i));
	}

	_smc_torque_limited = constrainNormalizedTorque(torque);

	// Consume the paper state so a publisher that stops updating cannot keep
	// feeding a stale rotor speed into the next cycle.
	_msmc_net_rotor_speed_valid = false;

	return _smc_torque_limited;
}

RateControl::AstsmcScalarStep RateControl::implicitSuperTwistingStep(float sliding, float state, float k1, float k2,
		float reaching_limit, float dt)
{
	const float sliding_abs = fabsf(sliding);
	const float sliding_sign = sliding > 0.f ? 1.f : (sliding < 0.f ? -1.f : 0.f);
	const float lambda = k2 - 0.25f * k1 * k1;
	float reaching_offset;

	if (sliding_abs > k2 * dt * dt) {
		const float radicand = math::max(sliding_abs - lambda * dt * dt, 0.f);
		reaching_offset = (2.f * lambda * dt + k1 * sqrtf(radicand)) * sliding_sign;

	} else {
		reaching_offset = 2.f * sliding / dt;
	}

	const float reaching_raw = state + reaching_offset;
	const float reaching_limited = math::constrain(reaching_raw, -reaching_limit, reaching_limit);
	const float state_difference = reaching_limited - state;
	const float conditioning_width = 2.f * k2 * dt;
	float state_next;

	if (fabsf(state_difference) > conditioning_width) {
		const float difference_sign = state_difference > 0.f ? 1.f : -1.f;
		state_next = state + dt * k2 * difference_sign;

	} else {
		state_next = 0.5f * (state + reaching_limited);
	}

	return {reaching_raw, reaching_limited, state_next};
}

float RateControl::scheduledAstsmcK1(float configured_k1, float sliding, float quiet_error_threshold,
		bool schedule_enabled, bool deep_quiet_enabled, float recovery_boost,
		float recovery_error_threshold)
{
	if (!schedule_enabled || quiet_error_threshold <= FLT_EPSILON) {
		return configured_k1;
	}

	constexpr float quiet_k1_ratio = 2.f / 3.f;
	constexpr float deep_quiet_k1_ratio = 1.f / 3.f;
	constexpr float quiet_plateau_ratio = 0.5f;
	constexpr float deep_quiet_plateau_ratio = 0.2f;
	constexpr float deep_quiet_exit_ratio = 0.4f;
	const float quiet_k1 = quiet_k1_ratio * configured_k1;
	const float normalized_error = fabsf(sliding) / quiet_error_threshold;
	float scheduled_k1 = quiet_k1;

	if (normalized_error > quiet_plateau_ratio) {
		const float k1_blend = math::constrain(
				(normalized_error - quiet_plateau_ratio) / (1.f - quiet_plateau_ratio), 0.f, 1.f);
		scheduled_k1 = quiet_k1 + k1_blend * (configured_k1 - quiet_k1);
	}

	if (deep_quiet_enabled && normalized_error < deep_quiet_exit_ratio) {
		const float deep_quiet_k1 = deep_quiet_k1_ratio * configured_k1;
		const float quiet_blend = math::constrain(
				(normalized_error - deep_quiet_plateau_ratio)
				/ (deep_quiet_exit_ratio - deep_quiet_plateau_ratio), 0.f, 1.f);
		scheduled_k1 = deep_quiet_k1 + quiet_blend * (quiet_k1 - deep_quiet_k1);
	}

	if (recovery_boost > FLT_EPSILON && recovery_error_threshold > quiet_error_threshold) {
		const float recovery_blend = math::constrain(
				(fabsf(sliding) - quiet_error_threshold)
				/ (recovery_error_threshold - quiet_error_threshold), 0.f, 1.f);
		scheduled_k1 += recovery_blend * recovery_boost;
	}

	return scheduled_k1;
}

float RateControl::astsmcRollPitchResidualAuthority(float base_authority, float extension,
		float sliding, float quiet_error_threshold)
{
	if (extension <= FLT_EPSILON || quiet_error_threshold <= FLT_EPSILON) {
		return base_authority;
	}

	const float release = math::constrain(fabsf(sliding) / quiet_error_threshold, 0.f, 1.f);
	return base_authority + release * extension;
}

float RateControl::variationRegularizedAppliedTorque(float desired_torque, float previous_torque,
		float residual_authority, float variation_weight)
{
	if (variation_weight <= FLT_EPSILON || residual_authority <= FLT_EPSILON) {
		return desired_torque;
	}

	const float variation_release = math::constrain(
			fabsf(desired_torque - previous_torque) / residual_authority, 0.f, 1.f);
	const float effective_weight = variation_weight * (1.f - variation_release) * (1.f - variation_release);
	return (desired_torque + effective_weight * previous_torque) / (1.f + effective_weight);
}

Vector3f RateControl::updateImplicitSuperTwisting(const Vector3f &rate, const Vector3f &rate_sp, const float dt,
		const bool landed, const float raw_dt, const bool ground_containment)
{
	_astsmc_rate_sp_raw = rate_sp;
	_astsmc_raw_dt = PX4_ISFINITE(raw_dt) ? raw_dt : dt;
	_astsmc_accepted_dt = dt;
	_astsmc_dt_valid = PX4_ISFINITE(_astsmc_raw_dt) && _astsmc_raw_dt >= _astsmc_dt_min
			   && _astsmc_raw_dt <= _astsmc_dt_max && PX4_ISFINITE(dt) && dt > FLT_EPSILON
			   && rate.isAllFinite() && rate_sp.isAllFinite();

	if (landed) {
		const bool count_reference_reset = _astsmc_reference_valid || _astsmc_reference_reset_pending;
		_astsmc_dt_valid = false;
		resetAstsmcState(false, count_reference_reset);
		return Vector3f();
	}

	if (ground_containment) {
		if (!_astsmc_ground_containment_active) {
			_astsmc_ground_containment_count++;
		}

		_astsmc_ground_containment_active = true;
		_astsmc_dt_valid = false;
		resetAstsmcState(false, true);
		_astsmc_ground_containment_active = true;
		return Vector3f();
	}

	_astsmc_ground_containment_active = false;
	_astsmc_allocator_conditioning_active.zero();

	_astsmc_max_airborne_dt = math::max(_astsmc_max_airborne_dt,
				 PX4_ISFINITE(_astsmc_raw_dt) ? _astsmc_raw_dt : 0.f);

	if (!_astsmc_dt_valid) {
		_astsmc_invalid_dt_hold_count++;

		if (_astsmc_previous_airborne_update_invalid) {
			_astsmc_consecutive_invalid_dt_hold_count++;
		}

		_astsmc_previous_airborne_update_invalid = true;
		_astsmc_accepted_dt = NAN;
		return _astsmc_last_torque;
	}

	_astsmc_previous_airborne_update_invalid = false;
	_astsmc_valid_update_count++;

	if (_astsmc_yaw_residual_extension > FLT_EPSILON && _astsmc_allocator_feedback_usable
	    && _astsmc_allocator_tightly_feasible) {
		constexpr float healthy_dwell_time = 0.1f;
		constexpr float release_time = 1.f;
		const float base_authority = _astsmc_residual_torque_limit(2);
		const float maximum_authority = math::min(_astsmc_torque_limit(2),
					 _astsmc_residual_torque_limit(2) + _astsmc_yaw_residual_extension);
		_astsmc_yaw_authority_healthy_dwell += dt;

		if (_astsmc_yaw_authority_healthy_dwell + FLT_EPSILON >= healthy_dwell_time) {
			const float release_rate = (maximum_authority - base_authority) / release_time;
			_astsmc_yaw_residual_authority_effective = math::min(
					_astsmc_yaw_residual_authority_effective + release_rate * dt, maximum_authority);
		}

		_astsmc_yaw_authority_release_active =
			_astsmc_yaw_residual_authority_effective > base_authority + FLT_EPSILON
			&& _astsmc_yaw_residual_authority_effective < maximum_authority - FLT_EPSILON;
	}

	if (!_astsmc_reference_valid) {
		for (int axis = 0; axis < 3; axis++) {
			_astsmc_reference[axis].reset(0.f, rate(axis), 0.f);
			_astsmc_rate_sp_shaped(axis) = rate(axis);
			_astsmc_reference_acceleration(axis) = 0.f;
			_astsmc_reference_jerk(axis) = 0.f;
		}

		_astsmc_reference_valid = true;

		if (_astsmc_reference_reset_pending) {
			_astsmc_reference_reset_count++;
			_astsmc_reference_reset_pending = false;
		}
	}

	const bool seed_pending = _astsmc_seed_pending;
	_astsmc_seed_pending = false;

	for (int axis = 0; axis < 3; axis++) {
		_astsmc_reference[axis].updateDurations(rate_sp(axis));
		_astsmc_reference[axis].updateTraj(dt);
		_astsmc_rate_sp_shaped(axis) = _astsmc_reference[axis].getCurrentVelocity();
		_astsmc_reference_acceleration(axis) = _astsmc_reference[axis].getCurrentAcceleration();
		_astsmc_reference_jerk(axis) = _astsmc_reference[axis].getCurrentJerk();
	}

	_astsmc_surface_raw = _astsmc_rate_sp_raw - rate;
	_astsmc_tracking_setpoint = _astsmc_rate_sp_shaped;

	for (int axis = 0; axis < 3; axis++) {
		const float reference_gap = _astsmc_rate_sp_raw(axis) - _astsmc_rate_sp_shaped(axis);
		const float boundary = _astsmc_sliding_boundary(axis);
		float tracking_correction = reference_gap;

		if (boundary > FLT_EPSILON) {
			tracking_correction = boundary * tanhf(reference_gap / boundary);
		}

		_astsmc_tracking_setpoint(axis) += _astsmc_tracking_blend * tracking_correction;
	}

	_astsmc_surface = _astsmc_tracking_setpoint - rate;
	const Vector3f angular_momentum = _astsmc_inertia.emult(rate);
	const Vector3f gyro_compensation = rate.cross(angular_momentum);

	for (int axis = 0; axis < 3; axis++) {
		const float inertia = _astsmc_inertia(axis);
		const float effectiveness = _astsmc_control_effectiveness(axis);
		const float control_gain = effectiveness / inertia;
		const bool allocation_miss = axis < 2 && _astsmc_allocator_feedback_usable
					     && fabsf(_astsmc_allocator_unallocated_torque(axis)) >= 0.001f;

		if (allocation_miss) {
			_astsmc_actuator_state(axis) = _astsmc_allocator_allocated_torque(axis);
		}

		const float residual_authority = axis == 2 ? _astsmc_yaw_residual_authority_effective
					 : astsmcRollPitchResidualAuthority(_astsmc_residual_torque_limit(axis),
						 _astsmc_roll_pitch_residual_extension, _astsmc_surface(axis),
						 _astsmc_quiet_k1_error_threshold);
		const float nominal_torque_limit = _astsmc_torque_limit(axis) - _astsmc_residual_torque_limit(axis);
		const float gyro = PX4_ISFINITE(gyro_compensation(axis)) ? gyro_compensation(axis) : 0.f;
		const float reference_feedforward = _astsmc_reference_feedforward
						    * (axis < 2 ? _astsmc_reference_feedforward_rp : 1.f);
		const float nominal_physical_torque = reference_feedforward * inertia
						      * _astsmc_reference_acceleration(axis)
						      + _astsmc_gyro_compensation * gyro;
		const float nominal_torque_raw = nominal_physical_torque / effectiveness;
		const float nominal_torque_limited = math::constrain(nominal_torque_raw,
						     -nominal_torque_limit, nominal_torque_limit);
		const float reaching_limit = control_gain * residual_authority;
		const float sliding = _astsmc_surface(axis);
		const bool deep_quiet_request = axis < 2 && _astsmc_trim_valid(axis)
						&& !_astsmc_selective_release_active(axis)
						&& !_astsmc_state_recovery_active(axis)
						&& fabsf(_astsmc_rate_sp_raw(axis)) <= _astsmc_trim_command_threshold
						&& fabsf(sliding) <= _astsmc_trim_error_threshold
						&& fabsf(_astsmc_reference_acceleration(axis))
						<= _astsmc_trim_acceleration_threshold;
		const float reaching_k1 = scheduledAstsmcK1(_astsmc_k1(axis), sliding,
					  _astsmc_quiet_k1_error_threshold, axis < 2, deep_quiet_request,
					  axis < 2 ? _astsmc_roll_pitch_k1_recovery_boost : 0.f,
					  _astsmc_recovery_error_threshold);

		// Equation (18) in the proper-implicit conditioned STA is written for x_dot = u + w.
		// Here s = rate_sp - rate gives s_dot = -G * torque + w, so reaching_input = -u.

		if (seed_pending) {
			const AstsmcScalarStep zero_state_step = implicitSuperTwistingStep(sliding, 0.f, reaching_k1,
								 _astsmc_k2(axis), reaching_limit, dt);
			const float seeded_residual_torque = math::constrain(
							_astsmc_seed_torque(axis) - nominal_torque_limited,
							-residual_authority,
							residual_authority);
			const float seeded_reaching_input = control_gain * seeded_residual_torque;
			_astsmc_integral_state(axis) = seeded_reaching_input - zero_state_step.reaching_input_raw;
		}

		AstsmcScalarStep implicit_step = implicitSuperTwistingStep(sliding, _astsmc_integral_state(axis),
							       reaching_k1, _astsmc_k2(axis), reaching_limit, dt);
		const AstsmcScalarStep zero_state_step = implicitSuperTwistingStep(sliding, 0.f, reaching_k1,
								 _astsmc_k2(axis), reaching_limit, dt);
		const AstsmcScalarStep trim_state_step = implicitSuperTwistingStep(sliding, _astsmc_trim_state(axis),
								 reaching_k1, _astsmc_k2(axis), reaching_limit, dt);
		const bool retained_state_wrong_direction = sliding * implicit_step.reaching_input_limited < -FLT_EPSILON;
		const bool trim_state_corrective = sliding * trim_state_step.reaching_input_limited > FLT_EPSILON;
		if (axis < 2 && !_astsmc_selective_release_active(axis)
		    && fabsf(_astsmc_rate_sp_raw(axis)) > _astsmc_trim_command_threshold) {
			_astsmc_selective_release_completed(axis) = false;
		}

		const bool selective_release_request = axis < 2 && _astsmc_trim_valid(axis)
						       && !_astsmc_selective_release_completed(axis)
						       && _astsmc_selective_release_state_rate > FLT_EPSILON
						       && fabsf(sliding) > _astsmc_selective_release_error_threshold
						       && retained_state_wrong_direction && trim_state_corrective;

		if (selective_release_request && !_astsmc_selective_release_active(axis)) {
			_astsmc_selective_release_count[axis]++;
			_astsmc_selective_trigger_sliding(axis) = sliding;
			_astsmc_selective_trigger_state(axis) = _astsmc_integral_state(axis);
			_astsmc_selective_trigger_trim(axis) = _astsmc_trim_state(axis);
			_astsmc_selective_trigger_reaching(axis) = implicit_step.reaching_input_limited;
			_astsmc_selective_release_active(axis) = true;
		}

		if (_astsmc_selective_release_active(axis)) {
			const float release_delta = _astsmc_selective_release_state_rate * dt;
			_astsmc_integral_state(axis) = math::constrain(_astsmc_trim_state(axis),
							       _astsmc_integral_state(axis) - release_delta,
							       _astsmc_integral_state(axis) + release_delta);
			// The trim-state step was already verified corrective at episode entry. Use it
			// immediately while the retained state moves to trim at the bounded release rate.
			implicit_step = trim_state_step;

			if (fabsf(_astsmc_integral_state(axis) - _astsmc_trim_state(axis)) <= release_delta) {
				_astsmc_integral_state(axis) = _astsmc_trim_state(axis);
				implicit_step = trim_state_step;
				_astsmc_selective_release_active(axis) = false;
				_astsmc_selective_release_completed(axis) = true;
			}
		}

		const bool large_error = fabsf(sliding) > _astsmc_recovery_error_threshold;
		const bool released_state_wrong_direction = sliding * implicit_step.reaching_input_limited < -FLT_EPSILON;
		const bool zero_state_corrective = sliding * zero_state_step.reaching_input_limited > FLT_EPSILON;
		const bool recover_state = large_error && released_state_wrong_direction && zero_state_corrective;

		if (recover_state) {
			if (!_astsmc_state_recovery_active(axis)) {
				_astsmc_state_recovery_count[axis]++;
				_astsmc_recovery_trigger_sliding(axis) = sliding;
				_astsmc_recovery_trigger_state(axis) = _astsmc_integral_state(axis);
				_astsmc_recovery_trigger_reaching(axis) = implicit_step.reaching_input_limited;
			}

			_astsmc_state_recovery_active(axis) = true;
			_astsmc_runtime_fault_latched = true;
			_astsmc_runtime_fault_reason |= astsmc_safety_status_s::RUNTIME_FAULT_STATE_RECOVERY;
			_astsmc_integral_state(axis) = 0.f;
			implicit_step = zero_state_step;

		} else {
			_astsmc_state_recovery_active(axis) = false;
		}

		const float reaching_raw = implicit_step.reaching_input_raw;
		const float desired_residual_torque = implicit_step.reaching_input_limited / control_gain;
		const float desired_applied_torque = nominal_torque_limited + desired_residual_torque;
		float command_lower = -_astsmc_torque_limit(axis);
		float command_upper = _astsmc_torque_limit(axis);

		if (_astsmc_torque_slew_rate(axis) > FLT_EPSILON) {
			const float command_delta = _astsmc_torque_slew_rate(axis) * dt;
			command_lower = math::max(command_lower, _astsmc_last_torque(axis) - command_delta);
			command_upper = math::min(command_upper, _astsmc_last_torque(axis) + command_delta);
		}

		const float alpha = _astsmc_actuator_time_constant(axis) > FLT_EPSILON ?
				    expf(-dt / _astsmc_actuator_time_constant(axis)) : 0.f;
		const float command_gain = 1.f - alpha;
		const float applied_lower = alpha * _astsmc_actuator_state(axis) + command_gain * command_lower;
		const float applied_upper = alpha * _astsmc_actuator_state(axis) + command_gain * command_upper;
		const float residual_lower = nominal_torque_limited - residual_authority;
		const float residual_upper = nominal_torque_limited + residual_authority;
		const float admissible_lower = math::max(applied_lower, residual_lower);
		const float admissible_upper = math::min(applied_upper, residual_upper);
		const float regularized_applied_torque = variationRegularizedAppliedTorque(
				desired_applied_torque, _astsmc_actuator_state(axis),
				residual_authority, _astsmc_variation_weight(axis));
		float applied_torque_target;

		if (admissible_lower <= admissible_upper) {
			applied_torque_target = math::constrain(regularized_applied_torque, admissible_lower, admissible_upper);

		} else {
			const float applied_midpoint = 0.5f * (applied_lower + applied_upper);
			applied_torque_target = applied_midpoint < residual_lower ? applied_upper : applied_lower;
		}

		float torque_command = command_gain > FLT_EPSILON ?
				       (applied_torque_target - alpha * _astsmc_actuator_state(axis)) / command_gain :
				       applied_torque_target;
		torque_command = math::constrain(torque_command, command_lower, command_upper);
		applied_torque_target = alpha * _astsmc_actuator_state(axis) + command_gain * torque_command;
		const float applied_reaching_input = control_gain * (applied_torque_target - nominal_torque_limited);
		float conditioning_reaching_input = applied_reaching_input;

		if (allocation_miss) {
			const float achieved_residual_torque = math::constrain(
					_astsmc_allocator_allocated_torque(axis) - _astsmc_nominal_torque_limited(axis),
					-residual_authority, residual_authority);
			conditioning_reaching_input = control_gain * achieved_residual_torque;
			_astsmc_allocator_conditioning_active(axis) = true;
		}

		const bool actuator_constraint = fabsf(applied_torque_target - regularized_applied_torque) > FLT_EPSILON;
		const bool internal_saturation = fabsf(reaching_raw - implicit_step.reaching_input_limited) > FLT_EPSILON
						 || actuator_constraint;
		const float state_difference = conditioning_reaching_input - _astsmc_integral_state(axis);
		const float conditioning_width = 2.f * _astsmc_k2(axis) * dt;
		const bool quiet_anchor_request = axis < 2 && _astsmc_trim_valid(axis)
						  && !_astsmc_selective_release_active(axis)
						  && !_astsmc_state_recovery_active(axis)
						  && fabsf(_astsmc_rate_sp_raw(axis)) <= _astsmc_trim_command_threshold
						  && fabsf(sliding) <= _astsmc_quiet_k1_error_threshold
						  && fabsf(_astsmc_reference_acceleration(axis)) <= _astsmc_trim_acceleration_threshold;

		if (quiet_anchor_request && !_astsmc_quiet_anchor_active(axis)) {
			_astsmc_quiet_anchor_count[axis]++;
		}

		_astsmc_quiet_anchor_active(axis) = quiet_anchor_request;
		_astsmc_deep_quiet_active(axis) = deep_quiet_request;
		float integral_state_next = _astsmc_integral_state(axis);

		if (quiet_anchor_request) {
			constexpr float quiet_anchor_k2_fraction = 0.5f;
			const float quiet_anchor_delta = quiet_anchor_k2_fraction * _astsmc_k2(axis) * dt;
			integral_state_next = math::constrain(_astsmc_trim_state(axis),
					      _astsmc_integral_state(axis) - quiet_anchor_delta,
					      _astsmc_integral_state(axis) + quiet_anchor_delta);

		} else if (!_astsmc_selective_release_active(axis)) {
			if (fabsf(state_difference) > conditioning_width) {
				integral_state_next = _astsmc_integral_state(axis)
						      + dt * _astsmc_k2(axis) * (state_difference > 0.f ? 1.f : -1.f);

			} else {
				integral_state_next = 0.5f * (_astsmc_integral_state(axis) + conditioning_reaching_input);
			}
		}

		_astsmc_reaching_input_raw(axis) = reaching_raw;
		_astsmc_reaching_input_limited(axis) = applied_reaching_input;
		_astsmc_nominal_torque_raw(axis) = nominal_torque_raw;
		_astsmc_nominal_torque_limited(axis) = nominal_torque_limited;
		_astsmc_residual_torque_raw(axis) = reaching_raw / control_gain;
		_astsmc_residual_torque_limited(axis) = torque_command - nominal_torque_limited;
		_astsmc_torque_raw(axis) = nominal_torque_raw + _astsmc_residual_torque_raw(axis);
		_astsmc_torque_limited(axis) = torque_command;
		_astsmc_actuator_state(axis) = applied_torque_target;
		_astsmc_applied_torque_target(axis) = applied_torque_target;
		_astsmc_command_lower(axis) = command_lower;
		_astsmc_command_upper(axis) = command_upper;
		_astsmc_variation_regularization(axis) = regularized_applied_torque - desired_applied_torque;
		_astsmc_actuator_constraint(axis) = actuator_constraint;
		_astsmc_internal_saturation(axis) = internal_saturation;
		_astsmc_nominal_saturation(axis) = fabsf(nominal_torque_raw - nominal_torque_limited) > FLT_EPSILON;

		if (_astsmc_internal_saturation(axis)) {
			_astsmc_residual_limit_count[axis]++;
		}

		if (_astsmc_nominal_saturation(axis)) {
			_astsmc_nominal_limit_count[axis]++;
		}

		if (!landed && PX4_ISFINITE(integral_state_next)) {
			_astsmc_integral_state(axis) = _astsmc_selective_release_active(axis)
						       ? math::constrain(_astsmc_trim_state(axis),
								 _astsmc_integral_state(axis), integral_state_next)
						       : integral_state_next;
		}

		if (axis < 2) {
			constexpr float trim_exit_scale = 1.5f;
			constexpr float trim_confidence_decay_rate = 0.5f;
			const bool trim_intervention = _astsmc_state_recovery_active(axis)
						       || _astsmc_selective_release_active(axis);
			const bool trim_neutral_enter = !_astsmc_internal_saturation(axis) && !trim_intervention
							&& fabsf(_astsmc_rate_sp_raw(axis)) <= _astsmc_trim_command_threshold
							&& fabsf(sliding) <= _astsmc_trim_error_threshold
							&& fabsf(_astsmc_reference_acceleration(axis))
							<= _astsmc_trim_acceleration_threshold;
			const bool trim_outside_exit = trim_intervention
						       || fabsf(_astsmc_rate_sp_raw(axis))
						       > trim_exit_scale * _astsmc_trim_command_threshold
						       || fabsf(sliding) > trim_exit_scale * _astsmc_trim_error_threshold
						       || fabsf(_astsmc_reference_acceleration(axis))
						       > trim_exit_scale * _astsmc_trim_acceleration_threshold;

			if (trim_neutral_enter) {
				_astsmc_trim_confidence(axis) = math::min(_astsmc_trim_confidence(axis) + dt,
							      _astsmc_trim_confidence_time);

			} else if (trim_outside_exit) {
				_astsmc_trim_confidence(axis) = math::max(_astsmc_trim_confidence(axis)
							      - trim_confidence_decay_rate * dt, 0.f);
			}

			if (!_astsmc_trim_valid(axis)
			    && _astsmc_trim_confidence(axis) >= _astsmc_trim_confidence_time) {
				_astsmc_trim_state(axis) = _astsmc_integral_state(axis);
				_astsmc_trim_valid(axis) = true;

			} else if (_astsmc_trim_valid(axis) && trim_neutral_enter) {
				const float trim_alpha = math::constrain(dt / _astsmc_trim_time_constant, 0.f, 1.f);
				_astsmc_trim_state(axis) += trim_alpha * (_astsmc_integral_state(axis) - _astsmc_trim_state(axis));
			}
		}
	}

	_astsmc_total_bound_valid = true;

	for (int axis = 0; axis < 3; axis++) {
		if (!PX4_ISFINITE(_astsmc_torque_limited(axis))
		    || fabsf(_astsmc_torque_limited(axis)) > _astsmc_torque_limit(axis) + 1e-5f) {
			_astsmc_total_bound_valid = false;
		}
	}

	if (!_astsmc_total_bound_valid) {
		_astsmc_total_bound_violation_count++;
		_astsmc_runtime_fault_latched = true;
		_astsmc_runtime_fault_reason |= astsmc_safety_status_s::RUNTIME_FAULT_TOTAL_BOUND;
		return _astsmc_last_torque;
	}

	_astsmc_last_torque = _astsmc_torque_limited;
	return _astsmc_torque_limited;
}

void RateControl::updateSMCIntegral(const Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		float err = rate_error(i);

		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			err = math::min(err, 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			err = math::max(err, 0.f);
		}

		if (_smc_last_torque(i) >= _msmc_torque_limit(i) - FLT_EPSILON) {
			err = math::min(err, 0.f);
		}

		if (_smc_last_torque(i) <= -_msmc_torque_limit(i) + FLT_EPSILON) {
			err = math::max(err, 0.f);
		}

		float rate_i = _smc_rate_int(i) + err * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_smc_rate_int(i) = math::constrain(rate_i, -_msmc_integral_limit(i), _msmc_integral_limit(i));
		}
	}
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.controller_type = static_cast<uint8_t>(_controller_type);
	rate_ctrl_status.model_valid = _msmc_model_valid;
	rate_ctrl_status.astsmc_valid = _astsmc_model_valid;

	for (int axis = 0; axis < 3; axis++) {
		rate_ctrl_status.smc_surface[axis] = _smc_surface(axis);
		rate_ctrl_status.smc_surface_filtered[axis] = _smc_s_filtered(axis);
		rate_ctrl_status.smc_torque_raw[axis] = _smc_torque_raw(axis);
		rate_ctrl_status.smc_torque_limited[axis] = _smc_torque_limited(axis);
	}

	if (_controller_type == 1) {
		rate_ctrl_status.rollspeed_integ = _mpc_rate_int(0);
		rate_ctrl_status.pitchspeed_integ = _mpc_rate_int(1);
		rate_ctrl_status.yawspeed_integ = _mpc_rate_int(2);

	} else if (_controller_type == 2) {
		rate_ctrl_status.rollspeed_integ = _smc_rate_int(0);
		rate_ctrl_status.pitchspeed_integ = _smc_rate_int(1);
		rate_ctrl_status.yawspeed_integ = _smc_rate_int(2);

	} else {
		rate_ctrl_status.rollspeed_integ = _rate_int(0);
		rate_ctrl_status.pitchspeed_integ = _rate_int(1);
		rate_ctrl_status.yawspeed_integ = _rate_int(2);
	}
}

void RateControl::getAstsmcStatus(astsmc_status_s &astsmc_status) const
{
	astsmc_status.configuration_valid = _astsmc_model_valid;
	astsmc_status.dt_valid = _astsmc_dt_valid;
	astsmc_status.total_bound_valid = _astsmc_total_bound_valid;
	astsmc_status.raw_dt = _astsmc_raw_dt;
	astsmc_status.accepted_dt = _astsmc_accepted_dt;
	astsmc_status.dt_min = _astsmc_dt_min;
	astsmc_status.dt_max = _astsmc_dt_max;
	astsmc_status.max_airborne_dt = _astsmc_max_airborne_dt;
	astsmc_status.valid_update_count = _astsmc_valid_update_count;
	astsmc_status.invalid_dt_hold_count = _astsmc_invalid_dt_hold_count;
	astsmc_status.consecutive_invalid_dt_hold_count = _astsmc_consecutive_invalid_dt_hold_count;
	astsmc_status.reference_reset_count = _astsmc_reference_reset_count;
	astsmc_status.total_bound_violation_count = _astsmc_total_bound_violation_count;

	for (int axis = 0; axis < 3; axis++) {
		astsmc_status.rate_setpoint_raw[axis] = _astsmc_rate_sp_raw(axis);
		astsmc_status.rate_setpoint_shaped[axis] = _astsmc_rate_sp_shaped(axis);
		astsmc_status.reference_acceleration[axis] = _astsmc_reference_acceleration(axis);
		astsmc_status.reference_jerk[axis] = _astsmc_reference_jerk(axis);
		astsmc_status.internal_saturation[axis] = _astsmc_internal_saturation(axis);
		astsmc_status.nominal_saturation[axis] = _astsmc_nominal_saturation(axis);
		astsmc_status.sliding_variable_raw[axis] = _astsmc_surface_raw(axis);
		astsmc_status.sliding_variable[axis] = _astsmc_surface(axis);
		astsmc_status.integral_state[axis] = _astsmc_integral_state(axis);
		astsmc_status.k1[axis] = _astsmc_k1(axis);
		astsmc_status.k2[axis] = _astsmc_k2(axis);
		astsmc_status.reaching_input_raw[axis] = _astsmc_reaching_input_raw(axis);
		astsmc_status.reaching_input_limited[axis] = _astsmc_reaching_input_limited(axis);
		astsmc_status.nominal_torque_raw[axis] = _astsmc_nominal_torque_raw(axis);
		astsmc_status.nominal_torque_limited[axis] = _astsmc_nominal_torque_limited(axis);
		astsmc_status.residual_torque_raw[axis] = _astsmc_residual_torque_raw(axis);
		astsmc_status.residual_torque_limited[axis] = _astsmc_residual_torque_limited(axis);
		astsmc_status.residual_authority[axis] = axis == 2 ? _astsmc_yaw_residual_authority_effective
						       : astsmcRollPitchResidualAuthority(
							       _astsmc_residual_torque_limit(axis),
							       _astsmc_roll_pitch_residual_extension,
							       _astsmc_surface(axis), _astsmc_quiet_k1_error_threshold);
		astsmc_status.actuator_time_constant[axis] = _astsmc_actuator_time_constant(axis);
		astsmc_status.torque_slew_rate[axis] = _astsmc_torque_slew_rate(axis);
		astsmc_status.variation_weight[axis] = _astsmc_variation_weight(axis);
		astsmc_status.variation_regularization[axis] = _astsmc_variation_regularization(axis);
		astsmc_status.actuator_state[axis] = _astsmc_actuator_state(axis);
		astsmc_status.applied_torque_target[axis] = _astsmc_applied_torque_target(axis);
		astsmc_status.command_lower[axis] = _astsmc_command_lower(axis);
		astsmc_status.command_upper[axis] = _astsmc_command_upper(axis);
		astsmc_status.actuator_constraint[axis] = _astsmc_actuator_constraint(axis);
		astsmc_status.nominal_limit_count[axis] = _astsmc_nominal_limit_count[axis];
		astsmc_status.residual_limit_count[axis] = _astsmc_residual_limit_count[axis];
		astsmc_status.torque_raw[axis] = _astsmc_torque_raw(axis);
		astsmc_status.torque_limited[axis] = _astsmc_torque_limited(axis);
	}
}

void RateControl::getAstsmcSafetyStatus(astsmc_safety_status_s &astsmc_safety_status) const
{
	astsmc_safety_status.runtime_fault_latched = _astsmc_runtime_fault_latched;
	astsmc_safety_status.runtime_fault_reason = _astsmc_runtime_fault_reason;
	astsmc_safety_status.ground_containment_active = _astsmc_ground_containment_active;
	astsmc_safety_status.ground_containment_count = _astsmc_ground_containment_count;

	for (int axis = 0; axis < 3; axis++) {
		astsmc_safety_status.state_recovery_active[axis] = _astsmc_state_recovery_active(axis);
		astsmc_safety_status.state_recovery_count[axis] = _astsmc_state_recovery_count[axis];
		astsmc_safety_status.recovery_trigger_sliding[axis] = _astsmc_recovery_trigger_sliding(axis);
		astsmc_safety_status.recovery_trigger_state[axis] = _astsmc_recovery_trigger_state(axis);
		astsmc_safety_status.recovery_trigger_reaching[axis] = _astsmc_recovery_trigger_reaching(axis);

		if (axis < 2) {
			astsmc_safety_status.selective_release_active[axis] = _astsmc_selective_release_active(axis);
			astsmc_safety_status.quiet_anchor_active[axis] = _astsmc_quiet_anchor_active(axis);
			astsmc_safety_status.deep_quiet_active[axis] = _astsmc_deep_quiet_active(axis);
			astsmc_safety_status.trim_valid[axis] = _astsmc_trim_valid(axis);
			astsmc_safety_status.selective_release_count[axis] = _astsmc_selective_release_count[axis];
			astsmc_safety_status.quiet_anchor_count[axis] = _astsmc_quiet_anchor_count[axis];
			astsmc_safety_status.selective_trigger_sliding[axis] = _astsmc_selective_trigger_sliding(axis);
			astsmc_safety_status.selective_trigger_state[axis] = _astsmc_selective_trigger_state(axis);
			astsmc_safety_status.selective_trigger_trim[axis] = _astsmc_selective_trigger_trim(axis);
			astsmc_safety_status.selective_trigger_reaching[axis] = _astsmc_selective_trigger_reaching(axis);
			astsmc_safety_status.trim_state[axis] = _astsmc_trim_state(axis);
			astsmc_safety_status.trim_confidence[axis] = _astsmc_trim_confidence(axis);
		}
	}
}
