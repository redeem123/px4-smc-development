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

bool RateControl::setControllerType(int type)
{
	if (type < 0 || type > 2 || (type == 2 && !_msmc_model_valid)) {
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
			}
		}
	}

	_controller_type = type;
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
			     const float dt, const bool landed)
{
	Vector3f torque;

	if (_controller_type == 1) {
		torque = updateMPC(rate, rate_sp, dt, landed);

	} else if (_controller_type == 2) {
		torque = updateModelBasedSMC(rate, rate_sp, dt, landed);

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
	const Vector3f gyro_compensation = rate.cross(angular_momentum);

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

		// Euler rigid-body model: J*w_dot + w x (J*w) = effectiveness*u.
		const float reaching_rate = rate_sp_dot + _msmc_c(i) * rate_error(i) + _msmc_eta(i) * sat_s
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
	return _smc_torque_limited;
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
