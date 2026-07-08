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
Vector3f sanitizeNonNegativeVector(const Vector3f &value)
{
	Vector3f sanitized;

	for (int i = 0; i < 3; i++) {
		sanitized(i) = PX4_ISFINITE(value(i)) ? math::max(value(i), 0.f) : 0.f;
	}

	return sanitized;
}

bool changed(const Vector3f &previous, const Vector3f &next)
{
	return (previous - next).abs().max() > FLT_EPSILON;
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

Vector3f sanitizeNormalizedTorqueLimit(const Vector3f &value)
{
	Vector3f sanitized;

	for (int i = 0; i < 3; i++) {
		sanitized(i) = PX4_ISFINITE(value(i)) ? math::constrain(value(i), 0.f, 1.f) : 1.f;
	}

	return sanitized;
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

void RateControl::setControllerType(int type)
{
	if (_controller_type != type) {
		resetIntegral();
	}

	_controller_type = type;
}

void RateControl::setModelBasedSmcGains(const Vector3f &inertia, const Vector3f &c, const Vector3f &eta,
					const Vector3f &bnd, const Vector3f &ks, float rate_sp_derivative_limit)
{
	const Vector3f sanitized_inertia = sanitizeNonNegativeVector(inertia);
	const Vector3f sanitized_c = sanitizeNonNegativeVector(c);
	const Vector3f sanitized_eta = sanitizeNonNegativeVector(eta);
	const Vector3f sanitized_bnd = sanitizeNonNegativeVector(bnd);
	const Vector3f sanitized_ks = sanitizeNonNegativeVector(ks);
	const float sanitized_rate_sp_derivative_limit = sanitizeNonNegative(rate_sp_derivative_limit);

	const bool gains_changed = changed(_msmc_inertia, sanitized_inertia)
				   || changed(_msmc_c, sanitized_c)
				   || changed(_msmc_eta, sanitized_eta)
				   || changed(_msmc_bnd, sanitized_bnd)
				   || changed(_msmc_ks, sanitized_ks)
				   || fabsf(_msmc_rate_sp_derivative_limit - sanitized_rate_sp_derivative_limit) > FLT_EPSILON;

	_msmc_inertia = sanitized_inertia;
	_msmc_c = sanitized_c;
	_msmc_eta = sanitized_eta;
	_msmc_bnd = sanitized_bnd;
	_msmc_ks = sanitized_ks;
	_msmc_rate_sp_derivative_limit = sanitized_rate_sp_derivative_limit;

	if (gains_changed) {
		resetSmcState();
	}
}

void RateControl::setSMCSafeguards(float cutoff, float slew)
{
	const float sanitized_cutoff = sanitizeNonNegative(cutoff);
	const float sanitized_slew = sanitizeNonNegative(slew);

	const bool safeguards_changed = fabsf(_smc_lpf_cutoff - sanitized_cutoff) > FLT_EPSILON
					|| fabsf(_smc_slew_max - sanitized_slew) > FLT_EPSILON;

	_smc_lpf_cutoff = sanitized_cutoff;
	_smc_slew_max = sanitized_slew;

	if (safeguards_changed) {
		resetSmcState();
	}
}

void RateControl::setMpcGains(const Vector3f &inertia, const Vector3f &rate_weight,
			      const Vector3f &control_effectiveness, const Vector3f &torque_weight,
			      const Vector3f &torque_rate_weight, int horizon, float torque_slew_rate)
{
	const Vector3f sanitized_inertia = sanitizeNonNegativeVector(inertia);
	const Vector3f sanitized_rate_weight = sanitizeNonNegativeVector(rate_weight);
	const Vector3f sanitized_control_effectiveness = sanitizeNonNegativeVector(control_effectiveness);
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
		_mpc_last_torque.zero();
		_mpc_last_torque_valid = false;
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

void RateControl::setMpcTorqueLimit(const Vector3f &torque_limit)
{
	_mpc_torque_limit = sanitizeNormalizedTorqueLimit(torque_limit);
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

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
				     const float dt, const bool landed)
{
	if (_controller_type == 1) {
		return updateMPC(rate, rate_sp, dt, landed);
	}

	if (_controller_type == 2) {
		return updateModelBasedSMC(rate, rate_sp, dt, angular_accel, landed);
	}

	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

Vector3f RateControl::updateMPC(const Vector3f &rate, const Vector3f &rate_sp, const float dt, const bool landed)
{
	if (landed) {
		_mpc_rate_int.zero();
		_mpc_last_torque.zero();
		_mpc_last_rate_sp.zero();
		_mpc_rate_sp_prev_valid = false;
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
		const float inertia = PX4_ISFINITE(_mpc_inertia(axis)) ? _mpc_inertia(axis) : 0.f;
		const float control_effectiveness = PX4_ISFINITE(_mpc_control_effectiveness(axis)) ?
						    _mpc_control_effectiveness(axis) : 0.f;

		if (inertia <= FLT_EPSILON || control_effectiveness <= FLT_EPSILON
		    || !PX4_ISFINITE(rate(axis)) || !PX4_ISFINITE(rate_sp(axis)) || dt <= FLT_EPSILON) {
			torque(axis) = 0.f;
			_mpc_rate_int(axis) = 0.f;
			continue;
		}

		const float rate_error = rate_sp(axis) - rate(axis);
		const float torque_limit = math::constrain(_mpc_torque_limit(axis), 0.f, 1.f);
		float lower = -torque_limit;
		float upper = torque_limit;
		const float last_torque = _mpc_last_torque_valid ? _mpc_last_torque(axis) : 0.f;

		if (_mpc_torque_slew_rate > FLT_EPSILON && _mpc_last_torque_valid && !landed) {
			const float max_delta = _mpc_torque_slew_rate * dt;
			lower = math::max(lower, last_torque - max_delta);
			upper = math::min(upper, last_torque + max_delta);
		}

		lower = math::constrain(lower, -1.f, 1.f);
		upper = math::constrain(upper, lower, 1.f);

		const int horizon = math::constrain(_mpc_horizon, 1, MPC_MAX_HORIZON);
		const float input_gain = dt * control_effectiveness / inertia;
		const float gyro = PX4_ISFINITE(gyro_compensation(axis)) ? gyro_compensation(axis) : 0.f;
		const float model_offset = -_mpc_gyro_compensation_weight * (dt / inertia) * gyro;
		const float q = _mpc_rate_weight(axis);
		const float r = _mpc_torque_weight(axis);
		const float du = _mpc_torque_rate_weight(axis);
		const float input_regularization = math::max(r + du, 1e-6f);
		float cost_to_go = q;
		float feedback_gain = 0.f;

		for (int i = 0; i < horizon; i++) {
			const float denominator = input_regularization + input_gain * input_gain * cost_to_go;
			feedback_gain = denominator > FLT_EPSILON ? input_gain * cost_to_go / denominator : 0.f;
			cost_to_go = q + cost_to_go - input_gain * cost_to_go * feedback_gain;
		}

		float control = feedback_gain * rate_error + _mpc_rate_int(axis);

		if (fabsf(input_gain) > FLT_EPSILON) {
			control -= model_offset / input_gain;
			control += dt * rate_sp_derivative(axis) / input_gain;
		}

		if (du > FLT_EPSILON && _mpc_last_torque_valid) {
			const float tracking_weight = math::max(q * input_gain * input_gain, 1e-6f);
			control = (tracking_weight * control + du * last_torque) / (tracking_weight + du);
		}

		torque(axis) = math::constrain(control, lower, upper);

		if (!landed) {
			float integrator_error = rate_error;

			if (torque(axis) >= upper - FLT_EPSILON) {
				integrator_error = math::min(integrator_error, 0.f);
			}

			if (torque(axis) <= lower + FLT_EPSILON) {
				integrator_error = math::max(integrator_error, 0.f);
			}

			if (_mpc_integral_gain(axis) > FLT_EPSILON && _lim_int(axis) > FLT_EPSILON) {
				const float rate_i = _mpc_rate_int(axis) + _mpc_integral_gain(axis) * integrator_error * dt;

				if (PX4_ISFINITE(rate_i)) {
					_mpc_rate_int(axis) = math::constrain(rate_i, -_lim_int(axis), _lim_int(axis));
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
		const Vector3f &angular_accel, const bool landed)
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

	Vector3f torque;

	for (int i = 0; i < 3; i++) {
		// Sliding surface s_i = e_i + c_i * e_int_i
		float s_i = rate_error(i) + _msmc_c(i) * _smc_rate_int(i);

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

		const float inertia = PX4_ISFINITE(_msmc_inertia(i)) ? math::max(_msmc_inertia(i), 0.f) : 0.f;

		float rate_sp_dot = 0.f;

		if (_msmc_rate_sp_derivative_limit > FLT_EPSILON) {
			rate_sp_dot = math::constrain(rate_sp_derivative(i), -_msmc_rate_sp_derivative_limit,
						      _msmc_rate_sp_derivative_limit);
		}

		// Model-based SMC law in PX4 normalized torque units.
		const float reaching_rate = rate_sp_dot + _msmc_c(i) * rate_error(i) + _msmc_eta(i) * sat_s
					    + _msmc_ks(i) * s_filtered;
		torque(i) = _gain_ff(i) * rate_sp(i) + gyro_compensation(i) + inertia * reaching_rate
			    - _gain_d(i) * angular_accel(i);
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

		torque = constrainNormalizedTorque(torque);
		_smc_last_torque = torque;
	}

	return constrainNormalizedTorque(torque);
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

		float rate_i = _smc_rate_int(i) + err * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_smc_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
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
