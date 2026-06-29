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

void RateControl::setSmcGains(const Vector3f &c, const Vector3f &eta, const Vector3f &bnd,
			 const Vector3f &ks, const Vector3f &keq)
{
	_smc_c = c;
	_smc_eta = eta;
	_smc_bnd = bnd;
	_smc_ks = ks;
	_smc_keq = keq;
}

void RateControl::setModelBasedSmcGains(const Vector3f &inertia, const Vector3f &c, const Vector3f &eta,
					const Vector3f &bnd, const Vector3f &ks, float rate_sp_derivative_limit)
{
	_msmc_inertia = inertia;
	_msmc_c = c;
	_msmc_eta = eta;
	_msmc_bnd = bnd;
	_msmc_ks = ks;
	_msmc_rate_sp_derivative_limit = 0.f;

	if (PX4_ISFINITE(rate_sp_derivative_limit)) {
		_msmc_rate_sp_derivative_limit = math::max(rate_sp_derivative_limit, 0.f);
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
				     const float dt, const bool landed)
{
	if (_controller_type == 1) {
		return updateSMC(rate, rate_sp, dt, landed);
	}

	if (_controller_type == 2) {
		return updateModelBasedSMC(rate, rate_sp, dt, landed);
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

Vector3f RateControl::updateModelBasedSMC(const Vector3f &rate, const Vector3f &rate_sp, const float dt,
		const bool landed)
{
	// angular rates error: e = rate_sp - rate
	const Vector3f rate_error = rate_sp - rate;

	Vector3f rate_sp_derivative;

	if (_smc_rate_sp_prev_valid && dt > FLT_EPSILON) {
		rate_sp_derivative = (rate_sp - _smc_last_rate_sp) * (1.f / dt);
	}

	_smc_last_rate_sp = rate_sp;
	_smc_rate_sp_prev_valid = true;

	// update integral only if we are not landed
	if (!landed) {
		updateSMCIntegral(rate_error, dt);

	} else {
		_smc_rate_int.zero();
		_smc_s_filtered.zero();
		_smc_last_torque.zero();
		_smc_last_rate_sp.zero();
		_smc_rate_sp_prev_valid = false;
	}

	const Vector3f angular_momentum = _msmc_inertia.emult(rate);
	const Vector3f gyro_compensation = rate.cross(angular_momentum);

	Vector3f torque;

	for (int i = 0; i < 3; i++) {
		// Sliding surface s_i = e_i + c_i * e_int_i
		float s_i = rate_error(i) + _msmc_c(i) * _smc_rate_int(i);

		// Apply low-pass filter on the sliding surface
		float s_filtered = s_i;

		if (_smc_lpf_cutoff > FLT_EPSILON) {
			const float tau = 1.f / (2.f * M_PI_F * _smc_lpf_cutoff);
			const float alpha = dt / (tau + dt);

			if (!PX4_ISFINITE(_smc_s_filtered(i))) {
				_smc_s_filtered(i) = s_i;
			}

			s_filtered = alpha * s_i + (1.f - alpha) * _smc_s_filtered(i);
			_smc_s_filtered(i) = s_filtered;

		} else {
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

		float rate_sp_dot = rate_sp_derivative(i);

		if (_msmc_rate_sp_derivative_limit > FLT_EPSILON) {
			rate_sp_dot = math::constrain(rate_sp_dot, -_msmc_rate_sp_derivative_limit,
						      _msmc_rate_sp_derivative_limit);
		}

		// Model-based control law:
		// tau = omega x J omega + J * (omega_sp_dot + c * e + eta * sat(s / phi) + ks * s)
		const float reaching_rate = rate_sp_dot + _msmc_c(i) * rate_error(i) + _msmc_eta(i) * sat_s
					    + _msmc_ks(i) * s_filtered;
		torque(i) = _gain_ff(i) * rate_sp(i) + gyro_compensation(i) + inertia * reaching_rate;
	}

	// Apply torque output slew rate limiting if enabled
	if (_smc_slew_max > FLT_EPSILON && !landed) {
		const float max_delta = _smc_slew_max * dt;

		for (int i = 0; i < 3; i++) {
			if (PX4_ISFINITE(_smc_last_torque(i))) {
				torque(i) = math::constrain(torque(i), _smc_last_torque(i) - max_delta, _smc_last_torque(i) + max_delta);
			}
		}
	}

	_smc_last_torque = torque;

	return torque;
}

Vector3f RateControl::updateSMC(const Vector3f &rate, const Vector3f &rate_sp, const float dt, const bool landed)
{
	// angular rates error: e = rate_sp - rate
	Vector3f rate_error = rate_sp - rate;

	// update integral only if we are not landed
	if (!landed) {
		updateSMCIntegral(rate_error, dt);
	} else {
		_smc_rate_int.zero();
		_smc_s_filtered.zero();
		_smc_last_torque.zero();
	}

	Vector3f torque;
	for (int i = 0; i < 3; i++) {
		// Sliding surface s_i = e_i + c_i * e_int_i
		float s_i = rate_error(i) + _smc_c(i) * _smc_rate_int(i);

		// Apply low-pass filter on the sliding surface
		float s_filtered = s_i;
		if (_smc_lpf_cutoff > FLT_EPSILON) {
			const float tau = 1.f / (2.f * M_PI_F * _smc_lpf_cutoff);
			const float alpha = dt / (tau + dt);
			if (!PX4_ISFINITE(_smc_s_filtered(i))) {
				_smc_s_filtered(i) = s_i;
			}
			s_filtered = alpha * s_i + (1.f - alpha) * _smc_s_filtered(i);
			_smc_s_filtered(i) = s_filtered;
		} else {
			_smc_s_filtered(i) = s_i;
		}

		// Saturation function sat(s_filtered / phi_i)
		float sat_s = s_filtered;
		if (_smc_bnd(i) > 1e-4f) {
			sat_s = math::constrain(s_filtered / _smc_bnd(i), -1.f, 1.f);
		} else {
			sat_s = (s_filtered > 0.f) ? 1.f : ((s_filtered < 0.f) ? -1.f : 0.f);
		}

		// Control law: torque_i = FF_i * rate_sp_i + Keq_i * error_i + eta_i * sat(s_filtered / phi_i) + ks_i * s_filtered
		torque(i) = _gain_ff(i) * rate_sp(i) + _smc_keq(i) * rate_error(i) + _smc_eta(i) * sat_s + _smc_ks(i) * s_filtered;
	}

	// Apply torque output slew rate limiting if enabled
	if (_smc_slew_max > FLT_EPSILON && !landed) {
		const float max_delta = _smc_slew_max * dt;
		for (int i = 0; i < 3; i++) {
			if (PX4_ISFINITE(_smc_last_torque(i))) {
				torque(i) = math::constrain(torque(i), _smc_last_torque(i) - max_delta, _smc_last_torque(i) + max_delta);
			}
		}
	}

	_smc_last_torque = torque;

	return torque;
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
	if (_controller_type == 1 || _controller_type == 2) {
		rate_ctrl_status.rollspeed_integ = _smc_rate_int(0);
		rate_ctrl_status.pitchspeed_integ = _smc_rate_int(1);
		rate_ctrl_status.yawspeed_integ = _smc_rate_int(2);
	} else {
		rate_ctrl_status.rollspeed_integ = _rate_int(0);
		rate_ctrl_status.pitchspeed_integ = _rate_int(1);
		rate_ctrl_status.yawspeed_integ = _rate_int(2);
	}
}
