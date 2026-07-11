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
 * PID, constrained MPC, and model-based SMC 3-axis angular-rate control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
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

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	/**
	 * Set controller type (0 = PID, 1 = MPC, 2 = model-based SMC)
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

	void resetSetpointHistory()
	{
		resetModelBasedSmcSetpoint();
		resetMpcSetpoint();
	}

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

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral()
	{
		_rate_int.zero();
		resetSmcState();
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

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);
	matrix::Vector3f updateMPC(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				   const float dt, const bool landed);
	matrix::Vector3f updateModelBasedSMC(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
					     const float dt, const bool landed);
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
	matrix::Vector3f _last_output;
	bool _last_output_valid{false};

	// Safeguards configurations
	float _smc_lpf_cutoff{20.0f}; ///< Cutoff frequency for sliding surface LPF (Hz)
	float _smc_slew_max{10.0f}; ///< Maximum torque rate of change per second

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
