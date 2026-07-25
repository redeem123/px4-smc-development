/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <lib/rate_control/rate_control.hpp>

using namespace matrix;

namespace
{
struct SmcTestParameters {
	Vector3f inertia{1.f, 1.f, 1.f};
	Vector3f control_effectiveness{1.f, 1.f, 1.f};
	Vector3f c{0.1f, 0.1f, 0.1f};
	Vector3f eta{};
	Vector3f boundary{0.1f, 0.1f, 0.1f};
	Vector3f ks{};
	float rate_sp_derivative_limit{0.f};
	Vector3f integral_limit{0.3f, 0.3f, 0.3f};
	Vector3f torque_limit{1.f, 1.f, 1.f};
	float cutoff{0.f};
	float slew{0.f};
};

struct AstsmcTestParameters {
	Vector3f inertia{1.f, 1.f, 1.f};
	Vector3f control_effectiveness{1.f, 1.f, 1.f};
	Vector3f k1{1.f, 1.f, 1.f};
	Vector3f k2{1.f, 1.f, 1.f};
	Vector3f torque_limit{1.f, 1.f, 1.f};
	Vector3f reference_acceleration_limit{100.f, 100.f, 100.f};
	Vector3f reference_jerk_limit{1000.f, 1000.f, 1000.f};
	Vector3f residual_torque_limit{1.f, 1.f, 1.f};
	Vector3f actuator_time_constant;
	Vector3f torque_slew_rate;
	Vector3f variation_weight;
	Vector3f sliding_boundary;
	float tracking_blend{0.f};
	float reference_feedforward{0.f};
	float reference_feedforward_rp{1.f};
	float gyro_compensation{0.f};
	float dt_min{0.000125f};
	float dt_max{0.02f};
	float recovery_error_threshold{1.f};
	float quiet_k1_error_threshold{0.25f};
	float selective_release_error_threshold{0.2f};
	float selective_release_state_rate{20.f};
	float trim_command_threshold{0.15f};
	float trim_error_threshold{0.2f};
	float trim_acceleration_threshold{1.f};
	float trim_confidence_time{0.5f};
	float trim_time_constant{20.f};
	float roll_pitch_residual_extension{0.f};
	float roll_pitch_k1_recovery_boost{0.f};
	float yaw_residual_extension{0.f};
};

bool configureModelBasedSmc(RateControl &rate_control, const SmcTestParameters &parameters)
{
	return rate_control.setModelBasedSmcParameters(parameters.inertia, parameters.control_effectiveness,
			parameters.c, parameters.eta, parameters.boundary, parameters.ks,
			parameters.rate_sp_derivative_limit, parameters.integral_limit, parameters.torque_limit,
			parameters.cutoff, parameters.slew);
}

bool configureAstsmc(RateControl &rate_control, const AstsmcTestParameters &parameters)
{
	return rate_control.setAstsmcParameters(parameters.inertia, parameters.control_effectiveness,
						parameters.k1, parameters.k2, parameters.torque_limit,
						parameters.reference_acceleration_limit, parameters.reference_jerk_limit,
						parameters.residual_torque_limit, parameters.actuator_time_constant,
						parameters.torque_slew_rate, parameters.variation_weight,
						parameters.sliding_boundary,
						parameters.tracking_blend, parameters.reference_feedforward,
						parameters.reference_feedforward_rp, parameters.gyro_compensation,
						parameters.dt_min, parameters.dt_max, parameters.recovery_error_threshold,
						parameters.quiet_k1_error_threshold,
						parameters.selective_release_error_threshold,
						parameters.selective_release_state_rate,
						parameters.trim_command_threshold, parameters.trim_error_threshold,
						parameters.trim_acceleration_threshold,
						parameters.trim_confidence_time, parameters.trim_time_constant,
						parameters.roll_pitch_residual_extension,
						parameters.roll_pitch_k1_recovery_boost,
						parameters.yaw_residual_extension);
}

float simulateAstsmcScalarRamp(bool model_assisted)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(2.f, 2.f, 2.f);
	parameters.control_effectiveness = Vector3f(4.f, 4.f, 4.f);
	parameters.k1 = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.k2 = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.reference_acceleration_limit = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.reference_jerk_limit = Vector3f(5.f, 5.f, 5.f);
	parameters.residual_torque_limit = model_assisted ? Vector3f(0.2f, 0.2f, 0.2f) : parameters.torque_limit;
	parameters.reference_feedforward = model_assisted ? 1.f : 0.f;
	parameters.dt_max = 0.01f;
	EXPECT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float ramp_rate = 0.4f;
	constexpr float target_limit = 0.8f;
	const float control_gain = parameters.control_effectiveness(0) / parameters.inertia(0);
	float rate = 0.f;
	float absolute_error_integral = 0.f;

	for (int sample = 0; sample < 1000; sample++) {
		const float time = sample * dt;
		const float target = math::min(ramp_rate * time, target_limit);
		const Vector3f torque = rate_control.update(Vector3f(rate, 0.f, 0.f), Vector3f(target, 0.f, 0.f),
							    Vector3f(), dt, false);
		rate += control_gain * torque(0) * dt;

		if (time > 0.2f) {
			absolute_error_integral += fabsf(target - rate) * dt;
		}
	}

	return absolute_error_integral;
}

struct RigidBodyTrackingMetrics {
	float yaw_absolute_error_integral;
	float final_error_norm;
	uint32_t invalid_dt_hold_count;
	uint32_t total_bound_violation_count;
};

struct UncertainYawTrackingMetrics {
	float absolute_error_integral;
	float final_absolute_error;
	float maximum_absolute_torque;
	uint32_t invalid_dt_hold_count;
	uint32_t state_recovery_count;
	uint32_t total_bound_violation_count;
	bool runtime_fault_latched;
};

UncertainYawTrackingMetrics simulateUncertainRealYawCard(float card_effectiveness, float residual_torque_limit,
		float actual_effectiveness, float matched_load)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.8f, 0.75f, card_effectiveness);
	parameters.k1 = Vector3f(3.f, 3.f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, residual_torque_limit);
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.f;
	parameters.gyro_compensation = 1.f;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	EXPECT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float actuator_time_constant = 0.025f;
	const float alpha = expf(-dt / actuator_time_constant);
	float yaw_rate = 0.f;
	float applied_torque = 0.f;
	float absolute_error_integral = 0.f;
	float maximum_absolute_torque = 0.f;

	for (int sample = 0; sample < 2000; sample++) {
		const float time = sample * dt;
		float target = 0.f;

		if (time >= 0.5f && time < 3.f) {
			target = 0.25f;

		} else if (time < 5.5f) {
			target = -0.2f;
		}

		const Vector3f torque = rate_control.update(Vector3f(0.f, 0.f, yaw_rate), Vector3f(0.f, 0.f, target),
					Vector3f(), dt, false, dt);
		maximum_absolute_torque = math::max(maximum_absolute_torque, fabsf(torque(2)));
		applied_torque = alpha * applied_torque + (1.f - alpha) * torque(2);
		yaw_rate += (actual_effectiveness * applied_torque + matched_load) / parameters.inertia(2) * dt;
		absolute_error_integral += fabsf(target - yaw_rate) * dt;
	}

	astsmc_status_s status{};
	astsmc_safety_status_s safety_status{};
	rate_control.getAstsmcStatus(status);
	rate_control.getAstsmcSafetyStatus(safety_status);

	return {
		absolute_error_integral,
		fabsf(yaw_rate),
		maximum_absolute_torque,
		status.invalid_dt_hold_count,
		safety_status.state_recovery_count[2],
		status.total_bound_violation_count,
		safety_status.runtime_fault_latched,
	};
}

struct CommandReleaseTrackingMetrics {
	float release_error_integral;
	float maximum_release_error;
	float final_error;
	float maximum_absolute_torque;
	uint32_t invalid_dt_hold_count;
	uint32_t selective_release_count;
	uint32_t state_recovery_count;
	uint32_t total_bound_violation_count;
	bool runtime_fault_latched;
};

struct UnderAuthorityTrackingMetrics {
	float absolute_error_integral;
	float peak_absolute_error;
	float final_absolute_error;
	float maximum_absolute_torque;
	uint32_t state_recovery_count;
	uint32_t total_bound_violation_count;
	bool runtime_fault_latched;
};

CommandReleaseTrackingMetrics simulateRealRollPitchCommandRelease(int axis, float reference_feedforward_rp,
		float actual_gain_scale, float matched_acceleration, bool selective_release = false)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.8f, 0.75f, 0.8f);
	parameters.k1 = Vector3f(3.f, 3.f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = reference_feedforward_rp;
	parameters.gyro_compensation = 1.f;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	parameters.selective_release_state_rate = selective_release ? 20.f : 0.f;
	EXPECT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float actuator_time_constant = 0.025f;
	const float alpha = expf(-dt / actuator_time_constant);
	const float model_gain = parameters.control_effectiveness(axis) / parameters.inertia(axis);
	const float actual_gain = actual_gain_scale * model_gain;
	Vector3f rate;
	Vector3f applied_torque;
	float release_error_integral = 0.f;
	float maximum_release_error = 0.f;
	float maximum_absolute_torque = 0.f;

	for (int sample = 0; sample < 1750; sample++) {
		const float time = sample * dt;
		float target = 0.f;

		if (time >= 0.5f && time < 1.5f) {
			target = 2.f;

		} else if (time >= 2.5f && time < 3.5f) {
			target = -2.f;

		} else if (time >= 4.5f && time < 5.5f) {
			target = 2.f;
		}

		Vector3f rate_setpoint;
		rate_setpoint(axis) = target;
		const Vector3f commanded_torque = rate_control.update(rate, rate_setpoint, Vector3f(), dt, false, dt);
		applied_torque = alpha * applied_torque + (1.f - alpha) * commanded_torque;
		rate(axis) += (actual_gain * applied_torque(axis) + matched_acceleration) * dt;
		maximum_absolute_torque = math::max(maximum_absolute_torque, fabsf(commanded_torque(axis)));

		const bool release_window = (time >= 1.5f && time < 2.5f)
					    || (time >= 3.5f && time < 4.5f)
					    || time >= 5.5f;

		if (release_window) {
			release_error_integral += fabsf(rate(axis)) * dt;
			maximum_release_error = math::max(maximum_release_error, fabsf(rate(axis)));
		}
	}

	astsmc_status_s status{};
	astsmc_safety_status_s safety_status{};
	rate_control.getAstsmcStatus(status);
	rate_control.getAstsmcSafetyStatus(safety_status);

	return {
		release_error_integral,
		maximum_release_error,
		fabsf(rate(axis)),
		maximum_absolute_torque,
		status.invalid_dt_hold_count,
		safety_status.selective_release_count[axis],
		safety_status.state_recovery_count[axis],
		status.total_bound_violation_count,
		safety_status.runtime_fault_latched,
	};
}

UnderAuthorityTrackingMetrics simulateRealRollPitchUnderAuthority(int axis, float card_effectiveness,
		float k1, float torque_limit, float reference_feedforward_rp, float residual_torque_limit,
		float actual_effectiveness, float roll_pitch_residual_extension = 0.f,
		float roll_pitch_k1_recovery_boost = 0.f)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(card_effectiveness, card_effectiveness, 0.8f);
	parameters.k1 = Vector3f(k1, k1, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(torque_limit, torque_limit, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(residual_torque_limit, residual_torque_limit, 0.1f);
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = reference_feedforward_rp;
	parameters.gyro_compensation = 1.f;
	parameters.roll_pitch_residual_extension = roll_pitch_residual_extension;
	parameters.roll_pitch_k1_recovery_boost = roll_pitch_k1_recovery_boost;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	EXPECT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float actuator_time_constant = 0.025f;
	const float actuator_alpha = expf(-dt / actuator_time_constant);
	const float actual_gain = actual_effectiveness / parameters.inertia(axis);
	Vector3f rate;
	Vector3f applied_torque;
	float absolute_error_integral = 0.f;
	float peak_absolute_error = 0.f;
	float maximum_absolute_torque = 0.f;

	for (int sample = 0; sample < 2000; sample++) {
		const float time = sample * dt;
		float target = 0.f;

		if (time >= 0.75f && time < 2.75f) {
			target = 3.5f;

		} else if (time >= 3.75f && time < 5.75f) {
			target = -3.5f;
		}

		Vector3f rate_setpoint;
		rate_setpoint(axis) = target;
		const Vector3f commanded_torque = rate_control.update(rate, rate_setpoint, Vector3f(), dt, false, dt);
		applied_torque = actuator_alpha * applied_torque + (1.f - actuator_alpha) * commanded_torque;
		rate(axis) += actual_gain * applied_torque(axis) * dt;
		const float absolute_error = fabsf(target - rate(axis));
		absolute_error_integral += absolute_error * dt;
		peak_absolute_error = math::max(peak_absolute_error, absolute_error);
		maximum_absolute_torque = math::max(maximum_absolute_torque, fabsf(commanded_torque(axis)));
	}

	astsmc_status_s status{};
	astsmc_safety_status_s safety_status{};
	rate_control.getAstsmcStatus(status);
	rate_control.getAstsmcSafetyStatus(safety_status);

	return {
		absolute_error_integral,
		peak_absolute_error,
		fabsf(rate(axis)),
		maximum_absolute_torque,
		safety_status.state_recovery_count[axis],
		status.total_bound_violation_count,
		safety_status.runtime_fault_latched,
	};
}

RigidBodyTrackingMetrics simulateUav985Astsmc(bool model_assisted, float yaw_sign)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.040461f, 0.035366f, 0.050951f);
	parameters.control_effectiveness = Vector3f(6.978f, 6.978f, 1.163f);
	parameters.k1 = Vector3f(3.f, 3.f, 1.5f);
	parameters.k2 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(20.f, 20.f, 3.f);
	parameters.reference_jerk_limit = Vector3f(100.f, 100.f, 20.f);
	parameters.residual_torque_limit = model_assisted ? Vector3f(0.1f, 0.1f, 0.05f) : parameters.torque_limit;
	parameters.reference_feedforward = model_assisted ? 1.f : 0.f;
	parameters.gyro_compensation = model_assisted ? 1.f : 0.f;
	parameters.dt_max = 0.01f;
	EXPECT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));

	const Vector3f actual_inertia = parameters.inertia.emult(Vector3f(1.12f, 0.9f, 1.15f));
	const Vector3f actual_effectiveness = parameters.control_effectiveness.emult(Vector3f(0.9f, 1.08f, 0.85f));
	const Vector3f disturbance(0.002f, -0.0015f, 0.001f * yaw_sign);
	constexpr float actuator_time_constant = 0.025f;
	const float timing_sequence[] {0.004f, 0.004f, 0.004f, 0.008f};
	Vector3f rate;
	Vector3f applied_torque;
	Vector3f target;
	float time = 0.f;
	float yaw_absolute_error_integral = 0.f;
	int sample = 0;

	while (time < 5.f) {
		const float raw_dt = timing_sequence[sample % 4];

		if (time < 0.3f) {
			target.zero();

		} else if (time < 2.6f) {
			target = Vector3f(0.45f, -0.35f, yaw_sign * 0.8f);

		} else {
			target = Vector3f(-0.25f, 0.2f, -yaw_sign * 0.5f);
		}

		const Vector3f commanded_torque = rate_control.update(rate, target, Vector3f(), raw_dt, false, raw_dt);
		const float alpha = expf(-raw_dt / actuator_time_constant);
		applied_torque = alpha * applied_torque + (1.f - alpha) * commanded_torque;
		const Vector3f gyro = rate.cross(actual_inertia.emult(rate));
		const Vector3f angular_acceleration = (actual_effectiveness.emult(applied_torque) - gyro + disturbance)
						      .edivide(actual_inertia);
		rate += angular_acceleration * raw_dt;
		yaw_absolute_error_integral += fabsf(target(2) - rate(2)) * raw_dt;
		time += raw_dt;
		sample++;
	}

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	return {
		yaw_absolute_error_integral,
		(target - rate).norm(),
		status.invalid_dt_hold_count,
		status.total_bound_violation_count,
	};
}
}

TEST(RateControlTest, AllZeroCase)
{
	RateControl rate_control;
	Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.f, false);
	EXPECT_EQ(torque, Vector3f());
}

TEST(RateControlTest, MpcMode)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_GT(torque(0), 0.f);
	EXPECT_LE(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, MpcTorqueOutputIsNormalized)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(100.f, -100.f, 100.f), Vector3f(), 0.01f,
				false);

	EXPECT_FLOAT_EQ(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), -1.f);
	EXPECT_FLOAT_EQ(torque(2), 1.f);
}

TEST(RateControlTest, MpcTorqueOutputUsesConfiguredLimit)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);
	rate_control.setMpcTorqueLimit(Vector3f(0.2f, 0.3f, 0.1f));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(100.f, -100.f, 100.f), Vector3f(), 0.01f,
				false);

	EXPECT_FLOAT_EQ(torque(0), 0.2f);
	EXPECT_FLOAT_EQ(torque(1), -0.3f);
	EXPECT_FLOAT_EQ(torque(2), 0.1f);
}

TEST(RateControlTest, MpcUsesRigidBodyGyroCompensation)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(2.f, 3.f, 4.f), Vector3f(100.f, 100.f, 100.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);
	rate_control.setMpcGyroCompensation(1.f);
	rate_control.setMpcActuatorTimeConstant(0.f);

	const Vector3f rates(0.1f, 0.2f, 0.3f);
	const Vector3f torque = rate_control.update(rates, rates, Vector3f(), 0.1f, false);

	EXPECT_NEAR(torque(0), 0.06f, 1e-3f);
	EXPECT_NEAR(torque(1), -0.06f, 1e-3f);
	EXPECT_NEAR(torque(2), 0.02f, 1e-3f);
}

TEST(RateControlTest, MpcUsesLimitedRateSetpointDerivativeFeedForward)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(2.f, 2.f, 2.f), Vector3f(100.f, 100.f, 100.f), Vector3f(4.f, 4.f, 4.f),
				 Vector3f(1.f, 1.f, 1.f), Vector3f(), 1, 0.f);
	rate_control.setMpcActuatorTimeConstant(0.f);
	rate_control.setMpcRateSetpointDerivativeLimit(1.f);

	const Vector3f first_torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.1f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.1f, -0.1f, 0.f), Vector3f(), 0.1f, false);

	EXPECT_EQ(first_torque, Vector3f());
	EXPECT_GT(torque(0), 0.5f);
	EXPECT_LT(torque(1), -0.5f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, MpcTorqueSlewLimit)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 1.f);

	const Vector3f first_torque = rate_control.update(Vector3f(), Vector3f(100.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(-100.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(first_torque(0), 1.f);
	EXPECT_NEAR(torque(0), 0.99f, 1e-5f);
}

TEST(RateControlTest, MpcInvalidPhysicalModelRetainsLastValidValues)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcActuatorTimeConstant(0.f);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);
	rate_control.setMpcGains(Vector3f(NAN, -1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_GT(torque(0), 0.f);
	EXPECT_NEAR(torque(0), torque(1), 1e-5f);
	EXPECT_NEAR(torque(1), torque(2), 1e-5f);
}

TEST(RateControlTest, MpcIntegralBiasIsBoundedAndResetWhenLanded)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(100.f, 100.f, 100.f), Vector3f(), 1, 0.f);
	rate_control.setMpcIntegralGain(Vector3f(1.f, 0.f, 0.f));
	rate_control.setMpcIntegralLimit(Vector3f(0.02f, 0.02f, 0.02f));

	const Vector3f first_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.01f, false);
	Vector3f torque = first_torque;

	for (int i = 0; i < 100; i++) {
		torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.01f, false);
	}

	EXPECT_GT(torque(0), first_torque(0));

	rate_ctrl_status_s rate_ctrl_status{};
	rate_control.getRateControlStatus(rate_ctrl_status);
	EXPECT_NEAR(rate_ctrl_status.rollspeed_integ, 0.02f, 1e-5f);
	EXPECT_FLOAT_EQ(rate_ctrl_status.pitchspeed_integ, 0.f);
	EXPECT_FLOAT_EQ(rate_ctrl_status.yawspeed_integ, 0.f);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, true);
	rate_control.getRateControlStatus(rate_ctrl_status);
	EXPECT_FLOAT_EQ(rate_ctrl_status.rollspeed_integ, 0.f);
}

TEST(RateControlTest, ModelBasedSmcMode)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.eta = Vector3f(0.9f, 0.9f, 0.9f);
	parameters.rate_sp_derivative_limit = 10.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.5f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcGyroCompensation)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(2.f, 3.f, 4.f);
	parameters.rate_sp_derivative_limit = 10.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f rates(0.1f, 0.2f, 0.3f);
	const Vector3f torque = rate_control.update(rates, rates, Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.06f);
	EXPECT_FLOAT_EQ(torque(1), -0.06f);
	EXPECT_FLOAT_EQ(torque(2), 0.02f);
}

TEST(RateControlTest, ModelBasedSmcRateSetpointDerivativeLimit)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.rate_sp_derivative_limit = 10.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcDoesNotUsePidFeedForwardOrDerivative)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(), Vector3f(), Vector3f(0.1f, 0.2f, 0.3f));
	rate_control.setFeedForwardGain(Vector3f(0.4f, 0.5f, 0.6f));
	SmcTestParameters parameters;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 2.f, -3.f), Vector3f(1.f, 2.f, -3.f),
				0.01f, false);

	EXPECT_NEAR(torque(0), 0.1f, 1e-6f);
	EXPECT_NEAR(torque(1), 0.2f, 1e-6f);
	EXPECT_NEAR(torque(2), -0.3f, 1e-6f);
}

namespace
{

// Reduced roll/pitch card with the EFF_Y=0.8 yaw correction that flew logs
// 11_39_30 and 11_41_04. It is the behavioral reference for the model update.
SmcTestParameters legacyRealMode2Card()
{
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.02f);
	parameters.control_effectiveness = Vector3f(1.f, 1.f, 0.8f);
	parameters.c = Vector3f(2.f, 2.f, 2.5f);
	parameters.eta = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.boundary = Vector3f(0.5f, 0.5f, 0.2f);
	parameters.ks = Vector3f(1.f, 1.f, 1.f);
	parameters.integral_limit = Vector3f(0.2f, 0.3f, 2.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.17f);
	parameters.cutoff = 40.f;
	parameters.slew = 15.f;
	return parameters;
}

// Identified physical model with the surface, reaching, linear, and integral
// gains scaled by the inverse J/EFF change so that the flight-validated
// small-error torque slope is preserved. Mirrors px4_mode2_configure.py.
SmcTestParameters identifiedRealMode2Card()
{
	SmcTestParameters parameters = legacyRealMode2Card();
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	parameters.c = Vector3f(1.12f, 1.06f, 0.981335f);
	parameters.eta = Vector3f(2.52f, 2.385f, 0.588801f);
	parameters.ks = Vector3f(0.56f, 0.53f, 0.392534f);
	parameters.integral_limit = Vector3f(0.357143f, 0.566038f, 5.f);
	return parameters;
}

// Strict paper card: the error integral is clamped away so the surface is the
// s2 of eq. 30, the boundary layer collapses to the parameter minimum so the
// reaching term approximates sign(s2), and the output filters are off.
// Mirrors px4_mode2_configure.py --paper-strict.
SmcTestParameters strictRealMode2Card()
{
	SmcTestParameters parameters = identifiedRealMode2Card();
	parameters.boundary = Vector3f(0.01f, 0.01f, 0.01f);
	parameters.integral_limit = Vector3f();
	// The paper has no software torque limit, so only allocator saturation remains.
	parameters.torque_limit = Vector3f(1.f, 1.f, 1.f);
	parameters.cutoff = 0.f;
	parameters.slew = 0.f;
	return parameters;
}

// Identified model without the compensating gain scaling: the card that a plain
// J/EFF replacement would install.
SmcTestParameters uncompensatedRealMode2Card()
{
	SmcTestParameters parameters = legacyRealMode2Card();
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	return parameters;
}

Vector3f holdConstantRateError(const SmcTestParameters &parameters, const Vector3f &rate_error, const float duration)
{
	RateControl rate_control;
	EXPECT_TRUE(configureModelBasedSmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(2));

	constexpr float dt = 0.004f;
	const int samples = math::max(1, static_cast<int>(roundf(duration / dt)));
	Vector3f torque;

	for (int sample = 0; sample < samples; sample++) {
		torque = rate_control.update(Vector3f(), rate_error, Vector3f(), dt, false);
	}

	return torque;
}

} // namespace

TEST(RateControlTest, ModelBasedSmcIdentifiedCardPreservesSmallErrorTorque)
{
	const SmcTestParameters legacy = legacyRealMode2Card();
	const SmcTestParameters identified = identifiedRealMode2Card();
	const SmcTestParameters uncompensated = uncompensatedRealMode2Card();

	// Amplitudes small enough that neither the torque nor the slew limit binds,
	// so the comparison isolates the control law.
	for (const float amplitude : {0.02f, 0.04f, 0.06f}) {
		const Vector3f rate_error(amplitude, amplitude, amplitude);
		const Vector3f legacy_torque = holdConstantRateError(legacy, rate_error, 0.004f);
		const Vector3f identified_torque = holdConstantRateError(identified, rate_error, 0.004f);
		const Vector3f uncompensated_torque = holdConstantRateError(uncompensated, rate_error, 0.004f);

		for (int axis = 0; axis < 3; axis++) {
			// The compensated card reproduces the flown response; the single-step
			// integral coupling is the only residual difference.
			EXPECT_NEAR(identified_torque(axis), legacy_torque(axis), 0.01f * fabsf(legacy_torque(axis)));
			// A plain model replacement would command far more torque.
			EXPECT_GT(fabsf(uncompensated_torque(axis)), 1.5f * fabsf(legacy_torque(axis)));
		}
	}

	// Repeat above the boundary layer with the output slew limit released, which
	// otherwise dominates the first sample of a large step.
	SmcTestParameters unslewed_legacy = legacy;
	SmcTestParameters unslewed_identified = identified;
	unslewed_legacy.slew = 0.f;
	unslewed_identified.slew = 0.f;

	for (const float amplitude : {0.6f, 1.2f}) {
		const Vector3f rate_error(amplitude, amplitude, amplitude);
		const Vector3f legacy_torque = holdConstantRateError(unslewed_legacy, rate_error, 0.004f);
		const Vector3f identified_torque = holdConstantRateError(unslewed_identified, rate_error, 0.004f);

		for (int axis = 0; axis < 3; axis++) {
			EXPECT_LT(fabsf(legacy_torque(axis)), legacy.torque_limit(axis));
			EXPECT_NEAR(identified_torque(axis), legacy_torque(axis), 0.01f * fabsf(legacy_torque(axis)));
		}
	}
}

TEST(RateControlTest, ModelBasedSmcIdentifiedCardCorrectsGyroCoupling)
{
	const SmcTestParameters legacy = legacyRealMode2Card();
	const SmcTestParameters identified = identifiedRealMode2Card();
	const Vector3f rate(0.5f, 0.5f, 0.5f);

	RateControl legacy_control;
	ASSERT_TRUE(configureModelBasedSmc(legacy_control, legacy));
	ASSERT_TRUE(legacy_control.setControllerType(2));

	RateControl identified_control;
	ASSERT_TRUE(configureModelBasedSmc(identified_control, identified));
	ASSERT_TRUE(identified_control.setControllerType(2));

	// Zero rate error isolates the modeled gyroscopic term w x (J w) / EFF.
	const Vector3f legacy_torque = legacy_control.update(rate, rate, Vector3f(), 0.004f, false);
	const Vector3f identified_torque = identified_control.update(rate, rate, Vector3f(), 0.004f, false);

	EXPECT_NEAR(legacy_torque(0), 0.25f * (0.02f - 0.01f) / 1.f, 1e-6f);
	EXPECT_NEAR(legacy_torque(1), 0.25f * (0.01f - 0.02f) / 1.f, 1e-6f);
	EXPECT_NEAR(identified_torque(0), 0.25f * (0.050951f - 0.01f) / 0.56f, 1e-6f);
	EXPECT_NEAR(identified_torque(1), 0.25f * (0.01f - 0.050951f) / 0.53f, 1e-6f);
	EXPECT_NEAR(identified_torque(2), 0.f, 1e-6f);
}

TEST(RateControlTest, ModelBasedSmcIdentifiedCardKeepsIntegralEndpoint)
{
	const SmcTestParameters legacy = legacyRealMode2Card();
	const SmcTestParameters identified = identifiedRealMode2Card();
	const Vector3f rate_error(0.08f, 0.08f, 0.08f);

	// The corrected model makes the integral surface offset build more slowly.
	const Vector3f legacy_transient = holdConstantRateError(legacy, rate_error, 0.2f);
	const Vector3f identified_transient = holdConstantRateError(identified, rate_error, 0.2f);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_LT(identified_transient(axis), legacy_transient(axis));
		EXPECT_GT(identified_transient(axis), 0.8f * legacy_transient(axis));
	}

	// Once both error integrals reach their limits, the rescaled MC_MSMC_ILIM_*
	// values restore the flown integral surface offset. Yaw stays about 2% short
	// because MC_MSMC_ILIM_Y is truncated at its parameter maximum of 5.
	const Vector3f legacy_settled = holdConstantRateError(legacy, rate_error, 90.f);
	const Vector3f identified_settled = holdConstantRateError(identified, rate_error, 90.f);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_NEAR(identified_settled(axis), legacy_settled(axis), 0.02f * fabsf(legacy_settled(axis)));
	}
}

TEST(RateControlTest, ModelBasedSmcPaperTermsAreDisabledByDefault)
{
	RateControl rate_control;
	SmcTestParameters parameters = identifiedRealMode2Card();
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	// Supplying state without constants must not change the plain law.
	rate_control.setModelBasedSmcPaperState(400.f, true);
	const Vector3f augmented = rate_control.update(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.5f, 0.5f, 0.5f),
				   Vector3f(), 0.004f, false);

	RateControl reference;
	ASSERT_TRUE(configureModelBasedSmc(reference, parameters));
	ASSERT_TRUE(reference.setControllerType(2));
	const Vector3f plain = reference.update(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.5f, 0.5f, 0.5f), Vector3f(),
						0.004f, false);

	EXPECT_EQ(augmented, plain);
}

TEST(RateControlTest, ModelBasedSmcRotorGyroscopicTermCancelsPropellerTorque)
{
	RateControl rate_control;
	SmcTestParameters parameters = identifiedRealMode2Card();
	parameters.slew = 0.f; // assert the raw law, not the slew limiter
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	constexpr float rotor_inertia = 0.00002f;
	ASSERT_TRUE(rate_control.setModelBasedSmcPaperConstants(Vector3f(), rotor_inertia));

	constexpr float net_rotor_speed = 300.f;
	const Vector3f rate(0.4f, -0.3f, 0.2f);
	rate_control.setModelBasedSmcPaperState(net_rotor_speed, true);
	const Vector3f torque = rate_control.update(rate, rate, Vector3f(), 0.004f, false);

	// Zero rate error leaves only w x (J w) plus the propeller term
	// J_R*Omega*(q, -p, 0), each divided by the axis effectiveness.
	const float rotor_momentum = rotor_inertia * net_rotor_speed;
	const Vector3f body_gyro = rate.cross(parameters.inertia.emult(rate));
	EXPECT_NEAR(torque(0), (body_gyro(0) + rotor_momentum * rate(1)) / parameters.control_effectiveness(0), 1e-6f);
	EXPECT_NEAR(torque(1), (body_gyro(1) - rotor_momentum * rate(0)) / parameters.control_effectiveness(1), 1e-6f);
	EXPECT_NEAR(torque(2), body_gyro(2) / parameters.control_effectiveness(2), 1e-6f);
}

TEST(RateControlTest, ModelBasedSmcAttitudeSurfaceTermFollowsPaperSign)
{
	SmcTestParameters parameters = identifiedRealMode2Card();
	parameters.slew = 0.f; // assert the raw law, not the slew limiter
	const Vector3f slope(4.f, 4.f, 2.8f);
	const Vector3f rate_sp(0.2f, -0.1f, 0.15f);

	RateControl rate_control;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	ASSERT_TRUE(rate_control.setModelBasedSmcPaperConstants(slope, 0.f));
	const Vector3f torque = rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);

	RateControl reference;
	ASSERT_TRUE(configureModelBasedSmc(reference, parameters));
	ASSERT_TRUE(reference.setControllerType(2));
	const Vector3f plain = reference.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);

	// z1 is taken as rate_sp/alpha1, so -alpha1^2*z1 reduces to -alpha1*rate_sp
	// and is the only difference from the plain law.
	for (int axis = 0; axis < 3; axis++) {
		const float expected = -slope(axis) * rate_sp(axis)
				       * parameters.inertia(axis) / parameters.control_effectiveness(axis);
		EXPECT_NEAR(torque(axis) - plain(axis), expected, 1e-6f);
	}
}

TEST(RateControlTest, ModelBasedSmcAttitudeSurfaceTermSurvivesAttitudeLoopNonlinearity)
{
	SmcTestParameters parameters = identifiedRealMode2Card();
	parameters.slew = 0.f;
	parameters.torque_limit = Vector3f(1.f, 1.f, 1.f); // keep the raw law unclipped
	const Vector3f slope(6.5f, 6.5f, 2.8f);

	// Whatever the attitude loop does to produce rate_sp, quaternion error or a
	// clipped rate limit, the surface stays the s2 of eq. 30 because z1 is read
	// back out of the setpoint rather than measured independently.
	for (const float setpoint : {0.05f, 0.8f, 3.4f}) {
		const Vector3f rate_sp(setpoint, setpoint, setpoint);
		RateControl rate_control;
		ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
		ASSERT_TRUE(rate_control.setControllerType(2));
		ASSERT_TRUE(rate_control.setModelBasedSmcPaperConstants(slope, 0.f));

		RateControl reference;
		ASSERT_TRUE(configureModelBasedSmc(reference, parameters));
		ASSERT_TRUE(reference.setControllerType(2));

		const Vector3f torque = rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);
		const Vector3f plain = reference.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);

		for (int axis = 0; axis < 3; axis++) {
			const float expected = -slope(axis) * setpoint
					       * parameters.inertia(axis) / parameters.control_effectiveness(axis);
			EXPECT_NEAR(torque(axis) - plain(axis), expected, 1e-6f);
		}
	}
}

TEST(RateControlTest, ModelBasedSmcPaperStateIsConsumedEachCycle)
{
	RateControl rate_control;
	SmcTestParameters parameters = identifiedRealMode2Card();
	parameters.slew = 0.f; // assert the raw law, not the slew limiter
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	constexpr float rotor_inertia = 0.00002f;
	ASSERT_TRUE(rate_control.setModelBasedSmcPaperConstants(Vector3f(), rotor_inertia));

	// Zero rate error isolates the rotor term: only w x (J w) survives without it.
	const Vector3f rate(0.4f, -0.3f, 0.2f);
	const Vector3f body_gyro = rate.cross(parameters.inertia.emult(rate));

	rate_control.setModelBasedSmcPaperState(300.f, true);
	const Vector3f supplied = rate_control.update(rate, rate, Vector3f(), 0.004f, false);
	// A publisher that stops updating must not keep feeding the stale speed.
	const Vector3f withheld = rate_control.update(rate, rate, Vector3f(), 0.004f, false);

	EXPECT_GT(fabsf(supplied(0) - body_gyro(0) / parameters.control_effectiveness(0)), 1e-6f);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_NEAR(withheld(axis), body_gyro(axis) / parameters.control_effectiveness(axis), 1e-6f);
	}
}

TEST(RateControlTest, ModelBasedSmcPaperConstantsRejectOutOfRangeValues)
{
	RateControl rate_control;
	EXPECT_FALSE(rate_control.setModelBasedSmcPaperConstants(Vector3f(-1.f, 0.f, 0.f), 0.f));
	EXPECT_FALSE(rate_control.setModelBasedSmcPaperConstants(Vector3f(21.f, 0.f, 0.f), 0.f));
	EXPECT_FALSE(rate_control.setModelBasedSmcPaperConstants(Vector3f(), -0.001f));
	EXPECT_FALSE(rate_control.setModelBasedSmcPaperConstants(Vector3f(), 0.2f));
	EXPECT_FALSE(rate_control.setModelBasedSmcPaperConstants(Vector3f(NAN, 0.f, 0.f), 0.f));
	EXPECT_TRUE(rate_control.setModelBasedSmcPaperConstants(Vector3f(4.f, 4.f, 2.8f), 0.00002f));
}

TEST(RateControlTest, ModelBasedSmcStrictCardMatchesThePaperControlLaw)
{
	const SmcTestParameters parameters = strictRealMode2Card();
	RateControl rate_control;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	const Vector3f slope(6.5f, 6.5f, 2.8f);
	ASSERT_TRUE(rate_control.setModelBasedSmcPaperConstants(slope, 0.f));

	const Vector3f rate(0.4f, -0.3f, 0.2f);
	const Vector3f rate_sp(0.7f, -0.5f, 0.35f);
	const Vector3f rate_error = rate_sp - rate;
	const Vector3f body_gyro = rate.cross(parameters.inertia.emult(rate));

	// U2 of eq. 33 with the k1*sign(s2) reaching term: every rate error here is
	// far outside the collapsed boundary layer. z1 = rate_sp/alpha1 collapses
	// -alpha1^2*z1 to -alpha1*rate_sp.
	Vector3f expected;

	for (int axis = 0; axis < 3; axis++) {
		ASSERT_GT(fabsf(rate_error(axis)), parameters.boundary(axis));
		const float reaching = -slope(axis) * rate_sp(axis)
				       + parameters.c(axis) * rate_error(axis)
				       + parameters.eta(axis) * ((rate_error(axis) > 0.f) ? 1.f : -1.f)
				       + parameters.ks(axis) * rate_error(axis);
		expected(axis) = (body_gyro(axis) + parameters.inertia(axis) * reaching)
				 / parameters.control_effectiveness(axis);
	}

	// Held long enough that a surviving integral would dominate the command.
	Vector3f torque;

	for (int step = 0; step < 500; step++) {
		torque = rate_control.update(rate, rate_sp, Vector3f(), 0.004f, false);
	}

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_NEAR(torque(axis), expected(axis), 1e-6f);
	}
}

TEST(RateControlTest, ModelBasedSmcStrictCardSurfaceCarriesNoIntegral)
{
	const SmcTestParameters parameters = strictRealMode2Card();
	RateControl rate_control;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f rate_sp(0.05f, 0.05f, 0.05f);
	const Vector3f first = rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);

	for (int step = 0; step < 2500; step++) {
		rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);
	}

	// s = e of eq. 30: ten seconds of constant error must not move the command.
	const Vector3f settled = rate_control.update(Vector3f(), rate_sp, Vector3f(), 0.004f, false);
	EXPECT_EQ(settled, first);
}

TEST(RateControlTest, ModelBasedSmcStrictCardReachingTermIsDiscontinuous)
{
	const SmcTestParameters parameters = strictRealMode2Card();
	RateControl rate_control;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	// Either side of a rate error far smaller than the flight card's boundary
	// layer, the reaching term must already have switched sign at full ETA.
	constexpr float small_error = 0.02f;
	const Vector3f positive = rate_control.update(Vector3f(), Vector3f(small_error, small_error, small_error),
				  Vector3f(), 0.004f, false);
	const Vector3f negative = rate_control.update(Vector3f(), Vector3f(-small_error, -small_error, -small_error),
				  Vector3f(), 0.004f, false);

	for (int axis = 0; axis < 3; axis++) {
		const float step = positive(axis) - negative(axis);
		const float switching = 2.f * parameters.eta(axis) * parameters.inertia(axis)
					/ parameters.control_effectiveness(axis);
		EXPECT_GT(step, switching);
	}
}

TEST(RateControlTest, MpcHorizonChangesOptimalCommand)
{
	RateControl short_horizon;
	short_horizon.setControllerType(1);
	short_horizon.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				  Vector3f(1.f, 1.f, 1.f), Vector3f(), 1, 0.f);

	RateControl long_horizon;
	long_horizon.setControllerType(1);
	long_horizon.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(1.f, 1.f, 1.f), Vector3f(), 8, 0.f);

	const float short_command = short_horizon.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);
	const float long_command = long_horizon.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	EXPECT_GT(long_command, short_command);
}

TEST(RateControlTest, MpcTorqueRateWeightPenalizesCommandChange)
{
	RateControl unpenalized;
	unpenalized.setControllerType(1);
	unpenalized.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(10.f, 10.f, 10.f), Vector3f(1.f, 1.f, 1.f),
				Vector3f(1.f, 1.f, 1.f), Vector3f(), 8, 0.f);

	RateControl penalized;
	penalized.setControllerType(1);
	penalized.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(10.f, 10.f, 10.f), Vector3f(1.f, 1.f, 1.f),
			      Vector3f(1.f, 1.f, 1.f), Vector3f(100.f, 100.f, 100.f), 8, 0.f);

	const float unpenalized_command = unpenalized.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f,
					  false)(0);
	const float penalized_command = penalized.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f,
					false)(0);

	EXPECT_LT(penalized_command, unpenalized_command);
}

TEST(RateControlTest, ControllerSwitchUsesLastTorqueForSlewConstraint)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(0.2f, 0.2f, 0.2f), Vector3f(), Vector3f());
	const float pid_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(100.f, 100.f, 100.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 8, 1.f);
	rate_control.setMpcActuatorTimeConstant(0.f);
	rate_control.setControllerType(1);

	const float mpc_torque = rate_control.update(Vector3f(), Vector3f(-10.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	EXPECT_NEAR(pid_torque, 0.2f, 1e-6f);
	EXPECT_LE(fabsf(mpc_torque - pid_torque), 0.0101f);
}

TEST(RateControlTest, ControllerSwitchPrioritizesConfiguredTorqueLimits)
{
	RateControl mpc;
	mpc.setPidGains(Vector3f(0.8f, 0.8f, 0.8f), Vector3f(), Vector3f());
	const float pid_torque = mpc.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);
	mpc.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(100.f, 100.f, 100.f), Vector3f(1.f, 1.f, 1.f),
			Vector3f(), Vector3f(), 8, 1.f);
	mpc.setMpcTorqueLimit(Vector3f(0.2f, 0.2f, 0.2f));
	mpc.setMpcActuatorTimeConstant(0.f);
	mpc.setControllerType(1);

	const float mpc_torque = mpc.update(Vector3f(), Vector3f(10.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	EXPECT_FLOAT_EQ(pid_torque, 0.8f);
	EXPECT_LE(fabsf(mpc_torque), 0.2f);

	RateControl smc;
	smc.setPidGains(Vector3f(0.8f, 0.8f, 0.8f), Vector3f(), Vector3f());
	smc.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	SmcTestParameters smc_parameters;
	smc_parameters.eta = Vector3f(10.f, 10.f, 10.f);
	smc_parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	smc_parameters.slew = 1.f;
	ASSERT_TRUE(configureModelBasedSmc(smc, smc_parameters));
	ASSERT_TRUE(smc.setControllerType(2));

	const float smc_torque = smc.update(Vector3f(), Vector3f(10.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	EXPECT_LE(fabsf(smc_torque), 0.2f);
}

TEST(RateControlTest, ModelBasedSmcUsesPhysicalControlEffectiveness)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.control_effectiveness = Vector3f(2.f, 2.f, 2.f);
	parameters.eta = Vector3f(0.9f, 0.9f, 0.9f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.25f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcUsesIndependentTorqueLimit)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.eta = Vector3f(10.f, 10.f, 10.f);
	parameters.torque_limit = Vector3f(0.2f, 0.3f, 0.1f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, -1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.2f);
	EXPECT_FLOAT_EQ(torque(1), -0.3f);
	EXPECT_FLOAT_EQ(torque(2), 0.1f);
}

TEST(RateControlTest, ModelBasedSmcYawIntegralRejectsMeasuredTorqueBias)
{
	constexpr float dt = 0.0015f;
	constexpr float yaw_disturbance = 0.092f;
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.02f);
	parameters.control_effectiveness = Vector3f(1.f, 1.f, 0.8f);
	parameters.c = Vector3f(2.f, 2.f, 1.5f);
	parameters.eta = Vector3f(3.5f, 3.5f, 1.5f);
	parameters.boundary = Vector3f(0.5f, 0.5f, 0.2f);
	parameters.ks = Vector3f(1.f, 1.f, 1.f);
	parameters.integral_limit = Vector3f(0.3f, 0.3f, 2.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	Vector3f rates;
	Vector3f torque;

	for (int sample = 0; sample < 10000; sample++) {
		torque = rate_control.update(rates, Vector3f(), Vector3f(), dt, false);
		rates(2) += parameters.control_effectiveness(2) / parameters.inertia(2)
			    * (torque(2) + yaw_disturbance) * dt;
	}

	EXPECT_NEAR(rates(2), 0.f, 0.01f);
	EXPECT_NEAR(torque(2), -yaw_disturbance, 0.005f);
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);
	EXPECT_GT(fabsf(status.yawspeed_integ), 0.3f);
}

TEST(RateControlTest, ModelBasedSmcSafeguardUpdatePreservesSlewReference)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.eta = Vector3f(0.9f, 0.9f, 0.9f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	const float initial_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	parameters.cutoff = 20.f;
	parameters.slew = 1.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	const float updated_torque = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.01f, false)(0);

	EXPECT_FLOAT_EQ(initial_torque, 0.5f);
	EXPECT_GE(updated_torque, initial_torque - 0.0101f);
}

TEST(RateControlTest, ModelBasedSmcTorqueOutputIsNormalized)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(5.f, 5.f, 5.f);
	parameters.eta = Vector3f(10.f, 10.f, 10.f);
	parameters.rate_sp_derivative_limit = 1000.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, -1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), -1.f);
	EXPECT_FLOAT_EQ(torque(2), 1.f);
}

TEST(RateControlTest, ModelBasedSmcZeroRateSetpointDerivativeLimitDisablesFeedForward)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.1f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcSetpointResetClearsDerivativeHistory)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.rate_sp_derivative_limit = 10.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.resetModelBasedSmcSetpoint();
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.1f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcLandedUpdateDoesNotSeedTorqueSlew)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.eta = Vector3f(0.9f, 0.f, 0.f);
	parameters.slew = 1.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f landed_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, true);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(landed_torque(0), 1.f);
	EXPECT_FLOAT_EQ(landed_torque(1), 0.f);
	EXPECT_FLOAT_EQ(landed_torque(2), 0.f);
	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcGainChangeClearsSetpointDerivativeHistory)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.rate_sp_derivative_limit = 10.f;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	parameters.inertia = Vector3f(2.f, 2.f, 2.f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.2f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcInvalidCardRetainsLastValidParameters)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.eta = Vector3f(0.9f, 0.9f, 0.9f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));
	const Vector3f valid_torque = rate_control.update(Vector3f(), Vector3f(0.5f, 0.5f, 0.5f), Vector3f(), 0.01f,
				      false);

	parameters.inertia = Vector3f(NAN, NAN, NAN);
	parameters.cutoff = NAN;
	EXPECT_FALSE(configureModelBasedSmc(rate_control, parameters));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.5f, 0.5f, 0.5f), Vector3f(), 0.01f, false);

	EXPECT_EQ(torque, valid_torque);
	EXPECT_EQ(rate_control.getControllerType(), 2);
	EXPECT_TRUE(rate_control.modelBasedSmcParametersValid());
}

TEST(RateControlTest, ModelBasedSmcCannotActivateWithoutValidCard)
{
	RateControl rate_control;

	EXPECT_FALSE(rate_control.setControllerType(2));
	EXPECT_EQ(rate_control.getControllerType(), 0);

	SmcTestParameters parameters;
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(2));
	EXPECT_EQ(rate_control.getControllerType(), 2);
}

TEST(RateControlTest, InvalidControllerTypeIsRejectedWithoutClamping)
{
	RateControl rate_control;

	ASSERT_TRUE(rate_control.setControllerType(1));
	EXPECT_FALSE(rate_control.setControllerType(3));
	EXPECT_EQ(rate_control.getControllerType(), 1);
	EXPECT_FALSE(rate_control.setControllerType(4));
	EXPECT_EQ(rate_control.getControllerType(), 1);
	EXPECT_FALSE(rate_control.setControllerType(-1));
	EXPECT_EQ(rate_control.getControllerType(), 1);
}

TEST(RateControlTest, ModelBasedSmcStatusContainsDiagnostics)
{
	RateControl rate_control;
	SmcTestParameters parameters;
	parameters.inertia = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.eta = Vector3f(0.9f, 0.9f, 0.9f);
	ASSERT_TRUE(configureModelBasedSmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(2));

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	rate_ctrl_status_s status{};
	rate_control.getRateControlStatus(status);

	EXPECT_EQ(status.controller_type, 2);
	EXPECT_TRUE(status.model_valid);
	EXPECT_GT(status.smc_surface[0], 0.f);
	EXPECT_GT(status.smc_torque_raw[0], 0.f);
	EXPECT_FLOAT_EQ(status.smc_torque_limited[0], torque(0));
}

TEST(RateControlTest, ImplicitSuperTwistingCannotActivateWithoutValidCard)
{
	RateControl rate_control;

	EXPECT_FALSE(rate_control.setControllerType(3));
	EXPECT_EQ(rate_control.getControllerType(), 0);

	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_TRUE(rate_control.setControllerType(3));
	EXPECT_EQ(rate_control.getControllerType(), 3);
}

struct CanonicalAstsmcStep {
	float input;
	float state_next;
};

CanonicalAstsmcStep canonicalConditionedAstsmcStep(float sliding, float state, float k1, float k2,
		float input_limit, float dt)
{
	const float lambda = k2 - 0.25f * k1 * k1;
	const float sliding_sign = sliding > 0.f ? 1.f : (sliding < 0.f ? -1.f : 0.f);
	float input_raw;

	if (fabsf(sliding) > k2 * dt * dt) {
		input_raw = state - (2.f * lambda * dt + k1 * sqrtf(fabsf(sliding) - lambda * dt * dt)) * sliding_sign;

	} else {
		input_raw = state - 2.f * sliding / dt;
	}

	const float input = math::constrain(input_raw, -input_limit, input_limit);
	const float state_difference = state - input;
	float state_next;

	if (fabsf(state_difference) > 2.f * k2 * dt) {
		const float difference_sign = state_difference > 0.f ? 1.f : -1.f;
		state_next = state - dt * k2 * difference_sign;

	} else {
		state_next = 0.5f * (state + input);
	}

	return {input, state_next};
}

struct TransformedAstsmcStep {
	float torque;
	float state_next;
};

TransformedAstsmcStep transformedConditionedAstsmcStep(float sliding, float state, float k1, float k2,
		float control_gain, float torque_limit, float dt)
{
	const float lambda = k2 - 0.25f * k1 * k1;
	const float sliding_sign = sliding > 0.f ? 1.f : (sliding < 0.f ? -1.f : 0.f);
	float reaching_offset;

	if (fabsf(sliding) > k2 * dt * dt) {
		reaching_offset = (2.f * lambda * dt + k1 * sqrtf(fabsf(sliding) - lambda * dt * dt)) * sliding_sign;

	} else {
		reaching_offset = 2.f * sliding / dt;
	}

	const float reaching_raw = state + reaching_offset;
	const float reaching_limit = control_gain * torque_limit;
	const float reaching = math::constrain(reaching_raw, -reaching_limit, reaching_limit);
	const float state_difference = reaching - state;
	float state_next;

	if (fabsf(state_difference) > 2.f * k2 * dt) {
		const float difference_sign = state_difference > 0.f ? 1.f : -1.f;
		state_next = state + dt * k2 * difference_sign;

	} else {
		state_next = 0.5f * (state + reaching);
	}

	return {reaching / control_gain, state_next};
}

TEST(RateControlTest, ImplicitSuperTwistingSignTransformMatchesCanonicalEquation18)
{
	struct TestCase {
		float sliding;
		float canonical_state;
		float k1;
		float k2;
		float control_gain;
		float torque_limit;
		float dt;
	};

	const TestCase test_cases[] {
		{0.01f, 0.f, 1.f, 1.f, 1.f, 1.f, 0.01f},
		{-0.01f, 0.2f, 1.f, 1.f, 2.f, 0.4f, 0.01f},
		{0.00005f, -0.1f, 1.f, 1.f, 4.f, 0.3f, 0.01f},
		{-0.0001f, 0.3f, 1.f, 1.f, 3.f, 0.2f, 0.01f},
		{0.4f, -0.5f, 10.f, 1.f, 5.f, 0.1f, 0.001f},
		{-0.4f, 0.5f, 10.f, 1.f, 5.f, 0.1f, 0.001f},
		{0.0004f, 0.02f, 2.f, 4.f, 7.f, 0.5f, 0.01f},
	};

	for (const TestCase &test_case : test_cases) {
		const float canonical_limit = test_case.control_gain * test_case.torque_limit;
		const CanonicalAstsmcStep canonical = canonicalConditionedAstsmcStep(test_case.sliding,
				test_case.canonical_state, test_case.k1, test_case.k2, canonical_limit, test_case.dt);
		const TransformedAstsmcStep transformed = transformedConditionedAstsmcStep(test_case.sliding,
				-test_case.canonical_state, test_case.k1, test_case.k2, test_case.control_gain,
				test_case.torque_limit, test_case.dt);

		EXPECT_NEAR(transformed.torque, -canonical.input / test_case.control_gain, 1e-6f);
		EXPECT_NEAR(transformed.state_next, -canonical.state_next, 1e-6f);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingProductionStepMatchesCanonicalEquation18)
{
	struct TestCase {
		float sliding;
		float canonical_state;
		float k1;
		float k2;
		float control_gain;
		float torque_limit;
		float dt;
	};

	const TestCase test_cases[] {
		{0.01f, 0.f, 1.f, 1.f, 1.f, 1.f, 0.01f},
		{-0.01f, 0.2f, 1.f, 1.f, 2.f, 0.4f, 0.01f},
		{0.00005f, -0.1f, 1.f, 1.f, 4.f, 0.3f, 0.01f},
		{-0.0001f, 0.3f, 1.f, 1.f, 3.f, 0.2f, 0.01f},
		{0.0001f, 0.02f, 2.f, 4.f, 7.f, 0.5f, 0.01f},
		{0.4f, -0.5f, 10.f, 1.f, 5.f, 0.1f, 0.001f},
		{-0.4f, 0.5f, 10.f, 1.f, 5.f, 0.1f, 0.001f},
	};

	for (const TestCase &test_case : test_cases) {
		const float canonical_limit = test_case.control_gain * test_case.torque_limit;
		const CanonicalAstsmcStep canonical = canonicalConditionedAstsmcStep(test_case.sliding,
				test_case.canonical_state, test_case.k1, test_case.k2, canonical_limit, test_case.dt);
		const RateControl::AstsmcScalarStep production = RateControl::implicitSuperTwistingStep(
				test_case.sliding, -test_case.canonical_state, test_case.k1, test_case.k2,
				canonical_limit, test_case.dt);

		EXPECT_NEAR(production.reaching_input_limited, -canonical.input, 1e-6f);
		EXPECT_NEAR(production.state_next, -canonical.state_next, 1e-6f);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingLargeBranchMatchesTransformedPublishedRecursion)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.quiet_k1_error_threshold = 0.05f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.01f;
	constexpr float sliding = 0.1f;
	const float lambda = parameters.k2(0) - 0.25f * parameters.k1(0) * parameters.k1(0);
	const float expected_reaching = 2.f * lambda * dt
					+ parameters.k1(0) * sqrtf(sliding - lambda * dt * dt);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(sliding, 0.f, 0.f), Vector3f(), dt, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_NEAR(torque(0), expected_reaching, 1e-6f);
	EXPECT_NEAR(status.reaching_input_raw[0], expected_reaching, 1e-6f);
	EXPECT_NEAR(status.integral_state[0], parameters.k2(0) * dt, 1e-6f);
	EXPECT_FALSE(status.internal_saturation[0]);
	EXPECT_TRUE(status.dt_valid);
}

TEST(RateControlTest, AstsmcQuietK1ScheduleBlendsOnlyRollPitchSmallErrors)
{
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.f, 0.5f, true), 3.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.25f, 0.5f, true), 3.f);
	EXPECT_NEAR(RateControl::scheduledAstsmcK1(4.5f, 0.375f, 0.5f, true), 3.75f, 1e-6f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.5f, 0.5f, true), 4.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 2.f, 0.5f, true), 4.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(1.5f, 0.f, 0.5f, false), 1.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.f, 0.5f, true, true), 1.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.1f, 0.5f, true, true), 1.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.2f, 0.5f, true, true), 3.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.25f, 0.5f, true, true), 3.f);
	EXPECT_NEAR(RateControl::scheduledAstsmcK1(4.5f, 0.15f, 0.5f, true, true), 2.25f, 1e-6f);
}

TEST(RateControlTest, AstsmcLargeErrorK1SchedulePreservesQuietGainAndAddsBoundedRecoveryBoost)
{
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.f, 0.5f, true, false, 1.5f, 1.f), 3.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.25f, 0.5f, true, false, 1.5f, 1.f), 3.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.5f, 0.5f, true, false, 1.5f, 1.f), 4.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 0.75f, 0.5f, true, false, 1.5f, 1.f), 5.25f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 1.f, 0.5f, true, false, 1.5f, 1.f), 6.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 2.f, 0.5f, true, false, 1.5f, 1.f), 6.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, -2.f, 0.5f, true, false, 1.5f, 1.f), 6.f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(4.5f, 2.f, 0.5f, true, false, 0.f, 1.f), 4.5f);
	EXPECT_FLOAT_EQ(RateControl::scheduledAstsmcK1(1.5f, 2.f, 0.5f, false, false, 1.5f, 1.f), 1.5f);
}

TEST(RateControlTest, AstsmcRollPitchResidualAuthorityReleasesOutsideQuietEnvelope)
{
	EXPECT_FLOAT_EQ(RateControl::astsmcRollPitchResidualAuthority(0.1f, 0.f, 2.f, 0.5f), 0.1f);
	EXPECT_FLOAT_EQ(RateControl::astsmcRollPitchResidualAuthority(0.1f, 0.1f, 0.f, 0.5f), 0.1f);
	EXPECT_NEAR(RateControl::astsmcRollPitchResidualAuthority(0.1f, 0.1f, 0.25f, 0.5f), 0.15f, 1e-6f);
	EXPECT_FLOAT_EQ(RateControl::astsmcRollPitchResidualAuthority(0.1f, 0.1f, 0.5f, 0.5f), 0.2f);
	EXPECT_FLOAT_EQ(RateControl::astsmcRollPitchResidualAuthority(0.1f, 0.1f, -2.f, 0.5f), 0.2f);
}

TEST(RateControlTest, ImplicitSuperTwistingReportsScheduledRollPitchResidualAuthority)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.roll_pitch_residual_extension = 0.1f;
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(0.25f, -0.5f, 0.5f), Vector3f(), 0.004f, false, 0.004f);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_NEAR(status.sliding_variable[0], 0.25f, 1e-6f);
	EXPECT_NEAR(status.sliding_variable[1], -0.5f, 1e-6f);
	EXPECT_NEAR(status.residual_authority[0], 0.15f, 1e-6f);
	EXPECT_FLOAT_EQ(status.residual_authority[1], 0.2f);
	EXPECT_FLOAT_EQ(status.residual_authority[2], 0.1f);
}

TEST(RateControlTest, ImplicitSuperTwistingQuietK1SchedulingReducesSmallErrorReaching)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float sliding = 0.04f;
	constexpr float quiet_k1 = 3.f;
	const float expected_k1 = quiet_k1;
	const float lambda = parameters.k2(0) - 0.25f * expected_k1 * expected_k1;
	const float expected_reaching = 2.f * lambda * dt
					+ expected_k1 * sqrtf(sliding - lambda * dt * dt);
	const float full_lambda = parameters.k2(0) - 0.25f * parameters.k1(0) * parameters.k1(0);
	const float full_k1_reaching = 2.f * full_lambda * dt
					     + parameters.k1(0) * sqrtf(sliding - full_lambda * dt * dt);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(sliding, 0.f, 0.f), Vector3f(), dt, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_NEAR(torque(0), expected_reaching, 1e-6f);
	EXPECT_NEAR(status.reaching_input_raw[0], expected_reaching, 1e-6f);
	EXPECT_LT(torque(0), full_k1_reaching);
	EXPECT_FLOAT_EQ(status.k1[0], parameters.k1(0));
	EXPECT_EQ(status.total_bound_violation_count, 0u);
}

TEST(RateControlTest, ImplicitSuperTwistingQuietK1SchedulingPreservesYaw)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.roll_pitch_k1_recovery_boost = 1.5f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float sliding = 2.f;
	const float lambda = parameters.k2(2) - 0.25f * parameters.k1(2) * parameters.k1(2);
	const float expected_reaching = 2.f * lambda * dt
					+ parameters.k1(2) * sqrtf(sliding - lambda * dt * dt);
	const float reaching_limit = parameters.control_effectiveness(2) / parameters.inertia(2)
				     * parameters.residual_torque_limit(2);
	const float expected_limited_reaching = math::constrain(expected_reaching, -reaching_limit, reaching_limit);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.f, 0.f, sliding), Vector3f(), dt, false);

	EXPECT_NEAR(torque(2), expected_limited_reaching, 1e-6f);
}

TEST(RateControlTest, ImplicitSuperTwistingSmallBranchMatchesTransformedPublishedRecursion)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.01f;
	constexpr float sliding = 0.00005f;
	constexpr float expected_reaching = 2.f * sliding / dt;
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(sliding, 0.f, 0.f), Vector3f(), dt, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_NEAR(torque(0), expected_reaching, 1e-7f);
	EXPECT_NEAR(status.integral_state[0], 0.5f * expected_reaching, 1e-7f);
	EXPECT_FALSE(status.internal_saturation[0]);
}

TEST(RateControlTest, ImplicitSuperTwistingRecoversStaleWrongDirectionState)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(1.5f, 1.5f, 1.5f);
	parameters.k2 = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.inertia = Vector3f(0.035f, 0.035f, 0.035f);
	parameters.control_effectiveness = Vector3f(7.f, 7.f, 7.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.recovery_error_threshold = 1.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 7000; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	astsmc_status_s before{};
	rate_control.getAstsmcStatus(before);
	ASSERT_GT(before.integral_state[0], 2.f);

	Vector3f torque;

	for (int sample = 0; sample < 1000; sample++) {
		torque = rate_control.update(Vector3f(), Vector3f(-2.f, 0.f, 0.f), Vector3f(), 0.001f, false);

		astsmc_safety_status_s interim{};
		rate_control.getAstsmcSafetyStatus(interim);

		if (interim.state_recovery_count[0] > 0) {
			break;
		}
	}
	astsmc_safety_status_s after{};
	rate_control.getAstsmcSafetyStatus(after);

	EXPECT_LT(torque(0), 0.f);
	EXPECT_TRUE(after.state_recovery_active[0]);
	EXPECT_FALSE(after.state_recovery_active[1]);
	EXPECT_FALSE(after.state_recovery_active[2]);
	EXPECT_EQ(after.state_recovery_count[0], 1u);
	EXPECT_GT(after.recovery_trigger_state[0], 3.f);
	EXPECT_LE(after.recovery_trigger_state[0], before.integral_state[0]);
	EXPECT_LT(after.recovery_trigger_sliding[0], -parameters.recovery_error_threshold);
	EXPECT_GT(after.recovery_trigger_reaching[0], 0.f);
	EXPECT_TRUE(after.runtime_fault_latched);
	EXPECT_NE(after.runtime_fault_reason & astsmc_safety_status_s::RUNTIME_FAULT_STATE_RECOVERY, 0u);
}

TEST(RateControlTest, ImplicitSuperTwistingDoesNotRecoverBelowThreshold)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(1.5f, 1.5f, 1.5f);
	parameters.k2 = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.inertia = Vector3f(0.035f, 0.035f, 0.035f);
	parameters.control_effectiveness = Vector3f(7.f, 7.f, 7.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.recovery_error_threshold = 1.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 7000; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	rate_control.update(Vector3f(), Vector3f(-0.5f, 0.f, 0.f), Vector3f(), 0.001f, false);
	astsmc_safety_status_s status{};
	rate_control.getAstsmcSafetyStatus(status);
	EXPECT_EQ(status.state_recovery_count[0], 0u);
	EXPECT_FALSE(status.runtime_fault_latched);
}

TEST(RateControlTest, ImplicitSuperTwistingGroundContainmentOverridesInvalidTiming)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.002f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	Vector3f airborne;

	for (int sample = 0; sample < 100; sample++) {
		airborne = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	astsmc_status_s airborne_status{};
	rate_control.getAstsmcStatus(airborne_status);
	ASSERT_NE(airborne, Vector3f());
	ASSERT_NE(airborne_status.integral_state[0], 0.f);

	const Vector3f contained = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f,
				  false, 0.01f, true);
	astsmc_safety_status_s contained_safety_status{};
	astsmc_status_s contained_status{};
	rate_control.getAstsmcSafetyStatus(contained_safety_status);
	rate_control.getAstsmcStatus(contained_status);

	EXPECT_EQ(contained, Vector3f());
	EXPECT_EQ(rate_control.getControllerType(), 3);
	EXPECT_TRUE(contained_safety_status.ground_containment_active);
	EXPECT_EQ(contained_safety_status.ground_containment_count, 1u);
	EXPECT_FALSE(contained_safety_status.runtime_fault_latched);
	EXPECT_EQ(contained_safety_status.runtime_fault_reason, 0u);
	EXPECT_EQ(contained_status.invalid_dt_hold_count, 0u);
	EXPECT_EQ(contained_status.total_bound_violation_count, 0u);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_EQ(contained_safety_status.state_recovery_count[axis], 0u);
		EXPECT_FLOAT_EQ(contained_status.integral_state[axis], 0.f);
		EXPECT_FLOAT_EQ(contained_status.torque_limited[axis], 0.f);
		EXPECT_FLOAT_EQ(contained_status.actuator_state[axis], 0.f);
		EXPECT_FLOAT_EQ(contained_status.applied_torque_target[axis], 0.f);
	}

	EXPECT_EQ(rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false, 0.01f, true), Vector3f());
	rate_control.getAstsmcSafetyStatus(contained_safety_status);
	EXPECT_EQ(contained_safety_status.ground_containment_count, 1u);

	rate_control.update(Vector3f(0.3f, 0.f, 0.f), Vector3f(), Vector3f(), 0.001f, false, 0.001f, false);
	astsmc_safety_status_s released_safety_status{};
	astsmc_status_s released_status{};
	rate_control.getAstsmcSafetyStatus(released_safety_status);
	rate_control.getAstsmcStatus(released_status);
	EXPECT_FALSE(released_safety_status.ground_containment_active);
	EXPECT_NEAR(released_status.rate_setpoint_shaped[0], 0.3f, 1e-3f);
	EXPECT_EQ(released_safety_status.ground_containment_count, 1u);
	EXPECT_FALSE(released_safety_status.runtime_fault_latched);
	EXPECT_EQ(released_safety_status.runtime_fault_reason, 0u);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_EQ(released_safety_status.state_recovery_count[axis], 0u);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingRuntimeFaultSurvivesRoutineReset)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	rate_control.latchAstsmcRuntimeFault(astsmc_safety_status_s::RUNTIME_FAULT_CONFIGURATION);
	rate_control.resetIntegral();
	astsmc_safety_status_s status{};
	rate_control.getAstsmcSafetyStatus(status);
	EXPECT_TRUE(status.runtime_fault_latched);
	EXPECT_NE(status.runtime_fault_reason & astsmc_safety_status_s::RUNTIME_FAULT_CONFIGURATION, 0u);
}

TEST(RateControlTest, ImplicitSuperTwistingConditionsStateOnInternalSaturation)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(10.f, 10.f, 10.f);
	parameters.torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.residual_torque_limit = parameters.torque_limit;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	Vector3f torque;

	for (int sample = 0; sample < 500; sample++) {
		torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_FLOAT_EQ(torque(0), 0.1f);
	EXPECT_TRUE(status.internal_saturation[0]);
	EXPECT_NEAR(status.integral_state[0], 0.1f, 0.002f);
	EXPECT_LE(fabsf(status.integral_state[0]), 0.102f);
}

TEST(RateControlTest, ImplicitSuperTwistingAcceptsClosedTimingBoundaries)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_min = 0.002f;
	parameters.dt_max = 0.004f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), parameters.dt_min, false,
			    parameters.dt_min);
	astsmc_status_s minimum_status{};
	rate_control.getAstsmcStatus(minimum_status);
	EXPECT_TRUE(minimum_status.dt_valid);
	EXPECT_FLOAT_EQ(minimum_status.accepted_dt, parameters.dt_min);
	EXPECT_FLOAT_EQ(minimum_status.dt_min, parameters.dt_min);
	EXPECT_FLOAT_EQ(minimum_status.dt_max, parameters.dt_max);

	rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), parameters.dt_max, false,
			    parameters.dt_max);
	astsmc_status_s maximum_status{};
	rate_control.getAstsmcStatus(maximum_status);
	EXPECT_TRUE(maximum_status.dt_valid);
	EXPECT_FLOAT_EQ(maximum_status.accepted_dt, parameters.dt_max);
	EXPECT_EQ(maximum_status.valid_update_count, minimum_status.valid_update_count + 1);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsTimingOutsideClosedInterval)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_min = 0.002f;
	parameters.dt_max = 0.004f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f valid_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.003f,
				      false, 0.003f);
	astsmc_status_s before{};
	rate_control.getAstsmcStatus(before);

	const Vector3f short_hold = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.001f,
				    false, 0.001f);
	astsmc_status_s after_short{};
	rate_control.getAstsmcStatus(after_short);
	EXPECT_EQ(short_hold, valid_torque);
	EXPECT_FALSE(after_short.dt_valid);
	EXPECT_EQ(after_short.valid_update_count, before.valid_update_count);
	EXPECT_FLOAT_EQ(after_short.rate_setpoint_shaped[0], before.rate_setpoint_shaped[0]);
	EXPECT_FLOAT_EQ(after_short.reference_acceleration[0], before.reference_acceleration[0]);
	EXPECT_FLOAT_EQ(after_short.integral_state[0], before.integral_state[0]);

	const Vector3f long_hold = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.005f,
				   false, 0.005f);
	astsmc_status_s after_long{};
	rate_control.getAstsmcStatus(after_long);
	EXPECT_EQ(long_hold, valid_torque);
	EXPECT_FALSE(after_long.dt_valid);
	EXPECT_EQ(after_long.valid_update_count, before.valid_update_count);
	EXPECT_EQ(after_long.invalid_dt_hold_count, before.invalid_dt_hold_count + 2);
	EXPECT_FLOAT_EQ(after_long.rate_setpoint_shaped[0], before.rate_setpoint_shaped[0]);
	EXPECT_FLOAT_EQ(after_long.reference_acceleration[0], before.reference_acceleration[0]);
	EXPECT_FLOAT_EQ(after_long.integral_state[0], before.integral_state[0]);
}

TEST(RateControlTest, ImplicitSuperTwistingInvalidDtHoldsBoundedCommandWithoutFallback)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.002f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f valid_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.001f, false);
	const Vector3f held_torque = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(held_torque, valid_torque);
	EXPECT_FALSE(status.dt_valid);
	EXPECT_EQ(rate_control.getControllerType(), 3);
}

TEST(RateControlTest, ImplicitSuperTwistingLandedInvalidDtReturnsZero)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.002f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f airborne_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.001f, false);
	const Vector3f landed_torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, true);

	EXPECT_GT(airborne_torque(0), 0.f);
	EXPECT_EQ(landed_torque, Vector3f());
}

TEST(RateControlTest, ImplicitSuperTwistingLandedUpdateResetsDynamicState)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 100; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	const Vector3f landed_torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, true);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(landed_torque, Vector3f());
	EXPECT_FALSE(status.dt_valid);
	EXPECT_FLOAT_EQ(status.integral_state[0], 0.f);
	EXPECT_FLOAT_EQ(status.integral_state[1], 0.f);
	EXPECT_FLOAT_EQ(status.integral_state[2], 0.f);
}

TEST(RateControlTest, ImplicitSuperTwistingLandedResetCountsNextReferenceInitializationOnce)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	ASSERT_EQ(status.reference_reset_count, 1u);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, true);
	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, true);
	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, false);
	rate_control.getAstsmcStatus(status);
	EXPECT_EQ(status.reference_reset_count, 2u);
}

TEST(RateControlTest, ImplicitSuperTwistingControllerSwitchIsBumpless)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(0.2f, 0.2f, 0.2f), Vector3f(), Vector3f());
	const Vector3f pid_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.001f, false);

	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));
	const Vector3f astsmc_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.001f, false);

	EXPECT_NEAR(astsmc_torque(0), pid_torque(0), 1e-6f);
	EXPECT_FLOAT_EQ(astsmc_torque(1), pid_torque(1));
	EXPECT_FLOAT_EQ(astsmc_torque(2), pid_torque(2));
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsConstantMatchedDisturbance)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(2.f, 2.f, 2.f);
	parameters.k2 = Vector3f(2.f, 2.f, 2.f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.001f;
	constexpr float disturbance = 0.1f;
	Vector3f rate;
	Vector3f torque;

	for (int sample = 0; sample < 10000; sample++) {
		torque = rate_control.update(rate, Vector3f(), Vector3f(), dt, false);
		rate(0) += (torque(0) + disturbance) * dt;
	}

	EXPECT_NEAR(rate(0), 0.f, 0.002f);
	EXPECT_NEAR(torque(0), -disturbance, 0.002f);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvertedTimingCardAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_min = 0.002f;
	parameters.dt_max = 0.004f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f valid_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.003f,
				      false, 0.003f);
	parameters.dt_min = 0.006f;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));

	const Vector3f held_torque = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.005f,
				     false, 0.005f);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(held_torque, valid_torque);
	EXPECT_FALSE(status.dt_valid);
	EXPECT_FLOAT_EQ(status.dt_min, 0.002f);
	EXPECT_FLOAT_EQ(status.dt_max, 0.004f);
	EXPECT_TRUE(rate_control.astsmcParametersValid());
}

TEST(RateControlTest, ImplicitSuperTwistingInvalidCardRetainsLastValidParameters)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));
	const Vector3f valid_torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.001f, false);

	parameters.k1 = Vector3f(NAN, NAN, NAN);
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.001f, false);

	EXPECT_TRUE(torque.isAllFinite());
	EXPECT_EQ(rate_control.getControllerType(), 3);
	EXPECT_TRUE(rate_control.astsmcParametersValid());
	EXPECT_GT(valid_torque(0), 0.f);
}


TEST(RateControlTest, ImplicitSuperTwistingReferenceRespectsAccelerationAndJerkLimits)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.reference_acceleration_limit = Vector3f(2.f, 3.f, 4.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 20.f, 30.f);
	parameters.residual_torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 500; sample++) {
		rate_control.update(Vector3f(), Vector3f(5.f, -5.f, 5.f), Vector3f(), 0.004f, false);
		astsmc_status_s status{};
		rate_control.getAstsmcStatus(status);

		for (int axis = 0; axis < 3; axis++) {
			EXPECT_LE(fabsf(status.reference_acceleration[axis]), parameters.reference_acceleration_limit(axis) + 1e-5f);
			EXPECT_LE(fabsf(status.reference_jerk[axis]), parameters.reference_jerk_limit(axis) + 1e-5f);
		}
	}
}

TEST(RateControlTest, ImplicitSuperTwistingTrackingBlendAddsBoundedReferenceGapRecovery)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.reference_acceleration_limit = Vector3f(1.f, 1.f, 1.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 10.f, 10.f);
	parameters.sliding_boundary = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.tracking_blend = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	const float reference_gap = status.rate_setpoint_raw[0] - status.rate_setpoint_shaped[0];
	const float expected_correction = parameters.tracking_blend * parameters.sliding_boundary(0)
					  * tanhf(reference_gap / parameters.sliding_boundary(0));

	EXPECT_FLOAT_EQ(status.sliding_variable_raw[0], 1.f);
	EXPECT_NEAR(status.sliding_variable[0], status.rate_setpoint_shaped[0] + expected_correction, 1e-6f);
	EXPECT_LE(status.sliding_variable[0] - status.rate_setpoint_shaped[0], 0.1f + 1e-6f);
}

TEST(RateControlTest, ImplicitSuperTwistingTrackingBlendDoesNotAmplifyMeasurementNoise)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.sliding_boundary = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.tracking_blend = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.update(Vector3f(0.1f, 0.f, 0.f), Vector3f(), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_FLOAT_EQ(status.rate_setpoint_raw[0], status.rate_setpoint_shaped[0]);
	EXPECT_FLOAT_EQ(status.sliding_variable[0], status.rate_setpoint_shaped[0] - 0.1f);
}

TEST(RateControlTest, ImplicitSuperTwistingZeroTrackingBlendPreservesShapedSurface)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.sliding_boundary = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.tracking_blend = 0.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_FLOAT_EQ(status.sliding_variable[0], status.rate_setpoint_shaped[0]);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvalidQuietK1ThresholdAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	parameters.quiet_k1_error_threshold = 1.1f * parameters.recovery_error_threshold;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_TRUE(status.configuration_valid);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvalidRollPitchK1RecoveryBoostAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.recovery_error_threshold = 1.f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f before = rate_control.update(Vector3f(), Vector3f(0.75f, 0.f, 0.f),
				Vector3f(), 0.004f, false, 0.004f);

	for (float invalid_boost : {-0.01f, 10.01f, NAN}) {
		parameters.roll_pitch_k1_recovery_boost = invalid_boost;
		EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	}

	parameters.roll_pitch_k1_recovery_boost = 1.5f;
	parameters.quiet_k1_error_threshold = parameters.recovery_error_threshold;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));

	rate_control.resetAstsmcState();
	const Vector3f after = rate_control.update(Vector3f(), Vector3f(0.75f, 0.f, 0.f),
			       Vector3f(), 0.004f, false, 0.004f);
	EXPECT_EQ(after, before);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvalidRollPitchResidualExtensionAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.roll_pitch_residual_extension = 0.05f;
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.tracking_blend = 1.f;
	parameters.variation_weight.zero();
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(0.5f, -0.5f, 0.f), Vector3f(), 0.004f, false, 0.004f);
	astsmc_status_s before{};
	rate_control.getAstsmcStatus(before);
	ASSERT_FLOAT_EQ(before.residual_authority[0], 0.15f);
	ASSERT_FLOAT_EQ(before.residual_authority[1], 0.15f);

	for (float invalid_extension : {0.11f, NAN}) {
		parameters.roll_pitch_residual_extension = invalid_extension;
		EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	}

	astsmc_status_s after{};
	rate_control.getAstsmcStatus(after);
	EXPECT_TRUE(after.configuration_valid);
	EXPECT_FLOAT_EQ(after.residual_authority[0], before.residual_authority[0]);
	EXPECT_FLOAT_EQ(after.residual_authority[1], before.residual_authority[1]);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvalidVariationWeightAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.variation_weight = Vector3f(4.f, 0.f, 0.f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	parameters.variation_weight(0) = NAN;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_FLOAT_EQ(status.variation_weight[0], 4.f);
}

TEST(RateControlTest, ImplicitSuperTwistingRejectsInvalidTrackingCardAtomically)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.sliding_boundary = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.tracking_blend = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	parameters.tracking_blend = 1.1f;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_GT(status.sliding_variable[0], status.rate_setpoint_shaped[0]);
	EXPECT_LE(status.sliding_variable[0] - status.rate_setpoint_shaped[0], 0.1f + 1e-6f);
}

TEST(RateControlTest, ImplicitSuperTwistingReferenceHandlesAlternatingValidTimingWithoutImpulses)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.reference_acceleration_limit = Vector3f(2.f, 2.f, 2.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 10.f, 10.f);
	parameters.dt_max = 0.01f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.004f, false);
	float previous_acceleration = 0.f;
	float previous_shaped_setpoint = 0.f;

	for (int sample = 0; sample < 200; sample++) {
		const float dt = sample % 2 == 0 ? 0.004f : 0.008f;
		rate_control.update(Vector3f(), Vector3f(5.f, 0.f, 0.f), Vector3f(), dt, false, dt);
		astsmc_status_s status{};
		rate_control.getAstsmcStatus(status);

		EXPECT_TRUE(status.dt_valid);
		EXPECT_FLOAT_EQ(status.accepted_dt, dt);
		EXPECT_LE(fabsf(status.reference_acceleration[0]), parameters.reference_acceleration_limit(0) + 1e-5f);
		EXPECT_LE(fabsf(status.reference_jerk[0]), parameters.reference_jerk_limit(0) + 1e-5f);
		EXPECT_LE(fabsf(status.reference_acceleration[0] - previous_acceleration),
			  parameters.reference_jerk_limit(0) * dt + 1e-5f);
		EXPECT_GE(status.rate_setpoint_shaped[0] + 1e-6f, previous_shaped_setpoint);
		previous_acceleration = status.reference_acceleration[0];
		previous_shaped_setpoint = status.rate_setpoint_shaped[0];
	}
}

TEST(RateControlTest, ImplicitSuperTwistingReferenceDoesNotClampValidHighRateTarget)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.reference_acceleration_limit = Vector3f(1000.f, 1000.f, 1000.f);
	parameters.reference_jerk_limit = Vector3f(10000.f, 10000.f, 10000.f);
	parameters.dt_max = 0.02f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 200; sample++) {
		rate_control.update(Vector3f(), Vector3f(120.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	}

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_NEAR(status.rate_setpoint_shaped[0], 120.f, 1e-4f);
}

TEST(RateControlTest, ImplicitSuperTwistingInvalidDtHoldsAllDynamicStateAndCountsEvent)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.005f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f torque_before = rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.004f, false);
	astsmc_status_s before{};
	rate_control.getAstsmcStatus(before);
	const Vector3f held = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.008f, false, 0.008f);
	astsmc_status_s after{};
	rate_control.getAstsmcStatus(after);

	EXPECT_EQ(held, torque_before);
	EXPECT_FALSE(after.dt_valid);
	EXPECT_TRUE(std::isnan(after.accepted_dt));
	EXPECT_FLOAT_EQ(after.raw_dt, 0.008f);
	EXPECT_EQ(after.valid_update_count, before.valid_update_count);
	EXPECT_EQ(after.invalid_dt_hold_count, before.invalid_dt_hold_count + 1);
	EXPECT_EQ(after.consecutive_invalid_dt_hold_count, before.consecutive_invalid_dt_hold_count);
	EXPECT_EQ(after.reference_reset_count, before.reference_reset_count);
	EXPECT_FLOAT_EQ(after.rate_setpoint_shaped[0], before.rate_setpoint_shaped[0]);
	EXPECT_FLOAT_EQ(after.sliding_variable_raw[0], before.sliding_variable_raw[0]);
	EXPECT_FLOAT_EQ(after.sliding_variable[0], before.sliding_variable[0]);
	EXPECT_FLOAT_EQ(after.reference_acceleration[0], before.reference_acceleration[0]);
	EXPECT_FLOAT_EQ(after.integral_state[0], before.integral_state[0]);
}

TEST(RateControlTest, ImplicitSuperTwistingConsecutiveInvalidDtCounterRecordsHiddenSequence)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.005f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.004f, false);
	rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.008f, false, 0.008f);
	rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.008f, false, 0.008f);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(status.valid_update_count, 1u);
	EXPECT_EQ(status.invalid_dt_hold_count, 2u);
	EXPECT_EQ(status.consecutive_invalid_dt_hold_count, 1u);

	rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.004f, false);
	rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.008f, false, 0.008f);
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(status.invalid_dt_hold_count, 3u);
	EXPECT_EQ(status.consecutive_invalid_dt_hold_count, 1u);
}

TEST(RateControlTest, ImplicitSuperTwistingContextResetPreservesOutputAndReseedsState)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.reference_acceleration_limit = Vector3f(2.f, 2.f, 2.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 10.f, 10.f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	Vector3f previous_torque;

	for (int sample = 0; sample < 50; sample++) {
		previous_torque = rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	astsmc_status_s before{};
	rate_control.getAstsmcStatus(before);
	ASSERT_NE(before.integral_state[0], 0.f);
	rate_control.resetSetpointHistory();
	astsmc_safety_status_s safety_reset{};
	rate_control.getAstsmcSafetyStatus(safety_reset);
	const Vector3f reseeded_torque = rate_control.update(Vector3f(), Vector3f(-0.5f, 0.f, 0.f), Vector3f(), 0.001f,
					 false);
	astsmc_status_s after{};
	rate_control.getAstsmcStatus(after);

	EXPECT_NEAR(reseeded_torque(0), previous_torque(0), 1e-6f);
	EXPECT_FALSE(safety_reset.trim_valid[0]);
	EXPECT_FALSE(safety_reset.trim_valid[1]);
	EXPECT_FLOAT_EQ(safety_reset.trim_confidence[0], 0.f);
	EXPECT_FLOAT_EQ(safety_reset.trim_confidence[1], 0.f);
	EXPECT_EQ(after.reference_reset_count, before.reference_reset_count + 1);
	EXPECT_NEAR(after.rate_setpoint_shaped[0], 0.f, 1e-5f);
	EXPECT_NE(after.integral_state[0], 0.f);
}

TEST(RateControlTest, ImplicitSuperTwistingControlDisableClearsStaleOutput)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f airborne_torque = rate_control.update(Vector3f(), Vector3f(0.5f, 0.f, 0.f), Vector3f(), 0.001f,
					 false);
	ASSERT_GT(airborne_torque(0), 0.f);

	rate_control.resetSetpointHistoryForControlDisable();
	const Vector3f reenabled_torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.001f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_EQ(reenabled_torque, Vector3f());
	EXPECT_TRUE(status.dt_valid);
	EXPECT_FLOAT_EQ(status.integral_state[0], 0.f);
	EXPECT_FLOAT_EQ(status.torque_limited[0], 0.f);
}

TEST(RateControlTest, ImplicitSuperTwistingControllerSwitchHoldsSeedAcrossInvalidFirstDt)
{
	RateControl rate_control;
	rate_control.setPidGains(Vector3f(0.2f, 0.2f, 0.2f), Vector3f(), Vector3f());
	const Vector3f pid_torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.001f, false);
	AstsmcTestParameters parameters;
	parameters.dt_max = 0.002f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f held = rate_control.update(Vector3f(), Vector3f(-1.f, 0.f, 0.f), Vector3f(), 0.01f, false, 0.01f);
	EXPECT_EQ(held, pid_torque);
}

TEST(RateControlTest, ImplicitSuperTwistingModelAssistanceImprovesScalarRampTracking)
{
	const float model_assisted_error = simulateAstsmcScalarRamp(true);
	const float regulation_only_error = simulateAstsmcScalarRamp(false);

	EXPECT_LT(model_assisted_error, 0.7f * regulation_only_error);
}

TEST(RateControlTest, AstsmcVariationRegularizationPreservesLegacyBehaviorAtZeroWeight)
{
	EXPECT_FLOAT_EQ(RateControl::variationRegularizedAppliedTorque(0.08f, 0.02f, 0.1f, 0.f), 0.08f);
}

TEST(RateControlTest, AstsmcVariationRegularizationReducesSmallCommandChanges)
{
	const float regularized = RateControl::variationRegularizedAppliedTorque(0.03f, 0.02f, 0.1f, 4.f);

	EXPECT_GT(regularized, 0.02f);
	EXPECT_LT(regularized, 0.03f);
}

TEST(RateControlTest, AstsmcVariationRegularizationReleasesLargeSignalAuthority)
{
	EXPECT_FLOAT_EQ(RateControl::variationRegularizedAppliedTorque(0.12f, 0.02f, 0.1f, 100.f), 0.12f);
	EXPECT_FLOAT_EQ(RateControl::variationRegularizedAppliedTorque(-0.08f, 0.02f, 0.1f, 100.f), -0.08f);
}

TEST(RateControlTest, ImplicitSuperTwistingVariationRegularizationIsObservableAndBounded)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.k2 = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.variation_weight = Vector3f(4.f, 0.f, 0.f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.01f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_FLOAT_EQ(status.variation_weight[0], 4.f);
	EXPECT_LT(status.variation_regularization[0], 0.f);
	EXPECT_LE(fabsf(torque(0)), parameters.torque_limit(0) + 1e-6f);
	EXPECT_TRUE(status.total_bound_valid);
}

TEST(RateControlTest, ImplicitSuperTwistingUav985RigidBodyTrackingImprovesWithModelAssistance)
{
	for (float yaw_sign : {-1.f, 1.f}) {
		const RigidBodyTrackingMetrics model_assisted = simulateUav985Astsmc(true, yaw_sign);
		const RigidBodyTrackingMetrics regulation_only = simulateUav985Astsmc(false, yaw_sign);

		EXPECT_LT(model_assisted.yaw_absolute_error_integral,
			  0.85f * regulation_only.yaw_absolute_error_integral);
		EXPECT_LT(model_assisted.final_error_norm, 0.15f);
		EXPECT_EQ(model_assisted.invalid_dt_hold_count, 0u);
		EXPECT_EQ(model_assisted.total_bound_violation_count, 0u);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingRollPitchFeedforwardImprovesCommandRelease)
{
	for (int axis : {0, 1}) {
		for (float actual_gain_scale : {0.7f, 1.f, 1.3f}) {
			for (float load_sign : {-1.f, 1.f}) {
				const float matched_acceleration = load_sign * 2.f;
				const CommandReleaseTrackingMetrics previous =
					simulateRealRollPitchCommandRelease(axis, 0.f, actual_gain_scale, matched_acceleration);
				const CommandReleaseTrackingMetrics candidate =
					simulateRealRollPitchCommandRelease(axis, 0.75f, actual_gain_scale, matched_acceleration);

				EXPECT_LT(candidate.release_error_integral, 0.9f * previous.release_error_integral);
				EXPECT_LT(candidate.final_error, 0.1f);
				EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
				EXPECT_EQ(candidate.invalid_dt_hold_count, 0u);
				EXPECT_EQ(candidate.state_recovery_count, 0u);
				EXPECT_EQ(candidate.total_bound_violation_count, 0u);
				EXPECT_FALSE(candidate.runtime_fault_latched);
			}
		}
	}
}

TEST(RateControlTest, ImplicitSuperTwistingMeasuredRollPitchAuthorityImprovesLargeCommandTracking)
{
	const float previous_effectiveness[] {0.8f, 0.75f};
	const float identified_effectiveness[] {0.56f, 0.53f};

	for (int axis : {0, 1}) {
		const UnderAuthorityTrackingMetrics previous = simulateRealRollPitchUnderAuthority(
				axis, previous_effectiveness[axis], 3.f, 0.2f, 0.75f, 0.1f, identified_effectiveness[axis]);
		const UnderAuthorityTrackingMetrics candidate = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 3.f, 0.2f, 0.5f, 0.1f, identified_effectiveness[axis]);

		EXPECT_LT(candidate.absolute_error_integral, 0.9f * previous.absolute_error_integral);
		EXPECT_LE(candidate.peak_absolute_error, previous.peak_absolute_error + 1e-5f);
		EXPECT_LT(candidate.final_absolute_error, 0.1f);
		EXPECT_GT(candidate.maximum_absolute_torque, previous.maximum_absolute_torque);
		EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
		EXPECT_EQ(candidate.state_recovery_count, 0u);
		EXPECT_EQ(candidate.total_bound_violation_count, 0u);
		EXPECT_FALSE(candidate.runtime_fault_latched);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingRollPitchRecoveryAuthorityImprovesLargeCommandTracking)
{
	const float identified_effectiveness[] {0.56f, 0.53f};

	for (int axis : {0, 1}) {
		const UnderAuthorityTrackingMetrics previous = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 4.5f, 0.2f, 0.5f, 0.1f, identified_effectiveness[axis]);
		const UnderAuthorityTrackingMetrics candidate = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 4.5f, 0.2f, 0.5f, 0.1f,
				identified_effectiveness[axis], 0.1f);

		EXPECT_LT(candidate.absolute_error_integral, previous.absolute_error_integral);
		EXPECT_LE(candidate.peak_absolute_error, previous.peak_absolute_error + 0.002f);
		EXPECT_LT(candidate.final_absolute_error, 0.1f);
		EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
		EXPECT_EQ(candidate.state_recovery_count, 0u);
		EXPECT_EQ(candidate.total_bound_violation_count, 0u);
		EXPECT_FALSE(candidate.runtime_fault_latched);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingDisabledRollPitchExtensionPreservesTrajectory)
{
	RateControl baseline;
	RateControl explicit_zero;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.5f;
	parameters.gyro_compensation = 1.f;
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	parameters.roll_pitch_residual_extension = 0.f;
	ASSERT_TRUE(configureAstsmc(explicit_zero, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(explicit_zero.setControllerType(3));

	for (int sample = 0; sample < 600; sample++) {
		const float time = 0.004f * sample;
		const Vector3f rate(0.4f * sinf(1.7f * time), -0.3f * cosf(1.3f * time), 0.1f * sinf(time));
		const Vector3f setpoint(3.5f * sinf(0.8f * time), -3.5f * cosf(0.7f * time), 0.2f);
		const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		const Vector3f explicit_zero_torque = explicit_zero.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		EXPECT_EQ(explicit_zero_torque, baseline_torque);
	}

	astsmc_status_s baseline_status{};
	astsmc_status_s explicit_zero_status{};
	baseline.getAstsmcStatus(baseline_status);
	explicit_zero.getAstsmcStatus(explicit_zero_status);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_FLOAT_EQ(explicit_zero_status.residual_authority[axis], baseline_status.residual_authority[axis]);
		EXPECT_FLOAT_EQ(explicit_zero_status.integral_state[axis], baseline_status.integral_state[axis]);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingLargeErrorK1BoostImprovesRecoveryWithinTorqueBound)
{
	const float identified_effectiveness[] {0.56f, 0.53f};

	for (int axis : {0, 1}) {
		const UnderAuthorityTrackingMetrics previous = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 4.5f, 0.2f, 0.5f, 0.1f,
				identified_effectiveness[axis], 0.1f, 0.f);
		const UnderAuthorityTrackingMetrics candidate = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 4.5f, 0.2f, 0.5f, 0.1f,
				identified_effectiveness[axis], 0.1f, 1.5f);

		EXPECT_LT(candidate.absolute_error_integral, previous.absolute_error_integral);
		EXPECT_LE(candidate.peak_absolute_error, previous.peak_absolute_error + 0.002f);
		EXPECT_LT(candidate.final_absolute_error, 0.1f);
		EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
		EXPECT_EQ(candidate.state_recovery_count, 0u);
		EXPECT_EQ(candidate.total_bound_violation_count, 0u);
		EXPECT_FALSE(candidate.runtime_fault_latched);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingDisabledLargeErrorK1BoostPreservesTrajectory)
{
	RateControl baseline;
	RateControl explicit_zero;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.roll_pitch_residual_extension = 0.1f;
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.5f;
	parameters.gyro_compensation = 1.f;
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.recovery_error_threshold = 1.f;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	parameters.roll_pitch_k1_recovery_boost = 0.f;
	ASSERT_TRUE(configureAstsmc(explicit_zero, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(explicit_zero.setControllerType(3));

	for (int sample = 0; sample < 600; sample++) {
		const float time = 0.004f * sample;
		const Vector3f rate(0.4f * sinf(1.7f * time), -0.3f * cosf(1.3f * time), 0.1f * sinf(time));
		const Vector3f setpoint(3.5f * sinf(0.8f * time), -3.5f * cosf(0.7f * time), 0.2f);
		const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		const Vector3f explicit_zero_torque = explicit_zero.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		EXPECT_EQ(explicit_zero_torque, baseline_torque);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingHigherRollPitchK1ImprovesLargeCommandTracking)
{
	const float identified_effectiveness[] {0.56f, 0.53f};

	for (int axis : {0, 1}) {
		const UnderAuthorityTrackingMetrics previous = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 3.f, 0.2f, 0.5f, 0.1f, identified_effectiveness[axis]);
		const UnderAuthorityTrackingMetrics candidate = simulateRealRollPitchUnderAuthority(
				axis, identified_effectiveness[axis], 4.5f, 0.2f, 0.5f, 0.1f, identified_effectiveness[axis]);

		EXPECT_LT(candidate.absolute_error_integral, 0.92f * previous.absolute_error_integral);
		EXPECT_LE(candidate.peak_absolute_error, previous.peak_absolute_error + 0.002f);
		EXPECT_LT(candidate.final_absolute_error, 0.1f);
		EXPECT_GT(candidate.maximum_absolute_torque, previous.maximum_absolute_torque);
		EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
		EXPECT_EQ(candidate.state_recovery_count, 0u);
		EXPECT_EQ(candidate.total_bound_violation_count, 0u);
		EXPECT_FALSE(candidate.runtime_fault_latched);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingQuietAnchorMovesStateTowardLearnedTrim)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.quiet_k1_error_threshold = 0.5f;
	parameters.trim_confidence_time = 0.1f;
	parameters.trim_time_constant = 100.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float matched_acceleration = 0.6f;
	Vector3f rate;

	for (int sample = 0; sample < 1000; sample++) {
		const Vector3f torque = rate_control.update(rate, Vector3f(), Vector3f(), dt, false, dt);
		rate(0) += (parameters.control_effectiveness(0) / parameters.inertia(0) * torque(0)
			    + matched_acceleration) * dt;
	}

	astsmc_safety_status_s learned{};
	rate_control.getAstsmcSafetyStatus(learned);
	ASSERT_TRUE(learned.trim_valid[0]);
	const float learned_trim = learned.trim_state[0];

	for (int sample = 0; sample < 500; sample++) {
		rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), dt, false, dt);
	}

	astsmc_status_s displaced{};
	rate_control.getAstsmcStatus(displaced);
	ASSERT_GT(fabsf(displaced.integral_state[0] - learned_trim), 0.1f);
	float previous_distance = fabsf(displaced.integral_state[0] - learned_trim);
	bool observed_anchor = false;

	for (int sample = 0; sample < 1000; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), dt, false, dt);
		astsmc_status_s status{};
		astsmc_safety_status_s safety{};
		rate_control.getAstsmcStatus(status);
		rate_control.getAstsmcSafetyStatus(safety);

		if (safety.quiet_anchor_active[0]) {
			const float distance = fabsf(status.integral_state[0] - safety.trim_state[0]);

			if (observed_anchor) {
				EXPECT_LE(distance, previous_distance + 1e-5f);
			}

			observed_anchor = true;
			previous_distance = distance;
		}
	}

	astsmc_status_s final_status{};
	astsmc_safety_status_s final_safety{};
	rate_control.getAstsmcStatus(final_status);
	rate_control.getAstsmcSafetyStatus(final_safety);
	EXPECT_TRUE(observed_anchor);
	EXPECT_GT(final_safety.quiet_anchor_count[0], 0u);
	EXPECT_LT(fabsf(final_status.integral_state[0] - final_safety.trim_state[0]), 0.05f);
	EXPECT_NE(final_safety.trim_state[0], 0.f);
	EXPECT_TRUE(final_safety.deep_quiet_active[0]);
	EXPECT_EQ(final_safety.state_recovery_count[0], 0u);
	EXPECT_FALSE(final_safety.runtime_fault_latched);
}

TEST(RateControlTest, ImplicitSuperTwistingQuietAnchorRequiresValidTrimAndNeutralCommand)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.trim_confidence_time = 0.1f;
	parameters.quiet_k1_error_threshold = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 20; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.004f, false, 0.004f);
	}

	astsmc_safety_status_s before_trim{};
	rate_control.getAstsmcSafetyStatus(before_trim);
	EXPECT_FALSE(before_trim.trim_valid[0]);
	EXPECT_FALSE(before_trim.quiet_anchor_active[0]);
	EXPECT_FALSE(before_trim.deep_quiet_active[0]);
	EXPECT_EQ(before_trim.quiet_anchor_count[0], 0u);

	for (int sample = 0; sample < 100; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.004f, false, 0.004f);
	}

	astsmc_safety_status_s neutral{};
	rate_control.getAstsmcSafetyStatus(neutral);
	ASSERT_TRUE(neutral.trim_valid[0]);
	EXPECT_TRUE(neutral.quiet_anchor_active[0]);
	EXPECT_TRUE(neutral.deep_quiet_active[0]);
	const uint32_t anchor_count = neutral.quiet_anchor_count[0];

	rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.004f, false, 0.004f);
	astsmc_safety_status_s commanded{};
	rate_control.getAstsmcSafetyStatus(commanded);
	EXPECT_FALSE(commanded.quiet_anchor_active[0]);
	EXPECT_FALSE(commanded.deep_quiet_active[0]);
	EXPECT_EQ(commanded.quiet_anchor_count[0], anchor_count);
}

TEST(RateControlTest, ImplicitSuperTwistingSelectiveReleasePreservesTrimAndAvoidsHardRecovery)
{
	for (int axis : {0, 1}) {
		for (float actual_gain_scale : {0.7f, 1.f, 1.3f}) {
			for (float load_sign : {-1.f, 1.f}) {
				const float matched_acceleration = load_sign * 2.f;
				const CommandReleaseTrackingMetrics baseline =
					simulateRealRollPitchCommandRelease(axis, 0.75f, actual_gain_scale, matched_acceleration, false);
				const CommandReleaseTrackingMetrics candidate =
					simulateRealRollPitchCommandRelease(axis, 0.75f, actual_gain_scale, matched_acceleration, true);

				EXPECT_LE(candidate.release_error_integral, 1.1f * baseline.release_error_integral);
				EXPECT_LT(candidate.final_error, 0.1f);
				EXPECT_LE(candidate.maximum_absolute_torque, 0.2f + 1e-6f);
				EXPECT_EQ(candidate.invalid_dt_hold_count, 0u);
				EXPECT_EQ(candidate.state_recovery_count, 0u);
				EXPECT_EQ(candidate.total_bound_violation_count, 0u);
				EXPECT_FALSE(candidate.runtime_fault_latched);
			}
		}
	}
}

TEST(RateControlTest, ImplicitSuperTwistingTrimConfidenceSurvivesControllerRateSpikes)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.8f, 0.75f, 0.8f);
	parameters.k1 = Vector3f(3.f, 3.f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.75f;
	parameters.trim_command_threshold = 0.15f;
	parameters.trim_error_threshold = 0.2f;
	parameters.trim_acceleration_threshold = 1.f;
	parameters.trim_confidence_time = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	Vector3f rate;
	astsmc_safety_status_s safety_status{};

	for (int sample = 0; sample < 250; sample++) {
		Vector3f rate_setpoint;

		if (sample % 30 == 15) {
			rate_setpoint(0) = 0.18f;
			rate_setpoint(1) = -0.18f;
		}

		const Vector3f torque = rate_control.update(rate, rate_setpoint, Vector3f(), dt, false, dt);
		rate(0) += (80.f * torque(0) + 0.25f * sinf(0.37f * sample)) * dt;
		rate(1) += (75.f * torque(1) + 0.30f * sinf(0.29f * sample)) * dt;
		rate_control.getAstsmcSafetyStatus(safety_status);
	}

	EXPECT_TRUE(safety_status.trim_valid[0]);
	EXPECT_TRUE(safety_status.trim_valid[1]);
	EXPECT_GE(safety_status.trim_confidence[0], parameters.trim_confidence_time - dt);
	EXPECT_GE(safety_status.trim_confidence[1], parameters.trim_confidence_time - dt);
	EXPECT_EQ(safety_status.state_recovery_count[0], 0u);
	EXPECT_EQ(safety_status.state_recovery_count[1], 0u);
	EXPECT_FALSE(safety_status.runtime_fault_latched);
}

TEST(RateControlTest, ImplicitSuperTwistingTrimConfidenceRejectsSustainedCommand)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.trim_confidence_time = 0.5f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;

	for (int sample = 0; sample < 100; sample++) {
		Vector3f rate_setpoint;
		rate_setpoint(0) = sample == 50 ? 0.18f : 0.f;
		rate_control.update(Vector3f(), rate_setpoint, Vector3f(), dt, false, dt);
	}

	astsmc_safety_status_s before_command{};
	rate_control.getAstsmcSafetyStatus(before_command);
	EXPECT_FALSE(before_command.trim_valid[0]);
	EXPECT_GT(before_command.trim_confidence[0], 0.3f);

	for (int sample = 0; sample < 200; sample++) {
		rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), dt, false, dt);
	}

	astsmc_safety_status_s after_command{};
	rate_control.getAstsmcSafetyStatus(after_command);
	EXPECT_FALSE(after_command.trim_valid[0]);
	EXPECT_LT(after_command.trim_confidence[0], before_command.trim_confidence[0]);
}

TEST(RateControlTest, ImplicitSuperTwistingSelectiveReleaseCompletesAfterTriggerClears)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(1.5f, 1.5f, 1.5f);
	parameters.k2 = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.inertia = Vector3f(0.035f, 0.035f, 0.035f);
	parameters.control_effectiveness = Vector3f(7.f, 7.f, 7.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.trim_confidence_time = 0.1f;
	parameters.selective_release_error_threshold = 0.15f;
	parameters.selective_release_state_rate = 20.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 4000; sample++) {
		rate_control.update(Vector3f(0.01f, 0.f, 0.f), Vector3f(), Vector3f(), 0.001f, false);
	}

	astsmc_safety_status_s learned{};
	rate_control.getAstsmcSafetyStatus(learned);
	ASSERT_TRUE(learned.trim_valid[0]);

	for (int sample = 0; sample < 7000; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	bool triggered = false;
	float trigger_distance = 0.f;
	uint32_t trigger_count = 0;

	for (int sample = 0; sample < 2000; sample++) {
		const Vector3f target = triggered ? Vector3f() : Vector3f(-0.5f, 0.f, 0.f);
		rate_control.update(Vector3f(), target, Vector3f(), 0.001f, false);
		astsmc_status_s status{};
		astsmc_safety_status_s safety{};
		rate_control.getAstsmcStatus(status);
		rate_control.getAstsmcSafetyStatus(safety);

		if (!triggered && safety.selective_release_count[0] > 0) {
			if (!safety.selective_release_active[0]) {
				continue;
			}

			triggered = true;
			trigger_count = safety.selective_release_count[0];
			trigger_distance = fabsf(status.integral_state[0] - safety.trim_state[0]);
			ASSERT_GT(trigger_distance, 0.1f);
			continue;
		}

		if (triggered && safety.selective_release_active[0]) {
			const float control_gain = parameters.control_effectiveness(0) / parameters.inertia(0);
			const float trim_reaching = safety.trim_state[0]
						    + (status.sliding_variable[0] >= 0.f ? 1.f : -1.f)
						    * (2.f * parameters.k2(0) * 0.001f
						       + parameters.k1(0) * sqrtf(fabsf(status.sliding_variable[0])));
			EXPECT_NEAR(status.residual_torque_limited[0],
				    math::constrain(trim_reaching / control_gain, -parameters.residual_torque_limit(0),
						    parameters.residual_torque_limit(0)), 0.01f);
		}

		if (triggered && !safety.selective_release_active[0]) {
			EXPECT_EQ(safety.selective_release_count[0], trigger_count);
			EXPECT_LT(fabsf(status.integral_state[0] - safety.trim_state[0]), 0.02f)
				<< "trigger_distance=" << trigger_distance << " q=" << status.integral_state[0]
				<< " trim=" << safety.trim_state[0] << " sample=" << sample;
			EXPECT_EQ(safety.state_recovery_count[0], 0u);
			EXPECT_FALSE(safety.runtime_fault_latched);
			return;
		}
	}

	FAIL() << "selective release did not complete after its trigger cleared";
}

TEST(RateControlTest, ImplicitSuperTwistingSelectiveReleaseMovesTowardNonzeroTrim)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.k1 = Vector3f(1.5f, 1.5f, 1.5f);
	parameters.k2 = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.inertia = Vector3f(0.035f, 0.035f, 0.035f);
	parameters.control_effectiveness = Vector3f(7.f, 7.f, 7.f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.trim_confidence_time = 0.1f;
	parameters.trim_time_constant = 20.f;
	parameters.selective_release_error_threshold = 0.15f;
	parameters.selective_release_state_rate = 20.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	for (int sample = 0; sample < 4000; sample++) {
		rate_control.update(Vector3f(0.01f, 0.f, 0.f), Vector3f(), Vector3f(), 0.001f, false);
	}

	astsmc_safety_status_s learned{};
	rate_control.getAstsmcSafetyStatus(learned);
	ASSERT_TRUE(learned.trim_valid[0]);
	ASSERT_NE(learned.trim_state[0], 0.f);

	for (int sample = 0; sample < 7000; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.001f, false);
	}

	for (int sample = 0; sample < 1000; sample++) {
		rate_control.update(Vector3f(), Vector3f(-0.5f, 0.f, 0.f), Vector3f(), 0.001f, false);
		astsmc_safety_status_s status{};
		rate_control.getAstsmcSafetyStatus(status);

		if (status.selective_release_count[0] > 0) {
			EXPECT_NEAR(status.selective_trigger_trim[0], learned.trim_state[0], 0.1f);
			EXPECT_NE(status.selective_trigger_trim[0], 0.f);
			EXPECT_EQ(status.state_recovery_count[0], 0u);
			EXPECT_FALSE(status.runtime_fault_latched);
			return;
		}
	}

	FAIL() << "selective release did not trigger";
}

TEST(RateControlTest, ImplicitSuperTwistingRealYawCardRejectsBoundedUncertainty)
{
	for (float actual_effectiveness : {0.55f, 0.8f}) {
		for (float load_sign : {-1.f, 1.f}) {
			const float matched_load = load_sign * 0.035f;
			const UncertainYawTrackingMetrics previous =
				simulateUncertainRealYawCard(1.163f, 0.05f, actual_effectiveness, matched_load);
			const UncertainYawTrackingMetrics candidate =
				simulateUncertainRealYawCard(0.8f, 0.1f, actual_effectiveness, matched_load);

			EXPECT_LT(candidate.absolute_error_integral, 0.75f * previous.absolute_error_integral);
			EXPECT_LT(candidate.final_absolute_error, 0.08f);
			EXPECT_LE(candidate.maximum_absolute_torque, 0.15f + 1e-6f);
			EXPECT_EQ(candidate.invalid_dt_hold_count, 0u);
			EXPECT_EQ(candidate.state_recovery_count, 0u);
			EXPECT_EQ(candidate.total_bound_violation_count, 0u);
			EXPECT_FALSE(candidate.runtime_fault_latched);
		}
	}
}

TEST(RateControlTest, ImplicitSuperTwistingRollPitchFeedforwardPreservesBiasRejection)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.8f, 0.75f, 0.8f);
	parameters.k1 = Vector3f(3.f, 3.f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.75f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	constexpr float dt = 0.004f;
	constexpr float matched_acceleration = 2.f;
	Vector3f rate;
	Vector3f torque;

	for (int sample = 0; sample < 2500; sample++) {
		Vector3f target;

		if (sample >= 100 && sample < 350) {
			target(0) = 2.f;
		}

		torque = rate_control.update(rate, target, Vector3f(), dt, false, dt);
		rate(0) += (parameters.control_effectiveness(0) / parameters.inertia(0) * torque(0)
			    + matched_acceleration) * dt;
	}

	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_NEAR(rate(0), 0.f, 0.02f);
	EXPECT_NEAR(torque(0), -matched_acceleration * parameters.inertia(0)
		    / parameters.control_effectiveness(0), 0.005f);
	EXPECT_NEAR(status.residual_authority[0], 0.1f, 1e-6f);
	EXPECT_FLOAT_EQ(status.nominal_torque_limited[0], 0.f);
	EXPECT_GT(fabsf(status.integral_state[0]), 0.5f);
	EXPECT_TRUE(status.total_bound_valid);
}

TEST(RateControlTest, ImplicitSuperTwistingNominalReferenceTorqueAndAuthorityPartition)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(2.f, 2.f, 2.f);
	parameters.control_effectiveness = Vector3f(4.f, 4.f, 4.f);
	parameters.torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.residual_torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.reference_acceleration_limit = Vector3f(1.f, 1.f, 1.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 10.f, 10.f);
	parameters.reference_feedforward = 1.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	const float expected_nominal_torque = parameters.reference_feedforward * parameters.inertia(0)
					      * status.reference_acceleration[0] / parameters.control_effectiveness(0);

	EXPECT_GT(status.reference_acceleration[0], 0.f);
	EXPECT_LE(status.reference_acceleration[0], parameters.reference_acceleration_limit(0));
	EXPECT_NEAR(status.nominal_torque_raw[0], expected_nominal_torque, 1e-5f);
	EXPECT_NEAR(status.nominal_torque_limited[0], expected_nominal_torque, 1e-5f);
	EXPECT_NEAR(status.residual_authority[0], 0.2f, 1e-6f);
	EXPECT_NEAR(torque(0), status.nominal_torque_limited[0] + status.residual_torque_limited[0], 1e-6f);
	EXPECT_LE(fabsf(torque(0)), parameters.torque_limit(0) + 1e-6f);
	EXPECT_TRUE(status.total_bound_valid);
	EXPECT_EQ(status.total_bound_violation_count, 0u);
}

TEST(RateControlTest, ImplicitSuperTwistingGyroNominalTorqueAtZeroTrackingError)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(2.f, 3.f, 4.f);
	parameters.control_effectiveness = Vector3f(1.f, 1.f, 1.f);
	parameters.torque_limit = Vector3f(1.f, 1.f, 1.f);
	parameters.residual_torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	parameters.gyro_compensation = 1.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	const Vector3f rates(0.1f, 0.2f, 0.3f);
	const Vector3f torque = rate_control.update(rates, rates, Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_NEAR(status.nominal_torque_raw[0], 0.06f, 1e-5f);
	EXPECT_NEAR(status.nominal_torque_raw[1], -0.06f, 1e-5f);
	EXPECT_NEAR(status.nominal_torque_raw[2], 0.02f, 1e-5f);
	EXPECT_NEAR(torque(0), 0.06f, 1e-5f);
	EXPECT_NEAR(torque(1), -0.06f, 1e-5f);
	EXPECT_NEAR(torque(2), 0.02f, 1e-5f);
}

TEST(RateControlTest, ImplicitSuperTwistingRollPitchFeedforwardScalePreservesYawModelAssistance)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(2.f, 2.f, 2.f);
	parameters.control_effectiveness = Vector3f(4.f, 4.f, 4.f);
	parameters.reference_acceleration_limit = Vector3f(1.f, 1.f, 1.f);
	parameters.reference_jerk_limit = Vector3f(10.f, 10.f, 10.f);
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.01f, false);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);

	EXPECT_FLOAT_EQ(status.nominal_torque_raw[0], 0.f);
	EXPECT_FLOAT_EQ(status.nominal_torque_raw[1], 0.f);
	EXPECT_GT(status.nominal_torque_raw[2], 0.f);
	EXPECT_NEAR(status.nominal_torque_raw[2], parameters.inertia(2) * status.reference_acceleration[2]
		    / parameters.control_effectiveness(2), 1e-5f);
}

TEST(RateControlTest, ImplicitSuperTwistingNominalSaturationPreservesResidualAuthority)
{
	for (float sign : {-1.f, 1.f}) {
		RateControl rate_control;
		AstsmcTestParameters parameters;
		parameters.inertia = Vector3f(1.f, 1.f, 1.f);
		parameters.control_effectiveness = Vector3f(1.f, 1.f, 1.f);
		parameters.torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
		parameters.residual_torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
		parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 10.f);
		parameters.reference_jerk_limit = Vector3f(100.f, 100.f, 100.f);
		parameters.reference_feedforward = 1.f;
		ASSERT_TRUE(configureAstsmc(rate_control, parameters));
		ASSERT_TRUE(rate_control.setControllerType(3));

		rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
		rate_control.update(Vector3f(), Vector3f(sign * 10.f, 0.f, 0.f), Vector3f(), 0.01f, false);
		astsmc_status_s status{};
		rate_control.getAstsmcStatus(status);

		EXPECT_TRUE(status.nominal_saturation[0]);
		EXPECT_NEAR(status.nominal_torque_limited[0], sign * 0.3f, 1e-5f);
		EXPECT_LE(fabsf(status.residual_torque_limited[0]), 0.2f + 1e-6f);
		EXPECT_LE(fabsf(status.torque_limited[0]), 0.5f + 1e-6f);
		EXPECT_GE(status.nominal_limit_count[0], 1u);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingValidCardChangeIsBumplessWithNominalTorque)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.residual_torque_limit = Vector3f(0.2f, 0.2f, 0.2f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));
	const Vector3f previous = rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.01f, false);

	parameters.reference_feedforward = 1.f;
	parameters.reference_acceleration_limit = Vector3f(1.f, 1.f, 1.f);
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	const Vector3f updated = rate_control.update(Vector3f(), Vector3f(0.2f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_NEAR(updated(0), previous(0), 1e-5f);
}

TEST(RateControlTest, ImplicitSuperTwistingYawExtensionCardValidationIsAtomic)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.yaw_residual_extension = 0.05f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	EXPECT_NEAR(rate_control.astsmcYawResidualAuthorityMaximum(), 0.15f, 1e-6f);

	parameters.yaw_residual_extension = 0.051f;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	EXPECT_NEAR(rate_control.astsmcYawResidualAuthorityMaximum(), 0.15f, 1e-6f);
	parameters.yaw_residual_extension = NAN;
	EXPECT_FALSE(configureAstsmc(rate_control, parameters));
	EXPECT_NEAR(rate_control.astsmcYawResidualAuthorityMaximum(), 0.15f, 1e-6f);
}

TEST(RateControlTest, ImplicitSuperTwistingYawExtensionRequiresHealthyDwellAndValidTiming)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.yaw_residual_extension = 0.05f;
	parameters.dt_max = 0.02f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	RateControl::AstsmcAllocatorFeedback feedback{};
	feedback.usable = true;
	feedback.new_sample = true;
	feedback.torque_setpoint_achieved = true;
	feedback.thrust_setpoint_achieved = true;
	rate_control.setAstsmcAllocatorFeedback(feedback);

	constexpr float dt = 0.01f;

	for (int sample = 0; sample < 9; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), dt, false, dt);
	}

	EXPECT_NEAR(rate_control.astsmcYawAuthorityHealthyDwell(), 0.09f, 1e-6f);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), 0.1f);
	const float dwell_before_invalid_timing = rate_control.astsmcYawAuthorityHealthyDwell();
	const float authority_before_invalid_timing = rate_control.astsmcYawResidualAuthorityEffective();
	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.03f, false, 0.03f);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawAuthorityHealthyDwell(), dwell_before_invalid_timing);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), authority_before_invalid_timing);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), dt, false, dt);
	EXPECT_NEAR(rate_control.astsmcYawAuthorityHealthyDwell(), 0.1f, 1e-6f);
	EXPECT_NEAR(rate_control.astsmcYawResidualAuthorityEffective(), 0.1005f, 1e-6f);
	EXPECT_TRUE(rate_control.astsmcYawAuthorityReleaseActive());

	for (int sample = 0; sample < 100; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), dt, false, dt);
	}

	EXPECT_NEAR(rate_control.astsmcYawResidualAuthorityEffective(), 0.15f, 1e-6f);
	EXPECT_FALSE(rate_control.astsmcYawAuthorityReleaseActive());
}

TEST(RateControlTest, ImplicitSuperTwistingYawExtensionBacksOffOnAllocationMiss)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.yaw_residual_extension = 0.05f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	RateControl::AstsmcAllocatorFeedback base_miss{};
	base_miss.usable = true;
	base_miss.new_sample = true;
	base_miss.torque_setpoint_achieved = false;
	base_miss.thrust_setpoint_achieved = true;
	rate_control.setAstsmcAllocatorFeedback(base_miss);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), 0.1f);
	EXPECT_TRUE(rate_control.astsmcYawAuthorityBackoffActive());
	EXPECT_EQ(rate_control.astsmcYawAuthorityBackoffCount(), 1u);

	RateControl::AstsmcAllocatorFeedback feedback{};
	feedback.usable = true;
	feedback.new_sample = true;
	feedback.torque_setpoint_achieved = true;
	feedback.thrust_setpoint_achieved = true;
	rate_control.setAstsmcAllocatorFeedback(feedback);

	for (int sample = 0; sample < 20; sample++) {
		rate_control.update(Vector3f(), Vector3f(0.f, 0.f, 10.f), Vector3f(), 0.01f, false, 0.01f);
	}

	ASSERT_GT(rate_control.astsmcYawResidualAuthorityEffective(), 0.1f);
	feedback.unallocated_torque = Vector3f(0.009353f, 0.099736f, 0.116139f);
	feedback.torque_setpoint_achieved = false;
	rate_control.setAstsmcAllocatorFeedback(feedback);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), 0.1f);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawAuthorityHealthyDwell(), 0.f);
	EXPECT_TRUE(rate_control.astsmcYawAuthorityBackoffActive());
	EXPECT_EQ(rate_control.astsmcYawAuthorityBackoffCount(), 2u);

	rate_control.setAstsmcAllocatorFeedback(feedback);
	EXPECT_EQ(rate_control.astsmcYawAuthorityBackoffCount(), 2u);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(0.f, 0.f, 10.f), Vector3f(), 0.01f, false, 0.01f);
	astsmc_status_s status{};
	rate_control.getAstsmcStatus(status);
	EXPECT_NEAR(status.residual_authority[2], 0.1f, 1e-6f);
	EXPECT_LE(fabsf(torque(2)), 0.15f + 1e-6f);
	EXPECT_TRUE(status.total_bound_valid);
}

TEST(RateControlTest, ImplicitSuperTwistingYawExtensionHysteresisHoldsAuthorityAndResetsDwell)
{
	RateControl rate_control;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.yaw_residual_extension = 0.05f;
	ASSERT_TRUE(configureAstsmc(rate_control, parameters));
	ASSERT_TRUE(rate_control.setControllerType(3));

	RateControl::AstsmcAllocatorFeedback feedback{};
	feedback.usable = true;
	feedback.new_sample = true;
	feedback.torque_setpoint_achieved = true;
	feedback.thrust_setpoint_achieved = true;
	rate_control.setAstsmcAllocatorFeedback(feedback);

	for (int sample = 0; sample < 20; sample++) {
		rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false, 0.01f);
	}

	const float released_authority = rate_control.astsmcYawResidualAuthorityEffective();
	ASSERT_GT(released_authority, 0.1f);
	feedback.unallocated_torque = Vector3f(0.00075f, 0.f, 0.f);
	rate_control.setAstsmcAllocatorFeedback(feedback);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), released_authority);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawAuthorityHealthyDwell(), 0.f);
	EXPECT_FALSE(rate_control.astsmcYawAuthorityBackoffActive());
	EXPECT_FALSE(rate_control.astsmcYawAuthorityReleaseActive());

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false, 0.01f);
	EXPECT_FLOAT_EQ(rate_control.astsmcYawResidualAuthorityEffective(), released_authority);
}

TEST(RateControlTest, ImplicitSuperTwistingAllocatedTorqueConditionsOnlyMissedRollAxis)
{
	RateControl baseline;
	RateControl conditioned;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(1.f, 1.f, 1.f);
	parameters.control_effectiveness = Vector3f(1.f, 1.f, 1.f);
	parameters.k1 = Vector3f(1.f, 1.f, 1.f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.f);
	parameters.torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
	parameters.residual_torque_limit = Vector3f(0.5f, 0.5f, 0.5f);
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	ASSERT_TRUE(configureAstsmc(conditioned, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(conditioned.setControllerType(3));

	constexpr float dt = 0.01f;
	const Vector3f rate;
	const Vector3f setpoint(0.5f, 0.4f, 0.3f);
	const Vector3f previous_torque = conditioned.update(rate, setpoint, Vector3f(), dt, false, dt);
	baseline.update(rate, setpoint, Vector3f(), dt, false, dt);

	RateControl::AstsmcAllocatorFeedback feedback{};
	feedback.usable = true;
	feedback.new_sample = true;
	feedback.torque_setpoint_achieved = false;
	feedback.thrust_setpoint_achieved = true;
	feedback.allocated_torque = previous_torque;
	feedback.allocated_torque(0) = 0.f;
	feedback.unallocated_torque(0) = previous_torque(0);
	conditioned.setAstsmcAllocatorFeedback(feedback);

	const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), dt, false, dt);
	const Vector3f conditioned_torque = conditioned.update(rate, setpoint, Vector3f(), dt, false, dt);
	astsmc_status_s baseline_status{};
	astsmc_status_s conditioned_status{};
	baseline.getAstsmcStatus(baseline_status);
	conditioned.getAstsmcStatus(conditioned_status);

	EXPECT_TRUE(conditioned.astsmcAllocatorConditioningActive(0));
	EXPECT_FALSE(conditioned.astsmcAllocatorConditioningActive(1));
	EXPECT_LT(conditioned_status.integral_state[0], baseline_status.integral_state[0]);
	EXPECT_FLOAT_EQ(conditioned_status.integral_state[1], baseline_status.integral_state[1]);
	EXPECT_FLOAT_EQ(conditioned_status.integral_state[2], baseline_status.integral_state[2]);
	EXPECT_FLOAT_EQ(conditioned_torque(1), baseline_torque(1));
	EXPECT_FLOAT_EQ(conditioned_torque(2), baseline_torque(2));
	EXPECT_TRUE(conditioned_status.total_bound_valid);
}

TEST(RateControlTest, ImplicitSuperTwistingFeasibleAllocatedTorquePreservesTrajectory)
{
	RateControl baseline;
	RateControl feedback_driven;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	ASSERT_TRUE(configureAstsmc(feedback_driven, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(feedback_driven.setControllerType(3));

	for (int sample = 0; sample < 100; sample++) {
		const Vector3f rate(0.1f * sinf(0.03f * sample), -0.08f * cosf(0.02f * sample), 0.05f);
		const Vector3f setpoint(0.2f, -0.15f, 0.1f * sinf(0.01f * sample));
		const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		const Vector3f feedback_torque = feedback_driven.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);

		RateControl::AstsmcAllocatorFeedback feedback{};
		feedback.usable = true;
		feedback.new_sample = true;
		feedback.torque_setpoint_achieved = true;
		feedback.thrust_setpoint_achieved = true;
		feedback.allocated_torque = feedback_torque;
		feedback_driven.setAstsmcAllocatorFeedback(feedback);
		EXPECT_EQ(feedback_torque, baseline_torque);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingUnusableOrStaleAllocatedTorquePreservesTrajectory)
{
	RateControl baseline;
	RateControl feedback_driven;
	AstsmcTestParameters parameters;
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	ASSERT_TRUE(configureAstsmc(feedback_driven, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(feedback_driven.setControllerType(3));

	for (int sample = 0; sample < 100; sample++) {
		RateControl::AstsmcAllocatorFeedback feedback{};
		feedback.usable = sample % 2 == 0;
		feedback.new_sample = !feedback.usable;
		feedback.torque_setpoint_achieved = false;
		feedback.thrust_setpoint_achieved = false;
		feedback.actuator_bound = true;
		feedback.allocated_torque = Vector3f(-0.4f, 0.35f, -0.3f);
		feedback.unallocated_torque = Vector3f(0.4f, -0.35f, 0.3f);
		feedback_driven.setAstsmcAllocatorFeedback(feedback);

		const Vector3f rate(0.1f * sinf(0.03f * sample), -0.08f * cosf(0.02f * sample), 0.05f);
		const Vector3f setpoint(0.2f, -0.15f, 0.1f * sinf(0.01f * sample));
		const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		const Vector3f feedback_torque = feedback_driven.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		EXPECT_EQ(feedback_torque, baseline_torque);
		EXPECT_FALSE(feedback_driven.astsmcAllocatorConditioningActive(0));
		EXPECT_FALSE(feedback_driven.astsmcAllocatorConditioningActive(1));
	}

	astsmc_status_s baseline_status{};
	astsmc_status_s feedback_status{};
	baseline.getAstsmcStatus(baseline_status);
	feedback_driven.getAstsmcStatus(feedback_status);

	for (int axis = 0; axis < 3; axis++) {
		EXPECT_FLOAT_EQ(feedback_status.integral_state[axis], baseline_status.integral_state[axis]);
	}
}

TEST(RateControlTest, ImplicitSuperTwistingAllocationLossConditioningLimitsRetainedStateAndRecoversAfterRelease)
{
	RateControl baseline;
	RateControl conditioned;
	AstsmcTestParameters parameters;
	parameters.inertia = Vector3f(0.01f, 0.01f, 0.050951f);
	parameters.control_effectiveness = Vector3f(0.56f, 0.53f, 0.8f);
	parameters.k1 = Vector3f(4.5f, 4.5f, 1.5f);
	parameters.k2 = Vector3f(1.f, 1.f, 1.5f);
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.reference_acceleration_limit = Vector3f(10.f, 10.f, 5.f);
	parameters.reference_jerk_limit = Vector3f(25.f, 25.f, 40.f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	parameters.variation_weight = Vector3f(1.f, 1.f, 0.f);
	parameters.tracking_blend = 0.75f;
	parameters.reference_feedforward = 1.f;
	parameters.reference_feedforward_rp = 0.5f;
	parameters.gyro_compensation = 1.f;
	parameters.roll_pitch_residual_extension = 0.1f;
	parameters.roll_pitch_k1_recovery_boost = 1.5f;
	parameters.dt_min = 0.0005f;
	parameters.dt_max = 0.005f;
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	ASSERT_TRUE(configureAstsmc(conditioned, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(conditioned.setControllerType(3));

	constexpr float dt = 0.004f;
	Vector3f baseline_rate;
	Vector3f conditioned_rate;
	Vector3f previous_baseline_torque;
	Vector3f previous_conditioned_torque;
	float maximum_baseline_integral_state = 0.f;
	float maximum_conditioned_integral_state = 0.f;
	float conditioned_rate_at_release = 0.f;
	uint32_t conditioning_cycles = 0;

	for (int sample = 0; sample < 1750; sample++) {
		const bool command_active = sample >= 125 && sample < 875;
		Vector3f rate_setpoint;
		rate_setpoint(0) = command_active ? -3.5f : 0.f;
		rate_setpoint(1) = command_active ? 2.5f : 0.f;

		if (sample > 0) {
			RateControl::AstsmcAllocatorFeedback feedback{};
			feedback.usable = true;
			feedback.new_sample = true;
			feedback.torque_setpoint_achieved = !command_active;
			feedback.thrust_setpoint_achieved = true;
			feedback.allocated_torque = previous_conditioned_torque;

			if (command_active) {
				feedback.allocated_torque(0) = math::constrain(previous_conditioned_torque(0), -0.08f, 0.08f);
				feedback.unallocated_torque(0) = previous_conditioned_torque(0) - feedback.allocated_torque(0);
			}

			conditioned.setAstsmcAllocatorFeedback(feedback);
		}

		previous_baseline_torque = baseline.update(baseline_rate, rate_setpoint, Vector3f(), dt, false, dt);
		previous_conditioned_torque = conditioned.update(conditioned_rate, rate_setpoint, Vector3f(), dt, false, dt);

		const float baseline_applied_roll = command_active ?
				math::constrain(previous_baseline_torque(0), -0.08f, 0.08f) : previous_baseline_torque(0);
		const float conditioned_applied_roll = command_active ?
				math::constrain(previous_conditioned_torque(0), -0.08f, 0.08f) : previous_conditioned_torque(0);
		baseline_rate(0) += parameters.control_effectiveness(0) / parameters.inertia(0) * baseline_applied_roll * dt;
		conditioned_rate(0) += parameters.control_effectiveness(0) / parameters.inertia(0) * conditioned_applied_roll * dt;
		baseline_rate(1) += parameters.control_effectiveness(1) / parameters.inertia(1) * previous_baseline_torque(1) * dt;
		conditioned_rate(1) += parameters.control_effectiveness(1) / parameters.inertia(1) * previous_conditioned_torque(1) * dt;

		if (conditioned.astsmcAllocatorConditioningActive(0)) {
			conditioning_cycles++;
		}

		if (command_active) {
			astsmc_status_s baseline_status{};
			astsmc_status_s conditioned_status{};
			baseline.getAstsmcStatus(baseline_status);
			conditioned.getAstsmcStatus(conditioned_status);
			maximum_baseline_integral_state = math::max(maximum_baseline_integral_state,
					fabsf(baseline_status.integral_state[0]));
			maximum_conditioned_integral_state = math::max(maximum_conditioned_integral_state,
					fabsf(conditioned_status.integral_state[0]));
		}

		if (sample == 874) {
			conditioned_rate_at_release = fabsf(conditioned_rate(0));
		}
	}

	astsmc_status_s conditioned_status{};
	astsmc_safety_status_s conditioned_safety_status{};
	conditioned.getAstsmcStatus(conditioned_status);
	conditioned.getAstsmcSafetyStatus(conditioned_safety_status);
	EXPECT_GT(conditioning_cycles, 100u);
	EXPECT_LT(maximum_conditioned_integral_state, maximum_baseline_integral_state);
	EXPECT_LT(fabsf(conditioned_rate(0)), conditioned_rate_at_release);
	EXPECT_LT(fabsf(conditioned_rate(0)), 0.25f);
	EXPECT_TRUE(conditioned_status.total_bound_valid);
	EXPECT_EQ(conditioned_status.total_bound_violation_count, 0u);
	EXPECT_FALSE(conditioned_safety_status.runtime_fault_latched);
}

TEST(RateControlTest, ImplicitSuperTwistingDisabledYawExtensionPreservesTrajectory)
{
	RateControl baseline;
	RateControl feedback_driven;
	AstsmcTestParameters parameters;
	parameters.torque_limit = Vector3f(0.2f, 0.2f, 0.15f);
	parameters.residual_torque_limit = Vector3f(0.1f, 0.1f, 0.1f);
	ASSERT_TRUE(configureAstsmc(baseline, parameters));
	ASSERT_TRUE(configureAstsmc(feedback_driven, parameters));
	ASSERT_TRUE(baseline.setControllerType(3));
	ASSERT_TRUE(feedback_driven.setControllerType(3));

	for (int sample = 0; sample < 200; sample++) {
		RateControl::AstsmcAllocatorFeedback feedback{};
		feedback.usable = sample % 7 != 0;
		feedback.new_sample = true;
		feedback.torque_setpoint_achieved = sample % 11 != 0;
		feedback.thrust_setpoint_achieved = sample % 13 != 0;
		feedback.actuator_bound = sample % 17 == 0;
		feedback.unallocated_torque = Vector3f(0.002f * (sample % 3), 0.f, 0.f);
		feedback_driven.setAstsmcAllocatorFeedback(feedback);
		const Vector3f rate(0.1f * sinf(0.03f * sample), -0.08f * cosf(0.02f * sample), 0.05f);
		const Vector3f setpoint(0.2f, -0.15f, 0.1f * sinf(0.01f * sample));
		const Vector3f baseline_torque = baseline.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		const Vector3f feedback_torque = feedback_driven.update(rate, setpoint, Vector3f(), 0.004f, false, 0.004f);
		EXPECT_EQ(feedback_torque, baseline_torque);
	}

	astsmc_status_s baseline_status{};
	astsmc_status_s feedback_status{};
	baseline.getAstsmcStatus(baseline_status);
	feedback_driven.getAstsmcStatus(feedback_status);
	EXPECT_FLOAT_EQ(feedback_status.residual_authority[2], baseline_status.residual_authority[2]);
	EXPECT_FLOAT_EQ(feedback_status.integral_state[2], baseline_status.integral_state[2]);
}
