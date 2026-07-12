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

bool configureModelBasedSmc(RateControl &rate_control, const SmcTestParameters &parameters)
{
	return rate_control.setModelBasedSmcParameters(parameters.inertia, parameters.control_effectiveness,
			parameters.c, parameters.eta, parameters.boundary, parameters.ks,
			parameters.rate_sp_derivative_limit, parameters.integral_limit, parameters.torque_limit,
			parameters.cutoff, parameters.slew);
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
