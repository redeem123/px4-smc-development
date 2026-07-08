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

	EXPECT_FLOAT_EQ(torque(0), 1.f);
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

	const Vector3f rates(0.1f, 0.2f, 0.3f);
	const Vector3f torque = rate_control.update(rates, rates, Vector3f(), 0.1f, false);

	EXPECT_NEAR(torque(0), 0.06f, 1e-5f);
	EXPECT_NEAR(torque(1), -0.06f, 1e-5f);
	EXPECT_NEAR(torque(2), 0.02f, 1e-5f);
}

TEST(RateControlTest, MpcUsesLimitedRateSetpointDerivativeFeedForward)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(2.f, 2.f, 2.f), Vector3f(), Vector3f(4.f, 4.f, 4.f),
				 Vector3f(1.f, 1.f, 1.f), Vector3f(), 1, 0.f);
	rate_control.setMpcRateSetpointDerivativeLimit(1.f);

	const Vector3f first_torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.1f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(10.f, -10.f, 0.f), Vector3f(), 0.1f, false);

	EXPECT_EQ(first_torque, Vector3f());
	EXPECT_NEAR(torque(0), 0.5f, 1e-5f);
	EXPECT_NEAR(torque(1), -0.5f, 1e-5f);
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

TEST(RateControlTest, MpcInvalidInertiaDisablesAxis)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setMpcGains(Vector3f(NAN, -1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(), Vector3f(), 1, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 1.f);
}

TEST(RateControlTest, MpcIntegralBiasIsBoundedAndResetWhenLanded)
{
	RateControl rate_control;
	rate_control.setControllerType(1);
	rate_control.setIntegratorLimit(Vector3f(0.02f, 0.02f, 0.02f));
	rate_control.setMpcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f), Vector3f(1.f, 1.f, 1.f),
				 Vector3f(100.f, 100.f, 100.f), Vector3f(), 1, 0.f);
	rate_control.setMpcIntegralGain(Vector3f(1.f, 0.f, 0.f));

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
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(), Vector3f(1.f, 1.f, 1.f),
					   Vector3f(0.1f, 0.1f, 0.1f), Vector3f(), 10.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.5f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcGyroCompensation)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(2.f, 3.f, 4.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 10.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	const Vector3f rates(0.1f, 0.2f, 0.3f);
	const Vector3f torque = rate_control.update(rates, rates, Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.06f);
	EXPECT_FLOAT_EQ(torque(1), -0.06f);
	EXPECT_FLOAT_EQ(torque(2), 0.02f);
}

TEST(RateControlTest, ModelBasedSmcRateSetpointDerivativeLimit)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 10.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcUsesRateDerivativeDamping)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setPidGains(Vector3f(), Vector3f(), Vector3f(0.1f, 0.2f, 0.3f));
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 0.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(), Vector3f(1.f, 2.f, -3.f), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), -0.1f);
	EXPECT_FLOAT_EQ(torque(1), -0.4f);
	EXPECT_FLOAT_EQ(torque(2), 0.9f);
}

TEST(RateControlTest, ModelBasedSmcTorqueOutputIsNormalized)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(10.f, 10.f, 10.f), Vector3f(), Vector3f(10.f, 10.f, 10.f),
					   Vector3f(0.1f, 0.1f, 0.1f), Vector3f(), 1000.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, -1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 1.f);
	EXPECT_FLOAT_EQ(torque(1), -1.f);
	EXPECT_FLOAT_EQ(torque(2), 1.f);
}

TEST(RateControlTest, ModelBasedSmcZeroRateSetpointDerivativeLimitDisablesFeedForward)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 0.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcSetpointResetClearsDerivativeHistory)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 10.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.resetModelBasedSmcSetpoint();
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcLandedUpdateDoesNotSeedTorqueSlew)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(), Vector3f(1.f, 0.f, 0.f),
					   Vector3f(0.1f, 0.1f, 0.1f), Vector3f(), 0.f);
	rate_control.setSMCSafeguards(0.f, 1.f);

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
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(1.f, 1.f, 1.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 10.f);
	rate_control.setSMCSafeguards(0.f, 0.f);

	rate_control.update(Vector3f(), Vector3f(), Vector3f(), 0.01f, false);
	rate_control.setModelBasedSmcGains(Vector3f(2.f, 2.f, 2.f), Vector3f(), Vector3f(), Vector3f(0.1f, 0.1f, 0.1f),
					   Vector3f(), 10.f);
	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 0.f, 0.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}

TEST(RateControlTest, ModelBasedSmcInvalidGainsAreSanitized)
{
	RateControl rate_control;
	rate_control.setControllerType(2);
	rate_control.setIntegratorLimit(Vector3f(1.f, 1.f, 1.f));
	rate_control.setModelBasedSmcGains(Vector3f(NAN, NAN, NAN), Vector3f(NAN, NAN, NAN), Vector3f(NAN, NAN, NAN),
					   Vector3f(NAN, NAN, NAN), Vector3f(NAN, NAN, NAN), NAN);
	rate_control.setSMCSafeguards(NAN, NAN);

	const Vector3f torque = rate_control.update(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f(), 0.01f, false);

	EXPECT_FLOAT_EQ(torque(0), 0.f);
	EXPECT_FLOAT_EQ(torque(1), 0.f);
	EXPECT_FLOAT_EQ(torque(2), 0.f);
}
