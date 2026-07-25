/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "RateControlLandingPolicy.hpp"

#include <gtest/gtest.h>

TEST(RateControlLandingPolicyTest, ImplicitSuperTwistingRequiresConfirmedLanding)
{
	EXPECT_FALSE(rateControlLanded(3, false, false));
	EXPECT_FALSE(rateControlLanded(3, false, true));
	EXPECT_TRUE(rateControlLanded(3, true, false));
	EXPECT_TRUE(rateControlLanded(3, true, true));
}

TEST(RateControlLandingPolicyTest, OtherControllersRetainMaybeLandedPolicy)
{
	for (int controller_type = 0; controller_type <= 2; controller_type++) {
		EXPECT_FALSE(rateControlLanded(controller_type, false, false));
		EXPECT_TRUE(rateControlLanded(controller_type, false, true));
		EXPECT_TRUE(rateControlLanded(controller_type, true, false));
		EXPECT_TRUE(rateControlLanded(controller_type, true, true));
	}
}

TEST(RateControlLandingPolicyTest, PreAirborneLowThrottleIsContainedUntilReleaseEvidence)
{
	EXPECT_TRUE(rateControlGroundContained(3, true, false, false, false, true, true));
	EXPECT_TRUE(rateControlGroundContained(3, true, false, false, true, true, true));
	EXPECT_TRUE(rateControlGroundContained(3, true, false, false, false, false, true));

	EXPECT_FALSE(rateControlGroundContained(3, false, false, false, false, true, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, false, true, false, true, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, false, false, false, true, false));

	for (int controller_type = 0; controller_type <= 2; controller_type++) {
		EXPECT_FALSE(rateControlGroundContained(controller_type, true, false, false, false, true, true));
	}
}

TEST(RateControlLandingPolicyTest, PostAirborneContainmentRequiresMaybeLanded)
{
	EXPECT_TRUE(rateControlGroundContained(3, true, true, false, true, true, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, true, false, false, true, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, true, true, true, true, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, true, false, true, false, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, true, false, true, true, false));
}

TEST(RateControlLandingPolicyTest, MaybeLandedAloneDoesNotImplyTypeThreeGroundContainment)
{
	EXPECT_FALSE(rateControlLanded(3, false, true));
	EXPECT_FALSE(rateControlGroundContained(3, true, true, false, true, false, true));
}

TEST(RateControlLandingPolicyTest, AirborneEvidenceRequiresClearLandStatesAndReleaseEvidence)
{
	EXPECT_TRUE(rateControlReleaseEvidence(true, false, false, false, false, false));
	EXPECT_TRUE(rateControlReleaseEvidence(true, false, false, false, true, true));
	EXPECT_FALSE(rateControlReleaseEvidence(false, false, false, false, false, false));
	EXPECT_FALSE(rateControlReleaseEvidence(true, true, false, false, false, false));
	EXPECT_FALSE(rateControlReleaseEvidence(true, false, true, false, false, false));
	EXPECT_FALSE(rateControlReleaseEvidence(true, false, false, true, false, false));
	EXPECT_FALSE(rateControlReleaseEvidence(true, false, false, false, true, false));
}

TEST(RateControlLandingPolicyTest, LowThrottleReleaseRequiresContinuousConfirmationDelay)
{
	constexpr uint64_t start = 1000000;
	constexpr uint64_t delay = 500000;

	EXPECT_FALSE(rateControlLowThrottleReleaseConfirmed(start, 0, delay));
	EXPECT_FALSE(rateControlLowThrottleReleaseConfirmed(start - 1, start, delay));
	EXPECT_FALSE(rateControlLowThrottleReleaseConfirmed(start + delay - 1, start, delay));
	EXPECT_TRUE(rateControlLowThrottleReleaseConfirmed(start + delay, start, delay));
}
