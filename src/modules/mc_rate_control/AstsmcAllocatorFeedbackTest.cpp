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

#include "AstsmcAllocatorFeedback.hpp"

#include <gtest/gtest.h>

namespace
{
AstsmcAllocatorFeedbackSample validSample()
{
	AstsmcAllocatorFeedbackSample sample{};
	sample.timestamp = 1010000;
	sample.timestamp_sample = 1008000;
	sample.torque_setpoint_achieved = true;
	sample.thrust_setpoint_achieved = true;
	return sample;
}

AstsmcAllocatorFeedbackContext validContext()
{
	AstsmcAllocatorFeedbackContext context{};
	context.eligible = true;
	context.require_new_sample = true;
	context.now = 1012000;
	context.current_sample = 1012000;
	context.first_command_sample = 1000000;
	context.last_command_sample = 1012000;
	context.last_accepted_sample = 1004000;
	return context;
}
}

TEST(AstsmcAllocatorFeedbackTest, AcceptsFreshMonotonicSample)
{
	const auto result = astsmcAllocatorFeedbackGate(validSample(), validContext());
	EXPECT_TRUE(result.usable);
	EXPECT_EQ(result.rejection_flags, AstsmcAllocatorFeedbackGateResult::RejectNone);
	EXPECT_FLOAT_EQ(result.publication_age_s, 0.002f);
	EXPECT_FLOAT_EQ(result.sample_age_s, 0.004f);
}

TEST(AstsmcAllocatorFeedbackTest, RejectsInactiveAndBeforeFirstCommand)
{
	auto context = validContext();
	context.eligible = false;
	context.first_command_sample = 0;
	const auto result = astsmcAllocatorFeedbackGate(validSample(), context);
	EXPECT_FALSE(result.usable);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectInactive, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectBeforeEpoch, 0);
}

TEST(AstsmcAllocatorFeedbackTest, RejectsStaleFutureAndUnmatchedSamples)
{
	auto sample = validSample();
	auto context = validContext();

	sample.timestamp = context.now + 1;
	sample.timestamp_sample = context.current_sample + 1;
	auto result = astsmcAllocatorFeedbackGate(sample, context);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectPublicationFuture, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectSampleFuture, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectAfterLastCommand, 0);

	sample = validSample();
	context.now = sample.timestamp + context.freshness_timeout + 1;
	context.current_sample = sample.timestamp_sample + context.freshness_timeout + 1;
	result = astsmcAllocatorFeedbackGate(sample, context);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectPublicationStale, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectSampleStale, 0);
}

TEST(AstsmcAllocatorFeedbackTest, RejectsDuplicateOutOfOrderAndPriorEpoch)
{
	auto sample = validSample();
	auto context = validContext();
	context.last_accepted_sample = sample.timestamp_sample;
	auto result = astsmcAllocatorFeedbackGate(sample, context);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectDuplicateOrOutOfOrder, 0);

	context.last_accepted_sample = 0;
	context.first_command_sample = sample.timestamp_sample + 1;
	result = astsmcAllocatorFeedbackGate(sample, context);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectBeforeEpoch, 0);
}

TEST(AstsmcAllocatorFeedbackTest, RejectsMalformedAndExcludedAllocatorStates)
{
	auto sample = validSample();
	auto context = validContext();
	sample.unallocated_torque[1] = NAN;
	sample.preflight_active = true;
	sample.handled_motor_failure_mask = 1;
	sample.motor_stop_mask = 2;
	const auto result = astsmcAllocatorFeedbackGate(sample, context);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectNonfiniteTorque, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectPreflight, 0);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectMotorFailure, 0);
}

TEST(AstsmcAllocatorFeedbackTest, RejectsNonfiniteThrustVectors)
{
	auto sample = validSample();
	sample.allocated_thrust[2] = INFINITY;
	auto result = astsmcAllocatorFeedbackGate(sample, validContext());
	EXPECT_FALSE(result.usable);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectNonfiniteThrust, 0);

	sample = validSample();
	sample.unallocated_thrust[0] = NAN;
	result = astsmcAllocatorFeedbackGate(sample, validContext());
	EXPECT_FALSE(result.usable);
	EXPECT_NE(result.rejection_flags & AstsmcAllocatorFeedbackGateResult::RejectNonfiniteThrust, 0);
}

TEST(AstsmcAllocatorFeedbackTest, CachedAcceptedSampleDoesNotFailOrdering)
{
	auto context = validContext();
	const auto sample = validSample();
	context.require_new_sample = false;
	context.last_accepted_sample = sample.timestamp_sample;
	EXPECT_TRUE(astsmcAllocatorFeedbackGate(sample, context).usable);
}
