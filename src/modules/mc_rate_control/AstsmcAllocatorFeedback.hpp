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

#pragma once

#include <cmath>
#include <cstdint>

struct AstsmcAllocatorFeedbackSample {
	uint64_t timestamp{0};
	uint64_t timestamp_sample{0};
	float allocated_torque[3] {};
	float unallocated_torque[3] {};
	float allocated_thrust[3] {};
	float unallocated_thrust[3] {};
	bool torque_setpoint_achieved{false};
	bool thrust_setpoint_achieved{false};
	bool actuator_bound{false};
	bool preflight_active{false};
	uint16_t handled_motor_failure_mask{0};
	uint16_t motor_stop_mask{0};
};

struct AstsmcAllocatorFeedbackContext {
	bool eligible{false};
	bool require_new_sample{false};
	uint64_t now{0};
	uint64_t current_sample{0};
	uint64_t first_command_sample{0};
	uint64_t last_command_sample{0};
	uint64_t last_accepted_sample{0};
	uint64_t freshness_timeout{20000};
};

struct AstsmcAllocatorFeedbackGateResult {
	enum RejectReason : uint16_t {
		RejectNone = 0,
		RejectInactive = 1u << 0,
		RejectNoStatus = 1u << 1,
		RejectPublicationFuture = 1u << 2,
		RejectPublicationStale = 1u << 3,
		RejectSampleFuture = 1u << 4,
		RejectSampleStale = 1u << 5,
		RejectBeforeEpoch = 1u << 6,
		RejectAfterLastCommand = 1u << 7,
		RejectDuplicateOrOutOfOrder = 1u << 8,
		RejectNonfiniteTorque = 1u << 9,
		RejectPreflight = 1u << 10,
		RejectMotorFailure = 1u << 11,
		RejectNonfiniteThrust = 1u << 12,
	};

	bool usable{false};
	uint16_t rejection_flags{RejectNone};
	float publication_age_s{NAN};
	float sample_age_s{NAN};
};

inline bool astsmcAllocatorFeedbackTorqueFinite(const AstsmcAllocatorFeedbackSample &sample)
{
	for (int axis = 0; axis < 3; axis++) {
		if (!std::isfinite(sample.allocated_torque[axis]) || !std::isfinite(sample.unallocated_torque[axis])) {
			return false;
		}
	}

	return true;
}

inline bool astsmcAllocatorFeedbackThrustFinite(const AstsmcAllocatorFeedbackSample &sample)
{
	for (int axis = 0; axis < 3; axis++) {
		if (!std::isfinite(sample.allocated_thrust[axis]) || !std::isfinite(sample.unallocated_thrust[axis])) {
			return false;
		}
	}

	return true;
}

inline AstsmcAllocatorFeedbackGateResult astsmcAllocatorFeedbackGate(
	const AstsmcAllocatorFeedbackSample &sample, const AstsmcAllocatorFeedbackContext &context)
{
	AstsmcAllocatorFeedbackGateResult result{};

	if (!context.eligible) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectInactive;
	}

	if (sample.timestamp == 0 || sample.timestamp_sample == 0) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectNoStatus;
	}

	if (sample.timestamp > context.now) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectPublicationFuture;

	} else if (sample.timestamp != 0) {
		const uint64_t publication_age = context.now - sample.timestamp;
		result.publication_age_s = static_cast<float>(publication_age) * 1e-6f;

		if (publication_age > context.freshness_timeout) {
			result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectPublicationStale;
		}
	}

	if (sample.timestamp_sample > context.current_sample) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectSampleFuture;

	} else if (sample.timestamp_sample != 0) {
		const uint64_t sample_age = context.current_sample - sample.timestamp_sample;
		result.sample_age_s = static_cast<float>(sample_age) * 1e-6f;

		if (sample_age > context.freshness_timeout) {
			result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectSampleStale;
		}
	}

	if (context.first_command_sample == 0 || sample.timestamp_sample < context.first_command_sample) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectBeforeEpoch;
	}

	if (context.last_command_sample == 0 || sample.timestamp_sample > context.last_command_sample) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectAfterLastCommand;
	}

	if (context.require_new_sample && sample.timestamp_sample <= context.last_accepted_sample) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectDuplicateOrOutOfOrder;
	}

	if (!astsmcAllocatorFeedbackTorqueFinite(sample)) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectNonfiniteTorque;
	}

	if (!astsmcAllocatorFeedbackThrustFinite(sample)) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectNonfiniteThrust;
	}

	if (sample.preflight_active) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectPreflight;
	}

	if (sample.handled_motor_failure_mask != 0 || sample.motor_stop_mask != 0) {
		result.rejection_flags |= AstsmcAllocatorFeedbackGateResult::RejectMotorFailure;
	}

	result.usable = result.rejection_flags == AstsmcAllocatorFeedbackGateResult::RejectNone;
	return result;
}
