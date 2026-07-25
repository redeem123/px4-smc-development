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

#include <cstdint>

constexpr bool rateControlLanded(int controller_type, bool landed, bool maybe_landed)
{
	return landed || (controller_type != 3 && maybe_landed);
}

constexpr bool rateControlLowThrottleReleaseConfirmed(uint64_t now, uint64_t no_ground_contact_since,
		uint64_t confirmation_delay)
{
	return no_ground_contact_since != 0 && now >= no_ground_contact_since
	       && now - no_ground_contact_since >= confirmation_delay;
}

constexpr bool rateControlReleaseEvidence(bool armed, bool landed, bool maybe_landed, bool ground_contact,
			bool has_low_throttle, bool allow_low_throttle_release)
{
	return armed && !landed && !maybe_landed && !ground_contact
	       && (!has_low_throttle || allow_low_throttle_release);
}

constexpr bool rateControlGroundContained(int controller_type, bool armed, bool released_since_arming, bool landed,
			bool maybe_landed, bool ground_contact, bool has_low_throttle)
{
	return controller_type == 3 && armed && !landed && has_low_throttle
	       && (!released_since_arming || (maybe_landed && ground_contact));
}
