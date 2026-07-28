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
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>

#include <stdint.h>

namespace type4_bench_platform
{

struct RuntimeSnapshot {
	uint64_t timestamp{0};
	uint64_t rate_ctrl_runtime{0};
	uint64_t benchmark_runtime{0};
	bool rate_ctrl_found{false};
	bool benchmark_found{false};
};

struct StackUsage {
	uint32_t total{0};
	uint32_t free{0};
	bool valid{false};
};

struct LatencySnapshot {
	uint16_t bucket[LATENCY_BUCKET_COUNT] {};
	uint32_t counter[LATENCY_BUCKET_COUNT + 1] {};
};

bool beginRuntimeMeasurement();
void endRuntimeMeasurement();
bool captureRuntimeSnapshot(px4_task_t benchmark_pid, RuntimeSnapshot &snapshot);
bool captureStackUsage(px4_task_t benchmark_pid, StackUsage &usage);
void captureLatencySnapshot(LatencySnapshot &snapshot);

} // namespace type4_bench_platform
