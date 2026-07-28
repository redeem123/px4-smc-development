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

#include "type4_bench_platform.hpp"

#include <px4_platform_common/px4_config.h>

#include <string.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION) && defined(CONFIG_STACK_COLORATION)
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <px4_platform/cpuload.h>
#endif

namespace type4_bench_platform
{

bool beginRuntimeMeasurement()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
	cpuload_monitor_start();
	return true;
#else
	return false;
#endif
}

void endRuntimeMeasurement()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
	cpuload_monitor_stop();
#endif
}

bool captureRuntimeSnapshot(px4_task_t benchmark_pid, RuntimeSnapshot &snapshot)
{
	snapshot = RuntimeSnapshot{};
	snapshot.timestamp = hrt_absolute_time();

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
	sched_lock();

	for (int index = 0; index < CONFIG_FS_PROCFS_MAX_TASKS; ++index) {
		const system_load_taskinfo_s &task = system_load.tasks[index];

		if (!task.valid || (task.tcb == nullptr)) {
			continue;
		}

		uint64_t runtime = task.total_runtime;

		if ((task.curr_start_time > 0) && (task.curr_start_time <= snapshot.timestamp)
		    && (task.tcb->task_state == TSTATE_TASK_RUNNING)) {
			runtime += snapshot.timestamp - task.curr_start_time;
		}

		if (task.tcb->pid == benchmark_pid) {
			snapshot.benchmark_runtime = runtime;
			snapshot.benchmark_found = true;
		}

		if (strcmp(task.tcb->name, "wq:rate_ctrl") == 0) {
			snapshot.rate_ctrl_runtime = runtime;
			snapshot.rate_ctrl_found = true;
		}
	}

	sched_unlock();
	return snapshot.rate_ctrl_found && snapshot.benchmark_found;
#else
	(void)benchmark_pid;
	return false;
#endif
}

bool captureStackUsage(px4_task_t benchmark_pid, StackUsage &usage)
{
	usage = StackUsage{};

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION) && defined(CONFIG_STACK_COLORATION)
	sched_lock();

	for (int index = 0; index < CONFIG_FS_PROCFS_MAX_TASKS; ++index) {
		const system_load_taskinfo_s &task = system_load.tasks[index];

		if (task.valid && (task.tcb != nullptr) && (task.tcb->pid == benchmark_pid)) {
			const ssize_t stack_free = up_check_tcbstack_remain(task.tcb);

			if ((stack_free >= 0) && (static_cast<size_t>(stack_free) <= task.tcb->adj_stack_size)) {
				usage.total = task.tcb->adj_stack_size;
				usage.free = static_cast<uint32_t>(stack_free);
				usage.valid = true;
			}

			break;
		}
	}

	sched_unlock();
	return usage.valid;
#else
	(void)benchmark_pid;
	return false;
#endif
}

void captureLatencySnapshot(LatencySnapshot &snapshot)
{
	for (uint16_t index = 0; index < LATENCY_BUCKET_COUNT; ++index) {
		const latency_info_t latency = get_latency(index, index);
		snapshot.bucket[index] = latency.bucket;
		snapshot.counter[index] = latency.counter;
	}

	snapshot.counter[LATENCY_BUCKET_COUNT] = get_latency(LATENCY_BUCKET_COUNT - 1, LATENCY_BUCKET_COUNT).counter;
}

} // namespace type4_bench_platform
