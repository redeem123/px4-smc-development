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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

#include <Type4WeightedAllocator.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

using type4_timing_bench::Type4WeightedAllocator;

namespace
{

constexpr int DefaultSamples = 1000;
constexpr int MaxSamples = 2048;
constexpr uint32_t DefaultBudgetUs = 300;
constexpr uint32_t DefaultPeriodUs = 1500;
constexpr float NormalizedKktTolerance = 1e-3f;
constexpr float ReachableDelta = 0.08f;
constexpr hrt_abstime VehicleStatusTimeout = 2 * 1000 * 1000;

uint32_t timing_samples[MaxSamples];
px4::atomic<px4_task_t> benchmark_owner{-1};
volatile sig_atomic_t cancellation_requested{0};

#if defined(__PX4_NUTTX)
void handleInterrupt(int)
{
	cancellation_requested = 1;
}
#endif

enum class Scenario : uint8_t {
	Feasible = 0,
	Saturated,
	NearDegenerate,
	RapidlySwitching,
};

struct TimingSummary {
	uint32_t median;
	uint32_t p95;
	uint32_t p99;
	uint32_t worst;
};

struct BenchmarkOwnerGuard {
	explicit BenchmarkOwnerGuard(px4_task_t owner) :
		_owner(owner)
	{
	}

	~BenchmarkOwnerGuard()
	{
		px4_task_t expected_owner = _owner;
		benchmark_owner.compare_exchange(&expected_owner, -1);
	}

private:
	px4_task_t _owner;
};

#if defined(__PX4_NUTTX)
struct SignalActionGuard {
	SignalActionGuard(int signal_number, const struct sigaction &previous_action) :
		_signal_number(signal_number),
		_previous_action(previous_action)
	{
	}

	~SignalActionGuard()
	{
		if (sigaction(_signal_number, &_previous_action, nullptr) != 0) {
			PX4_ERR("failed to restore signal handler");
		}
	}

private:
	int _signal_number;
	struct sigaction _previous_action;
};
#endif

int compareUint32(const void *left, const void *right)
{
	const uint32_t left_value = *static_cast<const uint32_t *>(left);
	const uint32_t right_value = *static_cast<const uint32_t *>(right);
	return (left_value > right_value) - (left_value < right_value);
}

uint32_t percentile(const uint32_t *sorted, int count, int numerator, int denominator)
{
	const int rank = (count * numerator + denominator - 1) / denominator;
	return sorted[rank - 1];
}

TimingSummary summarizeTimings(int count)
{
	qsort(timing_samples, count, sizeof(timing_samples[0]), compareUint32);
	return {
		.median = percentile(timing_samples, count, 50, 100),
		.p95 = percentile(timing_samples, count, 95, 100),
		.p99 = percentile(timing_samples, count, 99, 100),
		.worst = timing_samples[count - 1],
	};
}

const char *scenarioName(Scenario scenario)
{
	switch (scenario) {
	case Scenario::Feasible:
		return "feasible";

	case Scenario::Saturated:
		return "saturated";

	case Scenario::NearDegenerate:
		return "near-degenerate";

	case Scenario::RapidlySwitching:
		return "rapidly-switching";
	}

	return "unknown";
}

Type4WeightedAllocator::Problem makeProblem(Scenario scenario)
{
	Type4WeightedAllocator::Problem problem{};
	problem.num_axes = 4;
	problem.regularization = (scenario == Scenario::NearDegenerate) ? 1e-4f : 0.02f;

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		problem.reference(actuator) = 0.5f;
		problem.lower_bound(actuator) = problem.reference(actuator) - ReachableDelta;
		problem.upper_bound(actuator) = problem.reference(actuator) + ReachableDelta;
	}

	problem.axis_weight(0) = 8.f;
	problem.axis_weight(1) = 8.f;
	problem.axis_weight(2) = 1.f;
	problem.axis_weight(3) = 4.f;

	problem.effectiveness(0, 0) = -1.f;
	problem.effectiveness(0, 1) = 1.f;
	problem.effectiveness(0, 2) = 1.f;
	problem.effectiveness(0, 3) = -1.f;
	problem.effectiveness(1, 0) = 1.f;
	problem.effectiveness(1, 1) = -1.f;
	problem.effectiveness(1, 2) = 1.f;
	problem.effectiveness(1, 3) = -1.f;
	problem.effectiveness(2, 0) = 1.f;
	problem.effectiveness(2, 1) = 1.f;
	problem.effectiveness(2, 2) = -1.f;
	problem.effectiveness(2, 3) = -1.f;
	problem.effectiveness(3, 0) = 1.f;
	problem.effectiveness(3, 1) = 1.f;
	problem.effectiveness(3, 2) = 1.f;
	problem.effectiveness(3, 3) = 1.f;

	problem.target(0) = 0.15f;
	problem.target(1) = -0.12f;
	problem.target(2) = 0.05f;
	problem.target(3) = 2.f;

	if (scenario == Scenario::Saturated) {
		problem.target(0) = 2.5f;
		problem.target(1) = -2.2f;
		problem.target(2) = 1.5f;
		problem.target(3) = 3.8f;

	} else if (scenario == Scenario::NearDegenerate) {
		problem.effectiveness(0, 0) = 1.f;
		problem.effectiveness(0, 1) = 1.f;
		problem.effectiveness(0, 2) = 1.f;
		problem.effectiveness(0, 3) = 1.f;
		problem.effectiveness(1, 0) = 1.f;
		problem.effectiveness(1, 1) = 1.01f;
		problem.effectiveness(1, 2) = 0.99f;
		problem.effectiveness(1, 3) = 1.02f;
		problem.effectiveness(2, 0) = 1.f;
		problem.effectiveness(2, 1) = 0.99f;
		problem.effectiveness(2, 2) = 1.02f;
		problem.effectiveness(2, 3) = 0.98f;
		problem.effectiveness(3, 0) = 1.f;
		problem.effectiveness(3, 1) = 1.02f;
		problem.effectiveness(3, 2) = 0.98f;
		problem.effectiveness(3, 3) = 1.01f;
		problem.target(0) = 2.f;
		problem.target(1) = 2.01f;
		problem.target(2) = 1.99f;
		problem.target(3) = 2.02f;
	}

	return problem;
}

bool vehicleIsFreshAndDisarmed(uORB::Subscription &vehicle_status_sub, uORB::Subscription &actuator_armed_sub)
{
	vehicle_status_s vehicle_status{};
	actuator_armed_s actuator_armed{};

	if (!vehicle_status_sub.copy(&vehicle_status) || (vehicle_status.timestamp == 0)) {
		PX4_ERR("vehicle_status unavailable");
		return false;
	}

	if (!actuator_armed_sub.copy(&actuator_armed) || (actuator_armed.timestamp == 0)) {
		PX4_ERR("actuator_armed unavailable");
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((vehicle_status.timestamp > now) || (now - vehicle_status.timestamp > VehicleStatusTimeout)
	    || (actuator_armed.timestamp > now) || (now - actuator_armed.timestamp > VehicleStatusTimeout)) {
		PX4_ERR("arming status stale");
		return false;
	}

	if ((vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED) || vehicle_status.calibration_enabled
	    || vehicle_status.rc_calibration_in_progress || actuator_armed.armed || actuator_armed.prearmed
	    || actuator_armed.in_esc_calibration_mode) {
		PX4_ERR("benchmark requires disarmed state with no calibration active");
		return false;
	}

	return true;
}

bool parsePositiveInt(const char *text, int maximum, int &value)
{
	errno = 0;
	char *end = nullptr;
	const long parsed = strtol(text, &end, 10);

	if ((errno != 0) || (end == text) || (*end != '\0') || (parsed <= 0) || (parsed > maximum)) {
		return false;
	}

	value = static_cast<int>(parsed);
	return true;
}

bool runScenario(Type4WeightedAllocator &allocator, uORB::Subscription &vehicle_status_sub,
		 uORB::Subscription &actuator_armed_sub, Scenario scenario, int sample_count, uint32_t budget_us,
		 uint32_t period_us)
{
	auto problem = makeProblem(scenario);
	double worst_normalized_kkt_residual = 0.0;

	for (int sample = 0; sample < sample_count; ++sample) {
		if (cancellation_requested != 0) {
			PX4_WARN("%s cancelled before sample %d", scenarioName(scenario), sample);
			return false;
		}

		const hrt_abstime release_time = hrt_absolute_time();

		if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
			PX4_ERR("%s aborted before sample %d", scenarioName(scenario), sample);
			return false;
		}

		if (scenario == Scenario::RapidlySwitching) {
			const float direction = (sample & 1) ? 1.f : -1.f;
			problem.target(0) = direction * 2.1f;
			problem.target(1) = -direction * 1.8f;
			problem.target(2) = direction * 0.9f;
			problem.target(3) = (sample & 2) ? 3.7f : 1.2f;
		}

		const hrt_abstime start = hrt_absolute_time();
		const Type4WeightedAllocator::Result result = allocator.solve(problem);
		const hrt_abstime elapsed = hrt_absolute_time() - start;

		if (elapsed > UINT32_MAX) {
			PX4_ERR("%s timing overflow", scenarioName(scenario));
			return false;
		}

		timing_samples[sample] = static_cast<uint32_t>(elapsed);

		const double normalized_kkt_residual = result.normalized_kkt_residual_inf;

		if ((result.status != Type4WeightedAllocator::Status::Success) || !result.solution.isAllFinite()
		    || !PX4_ISFINITE(result.objective) || !PX4_ISFINITE(result.objective_scale)
		    || !PX4_ISFINITE(normalized_kkt_residual) || (result.objective_scale < 1.0)
		    || (result.primal_residual_inf > static_cast<double>(NormalizedKktTolerance))
		    || (normalized_kkt_residual > static_cast<double>(NormalizedKktTolerance))
		    || (result.faces_evaluated != Type4WeightedAllocator::NumFaces) || (result.linear_solves != 65)) {
			PX4_ERR("%s solve validation failed at sample %d", scenarioName(scenario), sample);
			return false;
		}

		worst_normalized_kkt_residual = fmax(worst_normalized_kkt_residual, normalized_kkt_residual);

		if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
			PX4_ERR("%s armed during sample %d", scenarioName(scenario), sample);
			return false;
		}

		const hrt_abstime sample_elapsed = hrt_absolute_time() - release_time;

		if (sample_elapsed < period_us) {
			px4_usleep(period_us - sample_elapsed);
		}

		if (cancellation_requested != 0) {
			PX4_WARN("%s cancelled after sample %d", scenarioName(scenario), sample);
			return false;
		}
	}

	const TimingSummary summary = summarizeTimings(sample_count);
	PX4_INFO("%-17s n=%d median=%lu us p95=%lu us p99=%lu us worst=%lu us kkt_norm=%.3g",
		 scenarioName(scenario), sample_count, static_cast<unsigned long>(summary.median),
		 static_cast<unsigned long>(summary.p95), static_cast<unsigned long>(summary.p99),
		 static_cast<unsigned long>(summary.worst), worst_normalized_kkt_residual);

	if (summary.worst > budget_us) {
		PX4_ERR("%s exceeded %lu us budget", scenarioName(scenario), static_cast<unsigned long>(budget_us));
		return false;
	}

	return true;
}

void printUsage()
{
	PX4_INFO("usage: type4_bench [-n samples] [-b budget_us] [-p period_us]");
	PX4_INFO("  deterministic weighted allocator timing benchmark; publishes no actuator outputs");
	PX4_INFO("  checks disarmed, unprearmed, non-calibrating state before and after every timed solve");
	PX4_INFO("  -p is a minimum pacing interval; only allocator.solve() is timed");
	PX4_INFO("  budget is checked against the worst sample after each scenario");
	PX4_INFO("  defaults: -n %d -b %lu -p %lu", DefaultSamples, static_cast<unsigned long>(DefaultBudgetUs),
		 static_cast<unsigned long>(DefaultPeriodUs));
}

} // namespace

extern "C" __EXPORT int type4_bench_main(int argc, char *argv[])
{
	const px4_task_t current_task = px4_getpid();

	if (current_task < 0) {
		PX4_ERR("failed to identify benchmark task");
		return 1;
	}

	for (;;) {
		const px4_task_t observed_owner = benchmark_owner.load();

		if (observed_owner >= 0) {
			bool owner_alive = false;

#if defined(__PX4_NUTTX)
			errno = 0;
			owner_alive = (kill(observed_owner, 0) == 0) || (errno == EPERM);
#else
			const int probe_result = px4_task_kill(observed_owner, 0);
			owner_alive = (probe_result == 0) || (probe_result == EPERM);
#endif

			if (owner_alive) {
				PX4_ERR("benchmark already running");
				return 1;
			}
		}

		px4_task_t expected_owner = observed_owner;

		if (benchmark_owner.compare_exchange(&expected_owner, current_task)) {
			break;
		}
	}

	BenchmarkOwnerGuard benchmark_owner_guard{current_task};
	cancellation_requested = 0;

#if defined(__PX4_NUTTX)
	struct sigaction cancellation_action {};
	cancellation_action.sa_handler = handleInterrupt;

	if (sigemptyset(&cancellation_action.sa_mask) != 0) {
		PX4_ERR("failed to initialize cancellation handler");
		return 1;
	}

	cancellation_action.sa_flags = 0;

	struct sigaction previous_interrupt_action {};

	if (sigaction(SIGINT, &cancellation_action, &previous_interrupt_action) != 0) {
		PX4_ERR("failed to install cancellation handler");
		return 1;
	}

	SignalActionGuard interrupt_handler_guard{SIGINT, previous_interrupt_action};

	struct sigaction previous_termination_action {};

	if (sigaction(SIGTERM, &cancellation_action, &previous_termination_action) != 0) {
		PX4_ERR("failed to install termination handler");
		return 1;
	}

	SignalActionGuard termination_handler_guard{SIGTERM, previous_termination_action};

#endif

	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription actuator_armed_sub{ORB_ID(actuator_armed)};
	int sample_count = DefaultSamples;
	int budget_us = DefaultBudgetUs;
	int period_us = DefaultPeriodUs;

	for (int argument = 1; argument < argc; ++argument) {
		const bool sample_option = strcmp(argv[argument], "-n") == 0;
		const bool budget_option = strcmp(argv[argument], "-b") == 0;
		const bool period_option = strcmp(argv[argument], "-p") == 0;

		if (!sample_option && !budget_option && !period_option) {
			printUsage();
			return 1;
		}

		if (++argument >= argc) {
			printUsage();
			return 1;
		}

		if (sample_option) {
			if (!parsePositiveInt(argv[argument], MaxSamples, sample_count)) {
				printUsage();
				return 1;
			}

		} else if (budget_option) {
			if (!parsePositiveInt(argv[argument], INT_MAX, budget_us)) {
				printUsage();
				return 1;
			}

		} else if (!parsePositiveInt(argv[argument], INT_MAX, period_us)) {
			printUsage();
			return 1;
		}
	}

	if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
		return 1;
	}

	if (budget_us > period_us) {
		PX4_ERR("timing budget must not exceed minimum pacing interval");
		return 1;
	}

	PX4_INFO("Phase-1 standalone allocator microbenchmark; budget=%d us minimum_interval=%d us", budget_us, period_us);
	Type4WeightedAllocator allocator;

	for (uint8_t scenario_index = 0; scenario_index <= static_cast<uint8_t>(Scenario::RapidlySwitching); ++scenario_index) {
		if (!runScenario(allocator, vehicle_status_sub, actuator_armed_sub,
				 static_cast<Scenario>(scenario_index), sample_count, static_cast<uint32_t>(budget_us),
				 static_cast<uint32_t>(period_us))) {
			return 1;
		}
	}

	if (cancellation_requested != 0) {
		PX4_WARN("allocator microbenchmark cancelled");
		return 1;
	}

	if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
		return 1;
	}

	PX4_INFO("allocator microbenchmark thresholds passed; full Type 4 timing and resource gate remains open");
	return 0;
}
