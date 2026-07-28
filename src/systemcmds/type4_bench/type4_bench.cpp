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

#include <Type4WeightedAllocator.hpp>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

using type4_timing_bench::Type4WeightedAllocator;
using type4_bench_platform::LatencySnapshot;
using type4_bench_platform::RuntimeSnapshot;
using type4_bench_platform::StackUsage;

namespace
{

constexpr int DefaultSamples = 1000;
constexpr int MaxSamples = 2048;
constexpr int GyroIntervalSamples = 256;
constexpr float NormalizedKktTolerance = 1e-3f;
constexpr float BenchmarkSlewTraversalFallback = 0.5f;
constexpr float DefaultResourceLimitPercent = 95.f;
constexpr double MaxObservedRateErrorFraction = 0.25;
constexpr hrt_abstime VehicleStatusTimeout = 2 * 1000 * 1000;
constexpr hrt_abstime CpuloadTimeout = 2 * 1000 * 1000;
constexpr uint32_t GyroPollTimeoutMs = 1000;

uint32_t timing_samples[MaxSamples];
uint32_t iteration_samples[MaxSamples];
uint32_t gyro_interval_samples[GyroIntervalSamples];
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

struct DistributionSummary {
	uint32_t minimum;
	uint32_t median;
	uint32_t p95;
	uint32_t p99;
	uint32_t maximum;
};

struct ResourceSample {
	float cpu_load{-1.f};
	float ram_usage{-1.f};
	hrt_abstime timestamp{0};
	bool valid{false};
};

struct ScenarioMetrics {
	DistributionSummary timing{};
	DistributionSummary iterations{};
	uint32_t warm_attempts{0};
	uint32_t warm_hits{0};
	uint32_t faces_evaluated{0};
	uint32_t linear_solves{0};
	uint32_t worst_release_lateness{0};
	uint32_t skipped_releases{0};
	uint32_t solver_deadline_misses{0};
	uint32_t loop_deadline_misses{0};
	double worst_normalized_kkt{0.0};
	double worst_primal_residual{0.0};
	float peak_cpu_load{0.f};
	float peak_ram_usage{0.f};
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

struct RuntimeMeasurementGuard {
	explicit RuntimeMeasurementGuard(bool active) :
		_active(active)
	{
	}

	~RuntimeMeasurementGuard()
	{
		if (_active) {
			type4_bench_platform::endRuntimeMeasurement();
		}
	}

private:
	bool _active;
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

DistributionSummary summarize(uint32_t *samples, int count)
{
	qsort(samples, count, sizeof(samples[0]), compareUint32);
	return {
		.minimum = samples[0],
		.median = percentile(samples, count, 50, 100),
		.p95 = percentile(samples, count, 95, 100),
		.p99 = percentile(samples, count, 99, 100),
		.maximum = samples[count - 1],
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

bool getIntParameter(const char *name, int32_t &value)
{
	const param_t handle = param_find(name);
	return (handle != PARAM_INVALID) && (param_get(handle, &value) == PX4_OK);
}

bool getFloatParameter(const char *name, float &value)
{
	const param_t handle = param_find(name);
	return (handle != PARAM_INVALID) && (param_get(handle, &value) == PX4_OK) && PX4_ISFINITE(value);
}

bool readResourceSample(uORB::Subscription &cpuload_sub, ResourceSample &sample, bool require_newer = false)
{
	cpuload_s cpuload{};

	if (!cpuload_sub.copy(&cpuload) || (cpuload.timestamp == 0)) {
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((cpuload.timestamp > now) || (now - cpuload.timestamp > CpuloadTimeout)
	    || !PX4_ISFINITE(cpuload.load) || !PX4_ISFINITE(cpuload.ram_usage)
	    || (cpuload.load < 0.f) || (cpuload.load > 1.f) || (cpuload.ram_usage < 0.f) || (cpuload.ram_usage > 1.f)
	    || (require_newer && sample.valid && (cpuload.timestamp <= sample.timestamp))) {
		return false;
	}

	sample.cpu_load = cpuload.load;
	sample.ram_usage = cpuload.ram_usage;
	sample.timestamp = cpuload.timestamp;
	sample.valid = true;
	return true;
}

bool measureGyroIntervals(int32_t configured_rate_hz, DistributionSummary &summary)
{
	orb_sub_t gyro_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));

	if (gyro_sub < 0) {
		PX4_ERR("failed to subscribe vehicle_angular_velocity");
		return false;
	}

	px4_pollfd_struct_t poll_fd{};
	poll_fd.fd = gyro_sub;
	poll_fd.events = POLLIN;
	vehicle_angular_velocity_s angular_velocity{};
	uint64_t previous_timestamp_sample = 0;
	int intervals = 0;
	bool valid = true;

	while (valid && (intervals < GyroIntervalSamples)) {
		const int poll_result = px4_poll(&poll_fd, 1, GyroPollTimeoutMs);

		if ((poll_result <= 0) || !(poll_fd.revents & POLLIN)
		    || (orb_copy(ORB_ID(vehicle_angular_velocity), gyro_sub, &angular_velocity) != PX4_OK)
		    || (angular_velocity.timestamp_sample == 0)) {
			valid = false;
			break;
		}

		if (previous_timestamp_sample != 0) {
			if ((angular_velocity.timestamp_sample <= previous_timestamp_sample)
			    || (angular_velocity.timestamp_sample - previous_timestamp_sample > UINT32_MAX)) {
				valid = false;
				break;
			}

			gyro_interval_samples[intervals++] =
				static_cast<uint32_t>(angular_velocity.timestamp_sample - previous_timestamp_sample);
		}

		previous_timestamp_sample = angular_velocity.timestamp_sample;
	}

	orb_unsubscribe(gyro_sub);

	if (!valid || (intervals != GyroIntervalSamples)) {
		PX4_ERR("vehicle_angular_velocity interval measurement failed");
		return false;
	}

	summary = summarize(gyro_interval_samples, intervals);
	const double expected_period_us = 1000000.0 / static_cast<double>(configured_rate_hz);
	const double relative_median_error = fabs(static_cast<double>(summary.median) - expected_period_us) / expected_period_us;

	if (relative_median_error > MaxObservedRateErrorFraction) {
		PX4_ERR("observed gyro median %lu us inconsistent with configured %.1f us",
			static_cast<unsigned long>(summary.median), expected_period_us);
		return false;
	}

	return true;
}

void applyReachableBounds(Type4WeightedAllocator::Problem &problem,
			  const Type4WeightedAllocator::ActuatorVector &previous_command,
			  const float slew_traversal_time[Type4WeightedAllocator::NumActuators], float dt)
{
	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		const float delta = dt / slew_traversal_time[actuator];
		problem.reference(actuator) = previous_command(actuator);
		problem.lower_bound(actuator) = fmaxf(0.f, previous_command(actuator) - delta);
		problem.upper_bound(actuator) = fminf(1.f, previous_command(actuator) + delta);
	}
}

bool runScenario(Type4WeightedAllocator &allocator, uORB::Subscription &vehicle_status_sub,
		 uORB::Subscription &actuator_armed_sub, uORB::Subscription &cpuload_sub, ResourceSample &latest_resources,
		 Scenario scenario, int sample_count, uint32_t budget_us, uint32_t period_us,
		 const float slew_traversal_time[Type4WeightedAllocator::NumActuators], ScenarioMetrics &metrics)
{
	auto problem = makeProblem(scenario);
	Type4WeightedAllocator::ActuatorVector previous_command{};

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		previous_command(actuator) = 0.5f;
	}

	allocator.reset();
	const hrt_abstime scenario_epoch = hrt_absolute_time() + period_us;
	uint64_t release_index = 0;

	for (int sample = 0; sample < sample_count; ++sample) {
		if (cancellation_requested != 0) {
			PX4_WARN("%s cancelled before sample %d", scenarioName(scenario), sample);
			return false;
		}

		const hrt_abstime scheduled_release = scenario_epoch + release_index * period_us;
		hrt_abstime now = hrt_absolute_time();

		if (now < scheduled_release) {
			px4_usleep(scheduled_release - now);
			now = hrt_absolute_time();
		}

		if (now > scheduled_release) {
			const hrt_abstime lateness = now - scheduled_release;
			metrics.worst_release_lateness = fmax(metrics.worst_release_lateness,
							      static_cast<uint32_t>(fmin(static_cast<double>(lateness), static_cast<double>(UINT32_MAX))));

			const uint64_t missed_periods = lateness / period_us;
			metrics.skipped_releases += missed_periods;
			release_index += missed_periods;
		}

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

		applyReachableBounds(problem, previous_command, slew_traversal_time, period_us * 1e-6f);
		const hrt_abstime start = hrt_absolute_time();
		const Type4WeightedAllocator::Result result = allocator.solve(problem);
		const hrt_abstime elapsed = hrt_absolute_time() - start;

		if (elapsed > UINT32_MAX) {
			PX4_ERR("%s timing overflow", scenarioName(scenario));
			return false;
		}

		timing_samples[sample] = static_cast<uint32_t>(elapsed);
		iteration_samples[sample] = result.iterations;
		metrics.warm_attempts += result.warm_start_attempted;
		metrics.warm_hits += result.warm_start_hit;
		metrics.faces_evaluated += result.faces_evaluated;
		metrics.linear_solves += result.linear_solves;
		metrics.solver_deadline_misses += elapsed > budget_us;

		const double normalized_kkt_residual = result.normalized_kkt_residual_inf;

		if ((result.status != Type4WeightedAllocator::Status::Success) || !result.solution.isAllFinite()
		    || !PX4_ISFINITE(result.objective) || !PX4_ISFINITE(result.objective_scale)
		    || !PX4_ISFINITE(normalized_kkt_residual) || (result.objective_scale < 1.0)
		    || (result.primal_residual_inf > static_cast<double>(NormalizedKktTolerance))
		    || (normalized_kkt_residual > static_cast<double>(NormalizedKktTolerance))
		    || (result.iterations == 0) || (result.iterations > Type4WeightedAllocator::MaxIterations)
		    || (result.faces_evaluated != result.iterations) || (result.linear_solves > result.faces_evaluated)) {
			PX4_ERR("%s solve validation failed at sample %d", scenarioName(scenario), sample);
			return false;
		}

		previous_command = result.solution;
		metrics.worst_normalized_kkt = fmax(metrics.worst_normalized_kkt, normalized_kkt_residual);
		metrics.worst_primal_residual = fmax(metrics.worst_primal_residual, result.primal_residual_inf);

		if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
			PX4_ERR("%s armed during sample %d", scenarioName(scenario), sample);
			return false;
		}

		if (readResourceSample(cpuload_sub, latest_resources, true)) {
			metrics.peak_cpu_load = fmaxf(metrics.peak_cpu_load, latest_resources.cpu_load);
			metrics.peak_ram_usage = fmaxf(metrics.peak_ram_usage, latest_resources.ram_usage);
		}

		metrics.loop_deadline_misses += hrt_absolute_time() > (scheduled_release + period_us);
		release_index++;
	}

	metrics.timing = summarize(timing_samples, sample_count);
	metrics.iterations = summarize(iteration_samples, sample_count);
	const float warm_rate = (metrics.warm_attempts > 0) ? 100.f * metrics.warm_hits / metrics.warm_attempts : 0.f;
	PX4_INFO("%-17s n=%d solve_us=%lu/%lu/%lu/%lu iter=%lu/%lu/%lu/%lu warm=%lu/%lu %.1f%%",
		 scenarioName(scenario), sample_count,
		 static_cast<unsigned long>(metrics.timing.median), static_cast<unsigned long>(metrics.timing.p95),
		 static_cast<unsigned long>(metrics.timing.p99), static_cast<unsigned long>(metrics.timing.maximum),
		 static_cast<unsigned long>(metrics.iterations.median), static_cast<unsigned long>(metrics.iterations.p95),
		 static_cast<unsigned long>(metrics.iterations.p99), static_cast<unsigned long>(metrics.iterations.maximum),
		 static_cast<unsigned long>(metrics.warm_hits), static_cast<unsigned long>(metrics.warm_attempts), (double)warm_rate);
	PX4_INFO("  faces=%lu linear=%lu jitter_worst=%lu us skip=%lu solve_miss=%lu loop_miss=%lu kkt=%.3g primal=%.3g",
		 static_cast<unsigned long>(metrics.faces_evaluated), static_cast<unsigned long>(metrics.linear_solves),
		 static_cast<unsigned long>(metrics.worst_release_lateness), static_cast<unsigned long>(metrics.skipped_releases),
		 static_cast<unsigned long>(metrics.solver_deadline_misses), static_cast<unsigned long>(metrics.loop_deadline_misses),
		 metrics.worst_normalized_kkt, metrics.worst_primal_residual);

	return (metrics.timing.maximum <= budget_us) && (metrics.skipped_releases == 0)
	       && (metrics.solver_deadline_misses == 0) && (metrics.loop_deadline_misses == 0);
}

void printLatencyDelta(const LatencySnapshot &before, const LatencySnapshot &after)
{
	PX4_INFO_RAW("hrt latency delta:");

	for (int index = 0; index < LATENCY_BUCKET_COUNT; ++index) {
		PX4_INFO_RAW(" <=%u:%lu", before.bucket[index],
			     static_cast<unsigned long>(after.counter[index] - before.counter[index]));
	}

	PX4_INFO_RAW(" >%u:%lu\n", before.bucket[LATENCY_BUCKET_COUNT - 1],
		     static_cast<unsigned long>(after.counter[LATENCY_BUCKET_COUNT] - before.counter[LATENCY_BUCKET_COUNT]));
}

void printUsage()
{
	PX4_INFO("usage: type4_bench [-n samples] [-b budget_us]");
	PX4_INFO("  disarmed Phase-1 weighted allocator timing/resource benchmark; publishes no actuator outputs");
	PX4_INFO("  intended period comes from IMU_GYRO_RATEMAX; default budget is 20%% of measured minimum period");
	PX4_INFO("  -b may only make the default budget stricter");
	PX4_INFO("  default samples: %d", DefaultSamples);
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
	uORB::Subscription cpuload_sub{ORB_ID(cpuload)};
	int sample_count = DefaultSamples;
	int budget_override_us = 0;

	for (int argument = 1; argument < argc; ++argument) {
		const bool sample_option = strcmp(argv[argument], "-n") == 0;
		const bool budget_option = strcmp(argv[argument], "-b") == 0;

		if (!sample_option && !budget_option) {
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

		} else if (!parsePositiveInt(argv[argument], INT_MAX, budget_override_us)) {
			printUsage();
			return 1;
		}
	}

	if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
		return 1;
	}

	int32_t configured_rate_hz = 0;

	if (!getIntParameter("IMU_GYRO_RATEMAX", configured_rate_hz)
	    || (configured_rate_hz < 50) || (configured_rate_hz > 10000)) {
		PX4_ERR("invalid IMU_GYRO_RATEMAX");
		return 1;
	}

	DistributionSummary gyro_intervals{};

	if (!measureGyroIntervals(configured_rate_hz, gyro_intervals)) {
		return 1;
	}

	const uint32_t configured_period_us = static_cast<uint32_t>(lround(1000000.0 / configured_rate_hz));
	const uint32_t accepted_period_us = fmin(configured_period_us, gyro_intervals.minimum);
	const uint32_t provisional_budget_us = accepted_period_us / 5;
	const uint32_t budget_us = (budget_override_us > 0) ? static_cast<uint32_t>(budget_override_us) : provisional_budget_us;

	if ((budget_us == 0) || (budget_us > provisional_budget_us)) {
		PX4_ERR("budget must be at most 20%% of accepted period (%lu us)",
			static_cast<unsigned long>(provisional_budget_us));
		return 1;
	}

	float slew_traversal_time[Type4WeightedAllocator::NumActuators] {};

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		char parameter_name[16];
		snprintf(parameter_name, sizeof(parameter_name), "CA_R%d_SLEW", actuator);
		float configured_slew = 0.f;

		if (!getFloatParameter(parameter_name, configured_slew) || (configured_slew < 0.f)) {
			PX4_ERR("invalid %s", parameter_name);
			return 1;
		}

		slew_traversal_time[actuator] = (configured_slew > 0.f) ? configured_slew : BenchmarkSlewTraversalFallback;
		PX4_INFO("%s=%.3f s; benchmark traversal=%.3f s%s", parameter_name, (double)configured_slew,
			 (double)slew_traversal_time[actuator], configured_slew > 0.f ? "" : " (fallback)");
	}

	float cpu_limit_percent = DefaultResourceLimitPercent;
	float ram_limit_percent = DefaultResourceLimitPercent;
	float configured_limit = 0.f;

	if (getFloatParameter("COM_CPU_MAX", configured_limit) && (configured_limit > 0.f)) {
		cpu_limit_percent = configured_limit;
	}

	if (getFloatParameter("COM_RAM_MAX", configured_limit) && (configured_limit > 0.f)) {
		ram_limit_percent = configured_limit;
	}

	ResourceSample latest_resources{};

	if (!readResourceSample(cpuload_sub, latest_resources)) {
		PX4_ERR("fresh cpuload unavailable");
		return 1;
	}

	PX4_INFO("Phase-1 allocator gate: rate=%ld Hz period=%lu us gyro_us=%lu/%lu/%lu/%lu/%lu budget=%lu us",
		 static_cast<long>(configured_rate_hz), static_cast<unsigned long>(configured_period_us),
		 static_cast<unsigned long>(gyro_intervals.minimum), static_cast<unsigned long>(gyro_intervals.median),
		 static_cast<unsigned long>(gyro_intervals.p95), static_cast<unsigned long>(gyro_intervals.p99),
		 static_cast<unsigned long>(gyro_intervals.maximum), static_cast<unsigned long>(budget_us));
	PX4_INFO("baseline CPU=%.1f%% RAM=%.1f%% limits=%.1f%%/%.1f%%",
		 (double)(latest_resources.cpu_load * 100.f), (double)(latest_resources.ram_usage * 100.f),
		 (double)cpu_limit_percent, (double)ram_limit_percent);

	const bool runtime_monitor_active = type4_bench_platform::beginRuntimeMeasurement();
	RuntimeMeasurementGuard runtime_guard{runtime_monitor_active};

	if (!runtime_monitor_active) {
		PX4_ERR("scheduler runtime instrumentation unavailable");
		return 1;
	}

	RuntimeSnapshot runtime_before{};
	LatencySnapshot latency_before{};
	type4_bench_platform::captureLatencySnapshot(latency_before);

	if (!type4_bench_platform::captureRuntimeSnapshot(current_task, runtime_before)) {
		PX4_ERR("failed to locate benchmark or wq:rate_ctrl task");
		return 1;
	}

	Type4WeightedAllocator allocator;
	uint32_t overall_worst_solve = 0;
	uint32_t overall_worst_lateness = 0;
	float peak_cpu_load = latest_resources.cpu_load;
	float peak_ram_usage = latest_resources.ram_usage;

	for (uint8_t scenario_index = 0; scenario_index <= static_cast<uint8_t>(Scenario::RapidlySwitching); ++scenario_index) {
		ScenarioMetrics metrics{};

		if (!runScenario(allocator, vehicle_status_sub, actuator_armed_sub, cpuload_sub, latest_resources,
				 static_cast<Scenario>(scenario_index), sample_count, budget_us, configured_period_us,
				 slew_traversal_time, metrics)) {
			PX4_ERR("%s scenario failed predeclared timing/deadline gate", scenarioName(static_cast<Scenario>(scenario_index)));
			return 1;
		}

		overall_worst_solve = fmax(overall_worst_solve, metrics.timing.maximum);
		overall_worst_lateness = fmax(overall_worst_lateness, metrics.worst_release_lateness);
		peak_cpu_load = fmaxf(peak_cpu_load, metrics.peak_cpu_load);
		peak_ram_usage = fmaxf(peak_ram_usage, metrics.peak_ram_usage);
	}

	if (cancellation_requested != 0) {
		PX4_WARN("allocator benchmark cancelled");
		return 1;
	}

	if (!vehicleIsFreshAndDisarmed(vehicle_status_sub, actuator_armed_sub)) {
		return 1;
	}

	const hrt_abstime resource_wait_start = hrt_absolute_time();
	bool resource_updated = false;

	while (!resource_updated && (hrt_elapsed_time(&resource_wait_start) <= CpuloadTimeout)) {
		resource_updated = readResourceSample(cpuload_sub, latest_resources, true);

		if (!resource_updated) {
			px4_usleep(10 * 1000);
		}
	}

	if (!resource_updated) {
		PX4_ERR("no fresh cpuload sample accumulated during benchmark");
		return 1;
	}

	peak_cpu_load = fmaxf(peak_cpu_load, latest_resources.cpu_load);
	peak_ram_usage = fmaxf(peak_ram_usage, latest_resources.ram_usage);
	RuntimeSnapshot runtime_after{};
	StackUsage stack_usage{};
	LatencySnapshot latency_after{};

	if (!type4_bench_platform::captureRuntimeSnapshot(current_task, runtime_after)
	    || !type4_bench_platform::captureStackUsage(current_task, stack_usage)) {
		PX4_ERR("failed to capture scheduler runtime or stack usage");
		return 1;
	}

	type4_bench_platform::captureLatencySnapshot(latency_after);
	const uint64_t runtime_interval = runtime_after.timestamp - runtime_before.timestamp;
	const uint64_t rate_ctrl_runtime = runtime_after.rate_ctrl_runtime - runtime_before.rate_ctrl_runtime;
	const uint64_t benchmark_runtime = runtime_after.benchmark_runtime - runtime_before.benchmark_runtime;
	const float rate_ctrl_share = (runtime_interval > 0) ? 100.f * rate_ctrl_runtime / runtime_interval : 0.f;
	const float benchmark_share = (runtime_interval > 0) ? 100.f * benchmark_runtime / runtime_interval : 0.f;
	const uint32_t required_stack_free = fmax(512u, stack_usage.total / 4u);
	const int64_t conservative_margin = static_cast<int64_t>(gyro_intervals.minimum)
					    - static_cast<int64_t>(overall_worst_lateness)
					    - static_cast<int64_t>(overall_worst_solve);

	PX4_INFO("runtime interval=%llu us wq:rate_ctrl=%llu us %.2f%% benchmark=%llu us %.2f%%",
		 static_cast<unsigned long long>(runtime_interval), static_cast<unsigned long long>(rate_ctrl_runtime),
		 (double)rate_ctrl_share, static_cast<unsigned long long>(benchmark_runtime), (double)benchmark_share);
	PX4_INFO("stack used/free/total=%lu/%lu/%lu bytes reserve_required=%lu bytes",
		 static_cast<unsigned long>(stack_usage.total - stack_usage.free), static_cast<unsigned long>(stack_usage.free),
		 static_cast<unsigned long>(stack_usage.total), static_cast<unsigned long>(required_stack_free));
	PX4_INFO("peak CPU=%.1f%% RAM=%.1f%% conservative margin=%lld us",
		 (double)(peak_cpu_load * 100.f), (double)(peak_ram_usage * 100.f), static_cast<long long>(conservative_margin));
	printLatencyDelta(latency_before, latency_after);

	if ((rate_ctrl_runtime == 0) || (stack_usage.free < required_stack_free)
	    || (peak_cpu_load * 100.f >= cpu_limit_percent) || (peak_ram_usage * 100.f >= ram_limit_percent)
	    || (conservative_margin <= 0)) {
		PX4_ERR("Phase-1 timing/resource gate failed");
		return 1;
	}

	PX4_INFO("Phase-1 timing/resource gate passed; Type 4 controller/theorem/HIL/flight gates remain open");
	return 0;
}
