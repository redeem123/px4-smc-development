#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


OFFBOARD_NAV_STATE = 14
ARMING_STATE_ARMED = 2
AXES = ("roll", "pitch", "yaw")


def square_waypoints(side_m, altitude_m):
    z = -altitude_m
    return [
        ("takeoff", np.array([0.0, 0.0, z])),
        ("east", np.array([side_m, 0.0, z])),
        ("north", np.array([side_m, side_m, z])),
        ("west", np.array([0.0, side_m, z])),
        ("home", np.array([0.0, 0.0, z])),
    ]


def dataset(ulog, name):
    return next(data.data for data in ulog.data_list if data.name == name)


def optional_dataset(ulog, name):
    return next((data.data for data in ulog.data_list if data.name == name), None)


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return {
            "n": 0,
            "mean": np.nan,
            "rms": np.nan,
            "p95": np.nan,
            "max": np.nan,
        }

    return {
        "n": len(values),
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def format_stats(result):
    return (
        f"n={result['n']} mean={result['mean']:.4f} rms={result['rms']:.4f} "
        f"p95={result['p95']:.4f} max={result['max']:.4f}"
    )


def first_offboard_window(vehicle_status):
    timestamps = vehicle_status["timestamp"].astype(float) / 1e6
    offboard = (
        (vehicle_status["nav_state"] == OFFBOARD_NAV_STATE)
        & (vehicle_status["arming_state"] == ARMING_STATE_ARMED)
    )
    indexes = np.flatnonzero(offboard)

    if len(indexes) == 0:
        raise RuntimeError("no armed OFFBOARD window found")

    start_index = indexes[0]
    end_index = start_index

    while end_index + 1 < len(offboard) and offboard[end_index + 1]:
        end_index += 1

    end_timestamp = timestamps[end_index + 1] if end_index + 1 < len(timestamps) else timestamps[end_index]
    return timestamps[start_index], end_timestamp


def zero_order_hold(sample_timestamps, setpoint_timestamps, setpoints):
    indexes = np.searchsorted(setpoint_timestamps, sample_timestamps, side="right") - 1
    valid_indexes = indexes >= 0
    sampled = np.full((len(sample_timestamps), setpoints.shape[1]), np.nan)
    sampled[valid_indexes] = setpoints[indexes[valid_indexes]]
    valid_samples = np.isfinite(sampled).all(axis=1)
    return sampled, valid_samples


def vector_data(data, fields):
    return np.column_stack([data[field] for field in fields]).astype(float)


def sample_timestamp_field(data):
    return "timestamp_sample" if "timestamp_sample" in data else "timestamp"


def sample_timestamps(data):
    return data[sample_timestamp_field(data)].astype(float) / 1e6


def exact_timestamp_indexes(reference_timestamps, sampled_timestamps):
    reference_by_timestamp = {
        int(timestamp): index for index, timestamp in enumerate(reference_timestamps)
    }
    matched_reference = []
    matched_samples = []

    for sample_index, timestamp in enumerate(sampled_timestamps):
        reference_index = reference_by_timestamp.get(int(timestamp))

        if reference_index is not None:
            matched_reference.append(reference_index)
            matched_samples.append(sample_index)

    return (
        np.asarray(matched_reference, dtype=int),
        np.asarray(matched_samples, dtype=int),
    )


def fraction(values):
    values = np.asarray(values, dtype=bool)
    return float(np.mean(values)) if len(values) > 0 else np.nan


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]

    if len(values) == 0:
        return {
            "n": 0,
            "min": np.nan,
            "mean": np.nan,
            "median": np.nan,
            "rms": np.nan,
            "p95": np.nan,
            "max": np.nan,
        }

    return {
        "n": len(values),
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def torque_limit_prefix(controller_type):
    prefixes = {
        1: "MC_MPC_TMAX",
        2: "MC_MSMC_TMAX",
        3: "MC_AST_TMAX",
    }

    if controller_type not in prefixes:
        raise RuntimeError(f"unsupported rate controller type {controller_type}")

    return prefixes[controller_type]


def torque_variation_metrics(timestamps, values, cutoff_hz):
    timestamps = np.asarray(timestamps, dtype=float)
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(timestamps) & np.isfinite(values)
    timestamps = timestamps[valid]
    values = values[valid]
    variation = float(np.sum(np.abs(np.diff(values)))) if len(values) > 1 else np.nan
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else np.nan

    return {
        "high_frequency": high_frequency_metrics(timestamps, values, cutoff_hz),
        "total_variation": variation,
        "variation_per_s": variation / duration if np.isfinite(duration) and duration > 0.0 else np.nan,
    }


def torque_component_metrics(timestamps, values, cutoff_hz, saturated):
    magnitude = distribution(np.abs(values))
    dynamics = torque_variation_metrics(timestamps, values, cutoff_hz)
    return {
        "samples": magnitude["n"],
        "rms": magnitude["rms"],
        "peak": magnitude["max"],
        "high_frequency_rms": dynamics["high_frequency"]["rms"],
        "total_variation": dynamics["total_variation"],
        "variation_per_s": dynamics["variation_per_s"],
        "saturation_fraction": fraction(saturated),
    }


def quaternion_yaw(data, prefix):
    w, x, y, z = (data[f"{prefix}[{index}]"] for index in range(4))
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrapped_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def settling_time(timestamps, absolute_error, threshold, dwell_s):
    for index, timestamp in enumerate(timestamps):
        dwell_end = np.searchsorted(timestamps, timestamp + dwell_s, side="left")

        if dwell_end > index and np.all(absolute_error[index:dwell_end] <= threshold):
            return float(timestamp - timestamps[0])

    return np.nan


def yaw_step_metrics(
    ulog,
    planned_start_s,
    step_hold_s,
    commanded_step,
    final_s,
    settle_threshold,
    settle_dwell_s,
):
    attitude = dataset(ulog, "vehicle_attitude")
    trajectory = dataset(ulog, "trajectory_setpoint")
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    attitude_timestamps = attitude["timestamp"].astype(float) / 1e6
    actual_yaw = quaternion_yaw(attitude, "q")
    trajectory_timestamps = trajectory["timestamp"].astype(float) / 1e6
    yaw_setpoints = trajectory["yaw"].astype(float)[:, np.newaxis]
    sampled_yaw, valid_yaw = zero_order_hold(attitude_timestamps, trajectory_timestamps, yaw_setpoints)
    sampled_yaw = sampled_yaw[:, 0]
    search_mask = (
        (trajectory_timestamps >= planned_start_s - 1.0)
        & (trajectory_timestamps <= planned_start_s + 3.0 * step_hold_s + 2.0)
        & np.isfinite(trajectory["yaw"])
    )
    search_timestamps = trajectory_timestamps[search_mask]
    search_setpoints = trajectory["yaw"].astype(float)[search_mask]
    transition_threshold = max(np.radians(2.0), 0.25 * commanded_step)
    transition_indexes = np.flatnonzero(
        np.abs(wrapped_angle(np.diff(search_setpoints))) >= transition_threshold
    ) + 1
    transition_times = []

    for index in transition_indexes:
        timestamp = search_timestamps[index]

        if not transition_times or timestamp - transition_times[-1] > 0.25:
            transition_times.append(timestamp)

    if len(transition_times) != 3:
        raise RuntimeError(f"expected three yaw setpoint transitions, found {len(transition_times)}")

    start_s = transition_times[0]
    end_s = transition_times[-1] + step_hold_s
    sequence_mask = (
        (attitude_timestamps >= start_s)
        & (attitude_timestamps <= end_s)
        & valid_yaw
        & np.isfinite(actual_yaw)
    )
    sequence_timestamps = attitude_timestamps[sequence_mask]
    sequence_error = np.abs(wrapped_angle(sampled_yaw[sequence_mask] - actual_yaw[sequence_mask]))
    sequence_setpoints = sampled_yaw[sequence_mask]

    if len(sequence_timestamps) < 10:
        raise RuntimeError("yaw-step sequence contains insufficient attitude samples")

    segments = []

    for index, segment_start in enumerate(transition_times):
        segment_end = transition_times[index + 1] if index + 1 < len(transition_times) else end_s
        segment_mask = (
            (attitude_timestamps >= segment_start)
            & (attitude_timestamps < segment_end)
            & valid_yaw
            & np.isfinite(actual_yaw)
        )
        segment_timestamps = attitude_timestamps[segment_mask]
        segment_error = np.abs(wrapped_angle(sampled_yaw[segment_mask] - actual_yaw[segment_mask]))
        final_mask = segment_timestamps >= segment_end - min(final_s, 0.5 * (segment_end - segment_start))
        segments.append(
            {
                "target": float(np.median(sampled_yaw[segment_mask])),
                "final_error": stats(segment_error[final_mask]),
                "settling_s": settling_time(
                    segment_timestamps, segment_error, settle_threshold, settle_dwell_s
                ),
            }
        )

    rate_timestamps = angular_velocity["timestamp"].astype(float) / 1e6
    yaw_rates = angular_velocity["xyz[2]"].astype(float)
    rate_mask = (
        (rate_timestamps >= start_s)
        & (rate_timestamps <= end_s)
        & np.isfinite(yaw_rates)
    )
    target_unwrapped = np.unwrap(np.array([segment["target"] for segment in segments]))

    return {
        "start_s": float(start_s),
        "end_s": float(end_s),
        "error": stats(sequence_error),
        "segments": segments,
        "transition_times": [float(timestamp) for timestamp in transition_times],
        "commanded_excursion": float(np.max(target_unwrapped) - np.min(target_unwrapped)),
        "max_rate": float(np.max(np.abs(yaw_rates[rate_mask]))),
    }


def high_frequency_metrics(timestamps, values, cutoff_hz):
    timestamps = np.asarray(timestamps, dtype=float)
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(timestamps) & np.isfinite(values)
    timestamps = timestamps[valid]
    values = values[valid]

    if len(values) < 8 or cutoff_hz <= 0.0:
        return {"rms": np.nan, "peak_hz": np.nan, "peak_amplitude": np.nan}

    low_pass = values[0]
    residual = np.zeros_like(values)
    time_constant = 1.0 / (2.0 * np.pi * cutoff_hz)

    for index in range(1, len(values)):
        dt = np.clip(timestamps[index] - timestamps[index - 1], 1e-5, 0.1)
        alpha = dt / (time_constant + dt)
        low_pass += alpha * (values[index] - low_pass)
        residual[index] = values[index] - low_pass

    median_dt = float(np.median(np.diff(timestamps)))

    if not np.isfinite(median_dt) or median_dt <= 0.0:
        return {
            "rms": float(np.sqrt(np.mean(residual * residual))),
            "peak_hz": np.nan,
            "peak_amplitude": np.nan,
        }

    uniform_timestamps = np.arange(timestamps[0], timestamps[-1], median_dt)

    if len(uniform_timestamps) < 8:
        return {"rms": np.nan, "peak_hz": np.nan, "peak_amplitude": np.nan}

    uniform_residual = np.interp(uniform_timestamps, timestamps, residual)
    uniform_residual -= np.mean(uniform_residual)
    window = np.hanning(len(uniform_residual))
    spectrum = np.fft.rfft(uniform_residual * window)
    frequencies = np.fft.rfftfreq(len(uniform_residual), median_dt)
    valid_frequencies = frequencies >= cutoff_hz

    if not np.any(valid_frequencies):
        peak_hz = np.nan
        peak_amplitude = np.nan

    else:
        valid_indexes = np.flatnonzero(valid_frequencies)
        peak_index = valid_indexes[np.argmax(np.abs(spectrum[valid_frequencies]))]
        window_gain = max(float(np.sum(window)), 1.0)
        peak_hz = float(frequencies[peak_index])
        peak_amplitude = float(2.0 * np.abs(spectrum[peak_index]) / window_gain)

    return {
        "rms": float(np.sqrt(np.mean(residual * residual))),
        "peak_hz": peak_hz,
        "peak_amplitude": peak_amplitude,
    }


def astsmc_required_fields():
    fields = {
        "timestamp",
        "timestamp_sample",
        "configuration_valid",
        "dt_valid",
        "total_bound_valid",
        "raw_dt",
        "accepted_dt",
        "dt_min",
        "dt_max",
        "max_airborne_dt",
        "valid_update_count",
        "invalid_dt_hold_count",
        "consecutive_invalid_dt_hold_count",
        "reference_reset_count",
        "total_bound_violation_count",
        "allocator_feedback_valid",
        "allocator_feedback_stale",
        "allocator_torque_setpoint_achieved",
        "allocator_timestamp_sample",
        "allocator_feedback_age_s",
    }

    for axis in range(3):
        fields.add(f"internal_saturation[{axis}]")
        fields.add(f"nominal_saturation[{axis}]")
        fields.add(f"nominal_limit_count[{axis}]")
        fields.add(f"residual_limit_count[{axis}]")

        for prefix in (
            "rate_setpoint_raw",
            "rate_setpoint_shaped",
            "reference_acceleration",
            "reference_jerk",
            "sliding_variable_raw",
            "sliding_variable",
            "integral_state",
            "k1",
            "k2",
            "reaching_input_raw",
            "reaching_input_limited",
            "nominal_torque_raw",
            "nominal_torque_limited",
            "residual_torque_raw",
            "residual_torque_limited",
            "residual_authority",
            "variation_weight",
            "variation_regularization",
            "torque_raw",
            "torque_limited",
            "allocated_torque",
            "allocation_residual",
        ):
            fields.add(f"{prefix}[{axis}]")

    return fields


def astsmc_safety_required_fields():
    fields = {
        "timestamp",
        "timestamp_sample",
        "runtime_fault_latched",
        "runtime_fault_reason",
        "ground_containment_active",
        "ground_containment_count",
    }

    for axis in range(3):
        fields.update(
            {
                f"state_recovery_active[{axis}]",
                f"state_recovery_count[{axis}]",
                f"recovery_trigger_sliding[{axis}]",
                f"recovery_trigger_state[{axis}]",
                f"recovery_trigger_reaching[{axis}]",
            }
        )

    return fields


def analyze_astsmc_diagnostics(
    astsmc_status,
    astsmc_safety_status,
    rate_control_status,
    land_detected,
    torque_setpoint,
    parameters,
    offboard_start_s,
    offboard_end_s,
    planned_maneuver_end_s,
    torque_limits,
    yaw_transition_times=None,
    angular_velocity=None,
    rate_osc_cutoff=4.0,
):
    result = {
        "present": astsmc_status is not None,
        "missing_fields": [],
        "status_samples": 0,
        "airborne_samples": 0,
        "identity_samples": 0,
        "controller_type_fraction": np.nan,
        "astsmc_valid_fraction": np.nan,
        "configuration_valid_fraction": np.nan,
        "runtime_fault_latched": False,
        "runtime_fault_reason": 0,
        "ground_containment_active_count": 0,
        "ground_containment_count_delta": 0,
        "state_recovery_count_delta": 0,
        "wrong_direction_escape_count": 0,
        "recovery_error_threshold": np.nan,
        "recovery_threshold_valid": False,
        "runtime_valid": False,
        "dt_invalid_count": 0,
        "dt_invalid_fraction": np.nan,
        "dt_invalid_consecutive_count": 0,
        "dt_invalid_transition_cluster_count": 0,
        "counter_reset_count": 0,
        "counter_wrap_count": 0,
        "valid_update_count_delta": 0,
        "reference_reset_count_delta": 0,
        "total_bound_violation_count_delta": 0,
        "dt_min": np.nan,
        "dt_max": np.nan,
        "max_airborne_dt": np.nan,
        "timing_bounds_consistent": False,
        "accepted_dt_consistent": False,
        "nonfinite_rows": 0,
        "nonfinite_fields": [],
        "sample_rate_hz": np.nan,
        "sample_interval_s": distribution([]),
        "timestamps_monotonic": False,
        "publication_lag_s": distribution([]),
        "publication_before_sample_count": 0,
        "gains_match_parameters": False,
        "gain_max_error": np.nan,
        "axes": {},
        "allocator": {},
        "torque_timestamp_match_count": 0,
        "torque_timestamp_match_fraction": np.nan,
        "command_mismatch": {},
        "rate_timestamp_source": None,
        "rate_timestamp_match_count": 0,
        "rate_timestamp_match_fraction": np.nan,
        "rate_error_identity_max": np.nan,
        "tracking": {},
    }

    if astsmc_status is None:
        result["missing_fields"] = ["astsmc_status"]
        return result

    if astsmc_safety_status is None:
        result["missing_fields"] = ["astsmc_safety_status"]
        return result

    required_sources = (
        ("astsmc_status", astsmc_status, astsmc_required_fields()),
        ("astsmc_safety_status", astsmc_safety_status, astsmc_safety_required_fields()),
        ("rate_ctrl_status", rate_control_status, {"timestamp", "controller_type", "astsmc_valid"}),
        ("vehicle_land_detected", land_detected, {"timestamp", "landed", "maybe_landed"}),
        (
            "vehicle_torque_setpoint",
            torque_setpoint,
            {"timestamp_sample", "xyz[0]", "xyz[1]", "xyz[2]"},
        ),
    )

    for source_name, source, required_fields in required_sources:
        if source is None:
            result["missing_fields"].append(source_name)
            continue

        result["missing_fields"].extend(
            f"{source_name}.{field}" for field in sorted(required_fields.difference(source))
        )

    if result["missing_fields"]:
        return result

    maneuver_end_s = min(offboard_end_s, planned_maneuver_end_s)
    status_timestamps = astsmc_status["timestamp_sample"].astype(float) / 1e6
    status_window = (
        (status_timestamps >= offboard_start_s)
        & (status_timestamps <= maneuver_end_s)
    )
    safety_timestamps = astsmc_safety_status["timestamp_sample"].astype(float) / 1e6
    safety_window = (
        (safety_timestamps >= offboard_start_s)
        & (safety_timestamps <= maneuver_end_s)
    )
    safety_fields = sorted(astsmc_safety_required_fields() - {"timestamp", "timestamp_sample"})
    sampled_safety, valid_safety = zero_order_hold(
        status_timestamps,
        safety_timestamps,
        np.column_stack([astsmc_safety_status[field] for field in safety_fields]),
    )
    safety_at_status = {
        field: sampled_safety[:, index] for index, field in enumerate(safety_fields)
    }
    result["status_samples"] = int(np.sum(status_window))

    land_timestamps = land_detected["timestamp"].astype(float) / 1e6
    land_states = np.column_stack(
        [land_detected["landed"], land_detected["maybe_landed"]]
    ).astype(float)
    sampled_land_state, valid_land_state = zero_order_hold(
        status_timestamps, land_timestamps, land_states
    )
    # Type 3 resets only for true landed. Keep maybe_landed transition samples in
    # the diagnostic window so lifecycle holds or impulses cannot be hidden.
    airborne = valid_land_state & ~sampled_land_state[:, 0].astype(bool)
    airborne_mask = status_window & airborne & valid_safety
    airborne_indexes = np.flatnonzero(airborne_mask)
    airborne_timestamps = status_timestamps[airborne_mask]
    result["airborne_samples"] = len(airborne_indexes)

    rate_status_timestamps = rate_control_status["timestamp"].astype(float) / 1e6
    identity_mask = (
        (rate_status_timestamps >= offboard_start_s)
        & (rate_status_timestamps <= maneuver_end_s)
    )
    result["identity_samples"] = int(np.sum(identity_mask))

    if result["identity_samples"] > 0:
        result["controller_type_fraction"] = fraction(
            rate_control_status["controller_type"][identity_mask] == 3
        )
        result["astsmc_valid_fraction"] = fraction(
            rate_control_status["astsmc_valid"][identity_mask]
        )

    if result["status_samples"] > 0:
        result["configuration_valid_fraction"] = fraction(
            astsmc_status["configuration_valid"][status_window]
        )

    if np.any(safety_window):
        result["runtime_fault_latched"] = bool(
            np.any(astsmc_safety_status["runtime_fault_latched"][safety_window])
        )
        result["runtime_fault_reason"] = int(
            np.bitwise_or.reduce(
                astsmc_safety_status["runtime_fault_reason"][safety_window].astype(np.uint32),
                initial=np.uint32(0),
            )
        )
        result["ground_containment_active_count"] = int(
            np.sum(astsmc_safety_status["ground_containment_active"][safety_window])
        )

    if result["airborne_samples"] == 0:
        return result

    sample_differences = np.diff(airborne_timestamps)
    positive_differences = sample_differences[sample_differences > 0.0]
    duration = airborne_timestamps[-1] - airborne_timestamps[0]
    result["sample_rate_hz"] = (
        (result["airborne_samples"] - 1) / duration
        if result["airborne_samples"] > 1 and duration > 0.0
        else np.nan
    )
    result["sample_interval_s"] = distribution(positive_differences)
    result["timestamps_monotonic"] = bool(
        len(sample_differences) == 0 or np.all(sample_differences > 0.0)
    )

    publication_timestamps = astsmc_status["timestamp"].astype(float)[airborne_mask] / 1e6
    publication_lag = publication_timestamps - airborne_timestamps
    result["publication_lag_s"] = distribution(publication_lag)
    result["publication_before_sample_count"] = int(np.sum(publication_lag < 0.0))

    dt_valid = astsmc_status["dt_valid"][airborne_mask].astype(bool)
    def counter_deltas(values):
        values = np.asarray(values, dtype=np.uint32)
        deltas = np.zeros(len(airborne_indexes), dtype=np.uint64)
        has_previous = airborne_indexes > 0

        if not np.any(has_previous):
            return deltas, 0, 0

        current = values[airborne_indexes[has_previous]].astype(np.uint64)
        previous = values[airborne_indexes[has_previous] - 1].astype(np.uint64)
        decreases = current < previous
        wraps = decreases & (previous > (1 << 31)) & (current < (1 << 31))
        resets = decreases & ~wraps
        changes = np.zeros(len(current), dtype=np.uint64)
        normal = ~decreases
        changes[normal] = current[normal] - previous[normal]
        changes[wraps] = current[wraps] + (1 << 32) - previous[wraps]
        deltas[has_previous] = changes
        return deltas, int(np.sum(resets)), int(np.sum(wraps))

    counter_series = {
        "invalid": astsmc_status["invalid_dt_hold_count"],
        "consecutive_invalid": astsmc_status["consecutive_invalid_dt_hold_count"],
        "valid": astsmc_status["valid_update_count"],
        "reference_reset": astsmc_status["reference_reset_count"],
        "total_bound": astsmc_status["total_bound_violation_count"],
        "ground_containment": safety_at_status["ground_containment_count"],
        "state_recovery_0": safety_at_status["state_recovery_count[0]"],
        "state_recovery_1": safety_at_status["state_recovery_count[1]"],
        "state_recovery_2": safety_at_status["state_recovery_count[2]"],
    }
    counter_results = {
        name: counter_deltas(values) for name, values in counter_series.items()
    }
    result["counter_reset_count"] = sum(
        resets for _deltas, resets, _wraps in counter_results.values()
    )
    result["counter_wrap_count"] = sum(
        wraps for _deltas, _resets, wraps in counter_results.values()
    )
    invalid_deltas = counter_results["invalid"][0]
    result["dt_invalid_count"] = int(np.sum(invalid_deltas, dtype=np.uint64))
    result["dt_invalid_consecutive_count"] = int(
        np.sum(counter_results["consecutive_invalid"][0], dtype=np.uint64)
    )
    result["ground_containment_count_delta"] = int(
        np.sum(counter_results["ground_containment"][0], dtype=np.uint64)
    )
    result["state_recovery_count_delta"] = int(
        sum(
            np.sum(counter_results[f"state_recovery_{axis}"][0], dtype=np.uint64)
            for axis in range(3)
        )
    )
    recovery_error_threshold = parameters.get("MC_AST_REC_ERR")

    if recovery_error_threshold is not None:
        result["recovery_error_threshold"] = float(recovery_error_threshold)
        result["recovery_threshold_valid"] = bool(
            np.isfinite(result["recovery_error_threshold"])
            and result["recovery_error_threshold"] > 0.0
        )

    result["valid_update_count_delta"] = int(
        np.sum(counter_results["valid"][0], dtype=np.uint64)
    )
    total_update_delta = result["dt_invalid_count"] + result["valid_update_count_delta"]
    result["dt_invalid_fraction"] = (
        result["dt_invalid_count"] / total_update_delta if total_update_delta > 0 else 0.0
    )
    result["reference_reset_count_delta"] = int(
        np.sum(counter_results["reference_reset"][0], dtype=np.uint64)
    )
    result["total_bound_violation_count_delta"] = int(
        np.sum(counter_results["total_bound"][0], dtype=np.uint64)
    )
    dt_min_values = astsmc_status["dt_min"][airborne_mask].astype(float)
    dt_max_values = astsmc_status["dt_max"][airborne_mask].astype(float)
    result["dt_min"] = float(np.median(dt_min_values))
    result["dt_max"] = float(np.median(dt_max_values))
    result["max_airborne_dt"] = float(
        np.max(astsmc_status["max_airborne_dt"][airborne_mask].astype(float))
    )
    result["timing_bounds_consistent"] = bool(
        np.all(np.isfinite(dt_min_values))
        and np.all(np.isfinite(dt_max_values))
        and np.all(dt_min_values > 0.0)
        and np.all(dt_min_values <= dt_max_values)
        and np.ptp(dt_min_values) <= 1e-9
        and np.ptp(dt_max_values) <= 1e-9
    )

    if yaw_transition_times:
        invalid_event_indexes = np.flatnonzero(invalid_deltas > 0)
        invalid_times = airborne_timestamps[invalid_event_indexes]
        result["dt_invalid_transition_cluster_count"] = int(
            sum(np.any(np.abs(invalid_times - transition) <= 0.1) for transition in yaw_transition_times)
        )

    required_float_fields = ["max_airborne_dt", "allocator_feedback_age_s"]

    raw_dt_values = astsmc_status["raw_dt"][airborne_mask].astype(float)
    raw_dt_valid = np.isfinite(raw_dt_values) & (raw_dt_values > 0.0)

    if not np.all(raw_dt_valid[1:]):
        result["nonfinite_fields"].append("raw_dt")

    accepted_dt_values = astsmc_status["accepted_dt"][airborne_mask].astype(float)
    accepted_dt_finite = np.isfinite(accepted_dt_values)
    accepted_dt_positive = accepted_dt_values > 0.0
    accepted_dt_in_bounds = (
        (accepted_dt_values >= dt_min_values)
        & (accepted_dt_values <= dt_max_values)
    )
    result["accepted_dt_consistent"] = bool(
        result["timing_bounds_consistent"]
        and np.all(accepted_dt_finite == dt_valid)
        and np.all(accepted_dt_positive[dt_valid])
        and np.all(accepted_dt_in_bounds[dt_valid])
    )

    if not result["accepted_dt_consistent"]:
        result["nonfinite_fields"].append("accepted_dt")

    for axis in range(3):
        required_float_fields.extend(
            f"{prefix}[{axis}]"
            for prefix in (
                "rate_setpoint_raw",
                "rate_setpoint_shaped",
                "reference_acceleration",
                "reference_jerk",
                "sliding_variable_raw",
                "sliding_variable",
                "integral_state",
                "k1",
                "k2",
                "reaching_input_raw",
                "reaching_input_limited",
                "nominal_torque_raw",
                "nominal_torque_limited",
                "residual_torque_raw",
                "residual_torque_limited",
                "residual_authority",
                "variation_weight",
                "variation_regularization",
                "torque_raw",
                "torque_limited",
                "allocated_torque",
                "allocation_residual",
            )
        )

    raw_dt_rows_valid = raw_dt_valid.copy()
    raw_dt_rows_valid[0] = True
    finite_columns = [raw_dt_rows_valid]

    for field in required_float_fields:
        values = astsmc_status[field][airborne_mask].astype(float)
        finite_columns.append(np.isfinite(values))

        if not np.all(np.isfinite(values)):
            result["nonfinite_fields"].append(field)

    finite_matrix = np.column_stack(finite_columns)
    result["nonfinite_rows"] = int(np.sum(~np.all(finite_matrix, axis=1)))

    gain_errors = []

    for axis, suffix in enumerate(("R", "P", "Y")):
        expected_k1 = parameters.get(f"MC_AST_K1_{suffix}")
        expected_k2 = parameters.get(f"MC_AST_K2_{suffix}")
        k1 = astsmc_status[f"k1[{axis}]"][airborne_mask].astype(float)
        k2 = astsmc_status[f"k2[{axis}]"][airborne_mask].astype(float)
        reaching_raw = astsmc_status[f"reaching_input_raw[{axis}]"][airborne_mask].astype(float)
        reaching_limited = astsmc_status[f"reaching_input_limited[{axis}]"][airborne_mask].astype(float)
        nominal_raw = astsmc_status[f"nominal_torque_raw[{axis}]"][airborne_mask].astype(float)
        nominal_limited = astsmc_status[f"nominal_torque_limited[{axis}]"][airborne_mask].astype(float)
        residual_raw = astsmc_status[f"residual_torque_raw[{axis}]"][airborne_mask].astype(float)
        residual_limited = astsmc_status[f"residual_torque_limited[{axis}]"][airborne_mask].astype(float)
        residual_authority = astsmc_status[f"residual_authority[{axis}]"][airborne_mask].astype(float)
        variation_weight = astsmc_status[f"variation_weight[{axis}]"][airborne_mask].astype(float)
        variation_regularization = astsmc_status[
            f"variation_regularization[{axis}]"
        ][airborne_mask].astype(float)
        variation_active = np.abs(variation_regularization) > 1e-8
        recovery_active = safety_at_status[
            f"state_recovery_active[{axis}]"
        ][airborne_mask].astype(bool)
        recovery_count_delta = int(
            np.sum(
                counter_results[f"state_recovery_{axis}"][0],
                dtype=np.uint64,
            )
        )
        recovery_trigger_sliding = safety_at_status[
            f"recovery_trigger_sliding[{axis}]"
        ][airborne_mask].astype(float)
        recovery_trigger_state = safety_at_status[
            f"recovery_trigger_state[{axis}]"
        ][airborne_mask].astype(float)
        recovery_trigger_reaching = safety_at_status[
            f"recovery_trigger_reaching[{axis}]"
        ][airborne_mask].astype(float)
        sliding = astsmc_status[f"sliding_variable[{axis}]"][airborne_mask].astype(float)
        large_error = (
            np.abs(sliding) > result["recovery_error_threshold"]
            if result["recovery_threshold_valid"]
            else np.ones(len(sliding), dtype=bool)
        )
        wrong_direction = sliding * reaching_limited < -1e-7
        wrong_direction_escape = large_error & wrong_direction & ~recovery_active
        torque_raw = astsmc_status[f"torque_raw[{axis}]"][airborne_mask].astype(float)
        torque_limited = astsmc_status[f"torque_limited[{axis}]"][airborne_mask].astype(float)
        reaching_clamp = np.abs(reaching_raw - reaching_limited)
        saturated = astsmc_status[f"internal_saturation[{axis}]"][airborne_mask].astype(bool)
        nominal_saturated = astsmc_status[f"nominal_saturation[{axis}]"][airborne_mask].astype(bool)
        residual_clamp_active = np.abs(residual_raw - residual_limited) > 1e-6
        nominal_clamp_active = np.abs(nominal_raw - nominal_limited) > 1e-6
        decomposition_error = np.abs(torque_limited - nominal_limited - residual_limited)
        residual_excess = np.abs(residual_limited) - residual_authority
        torque_excess = np.abs(torque_limited) - torque_limits[axis]

        if expected_k1 is not None and np.isfinite(float(expected_k1)):
            gain_errors.append(float(np.max(np.abs(k1 - float(expected_k1)))))

        if expected_k2 is not None and np.isfinite(float(expected_k2)):
            gain_errors.append(float(np.max(np.abs(k2 - float(expected_k2)))))

        result["axes"][AXES[axis]] = {
            "internal_saturation_count": int(
                np.sum(
                    counter_deltas(
                        astsmc_status[f"residual_limit_count[{axis}]"]
                    )[0],
                    dtype=np.uint64,
                )
            ),
            "internal_saturation_fraction": fraction(saturated),
            "nominal_saturation_count": int(
                np.sum(
                    counter_deltas(
                        astsmc_status[f"nominal_limit_count[{axis}]"]
                    )[0],
                    dtype=np.uint64,
                )
            ),
            "nominal_saturation_fraction": fraction(nominal_saturated),
            "rate_setpoint_raw": distribution(
                astsmc_status[f"rate_setpoint_raw[{axis}]"][airborne_mask]
            ),
            "rate_setpoint_shaped": distribution(
                astsmc_status[f"rate_setpoint_shaped[{axis}]"][airborne_mask]
            ),
            "reference_acceleration": distribution(
                np.abs(astsmc_status[f"reference_acceleration[{axis}]"][airborne_mask])
            ),
            "reference_jerk": distribution(
                np.abs(astsmc_status[f"reference_jerk[{axis}]"][airborne_mask])
            ),
            "sliding_variable_raw": distribution(
                np.abs(astsmc_status[f"sliding_variable_raw[{axis}]"][airborne_mask])
            ),
            "sliding_variable": distribution(
                np.abs(astsmc_status[f"sliding_variable[{axis}]"][airborne_mask])
            ),
            "integral_state": distribution(
                np.abs(astsmc_status[f"integral_state[{axis}]"][airborne_mask])
            ),
            "k1": distribution(k1),
            "k2": distribution(k2),
            "reaching_raw": distribution(np.abs(reaching_raw)),
            "reaching_limited": distribution(np.abs(reaching_limited)),
            "reaching_clamp": distribution(reaching_clamp),
            "nominal_torque_raw": distribution(np.abs(nominal_raw)),
            "nominal_torque_limited": distribution(np.abs(nominal_limited)),
            "nominal_clamp": distribution(np.abs(nominal_raw - nominal_limited)),
            "residual_torque_raw": distribution(np.abs(residual_raw)),
            "residual_torque_limited": distribution(np.abs(residual_limited)),
            "residual_authority": distribution(residual_authority),
            "variation_weight": distribution(variation_weight),
            "variation_regularization": distribution(
                np.abs(variation_regularization)
            ),
            "variation_regularization_active_count": int(np.sum(variation_active)),
            "variation_regularization_active_fraction": fraction(variation_active),
            "variation_regularization_signed_mean": float(
                np.mean(variation_regularization)
            ),
            "state_recovery_active_count": int(np.sum(recovery_active)),
            "state_recovery_active_fraction": fraction(recovery_active),
            "state_recovery_count_delta": recovery_count_delta,
            "recovery_trigger_sliding": distribution(
                np.abs(recovery_trigger_sliding)
            ),
            "recovery_trigger_state": distribution(
                np.abs(recovery_trigger_state)
            ),
            "recovery_trigger_reaching": distribution(
                np.abs(recovery_trigger_reaching)
            ),
            "wrong_direction_escape_count": int(
                np.sum(wrong_direction_escape)
            ),
            "wrong_direction_escape_fraction": fraction(
                wrong_direction_escape
            ),
            "torque_raw": distribution(np.abs(torque_raw)),
            "torque_limited": distribution(np.abs(torque_limited)),
            "torque_components": {
                "nominal": torque_component_metrics(
                    airborne_timestamps,
                    nominal_limited,
                    rate_osc_cutoff,
                    nominal_saturated,
                ),
                "residual": torque_component_metrics(
                    airborne_timestamps,
                    residual_limited,
                    rate_osc_cutoff,
                    saturated,
                ),
                "total": torque_component_metrics(
                    airborne_timestamps,
                    torque_limited,
                    rate_osc_cutoff,
                    np.abs(torque_limited) >= 0.98 * torque_limits[axis],
                ),
            },
            "decomposition_error": distribution(decomposition_error),
            "residual_bound_violation_count": int(np.sum(residual_excess > 1e-6)),
            "residual_bound_max_excess": float(max(np.max(residual_excess), 0.0)),
            "torque_bound_violation_count": int(np.sum(torque_excess > 1e-6)),
            "torque_bound_max_excess": float(max(np.max(torque_excess), 0.0)),
            "residual_saturation_disagreement_count": int(np.sum(saturated != residual_clamp_active)),
            "nominal_saturation_disagreement_count": int(
                np.sum(nominal_saturated != nominal_clamp_active)
            ),
        }
        result["wrong_direction_escape_count"] += int(
            np.sum(wrong_direction_escape)
        )

    result["gain_max_error"] = max(gain_errors) if gain_errors else np.nan
    result["gains_match_parameters"] = bool(
        gain_errors and np.isfinite(result["gain_max_error"]) and result["gain_max_error"] <= 1e-5
    )

    if angular_velocity is not None:
        required_rate_fields = {"xyz[0]", "xyz[1]", "xyz[2]"}

        if required_rate_fields.issubset(angular_velocity):
            rate_timestamp_field = sample_timestamp_field(angular_velocity)
            result["rate_timestamp_source"] = rate_timestamp_field
            rate_indexes, ast_indexes = exact_timestamp_indexes(
                angular_velocity[rate_timestamp_field],
                astsmc_status["timestamp_sample"][airborne_indexes],
            )
            result["rate_timestamp_match_count"] = len(ast_indexes)
            result["rate_timestamp_match_fraction"] = (
                len(ast_indexes) / result["airborne_samples"]
            )

            if len(ast_indexes) > 0:
                matched_ast_indexes = airborne_indexes[ast_indexes]
                matched_rates = vector_data(
                    angular_velocity,
                    ("xyz[0]", "xyz[1]", "xyz[2]"),
                )[rate_indexes]
                raw_setpoints = vector_data(
                    astsmc_status,
                    (
                        "rate_setpoint_raw[0]",
                        "rate_setpoint_raw[1]",
                        "rate_setpoint_raw[2]",
                    ),
                )[matched_ast_indexes]
                shaped_setpoints = vector_data(
                    astsmc_status,
                    (
                        "rate_setpoint_shaped[0]",
                        "rate_setpoint_shaped[1]",
                        "rate_setpoint_shaped[2]",
                    ),
                )[matched_ast_indexes]
                conditioned_error = vector_data(
                    astsmc_status,
                    (
                        "sliding_variable[0]",
                        "sliding_variable[1]",
                        "sliding_variable[2]",
                    ),
                )[matched_ast_indexes]
                raw_error = raw_setpoints - matched_rates
                shaping_lag = raw_setpoints - shaped_setpoints
                tracking_recovery = conditioned_error - (shaped_setpoints - matched_rates)
                identity_error = raw_error - shaping_lag + tracking_recovery - conditioned_error
                result["rate_error_identity_max"] = float(
                    np.max(np.abs(identity_error))
                )

                for axis, axis_name in enumerate(AXES):
                    result["tracking"][axis_name] = {
                        "raw_error": stats(np.abs(raw_error[:, axis])),
                        "reference_shaping_lag": stats(
                            np.abs(shaping_lag[:, axis])
                        ),
                        "tracking_recovery": stats(
                            np.abs(tracking_recovery[:, axis])
                        ),
                        "conditioned_error": stats(
                            np.abs(conditioned_error[:, axis])
                        ),
                        "identity_error": distribution(
                            np.abs(identity_error[:, axis])
                        ),
                    }

    feedback_valid = astsmc_status["allocator_feedback_valid"][airborne_mask].astype(bool)
    feedback_stale = astsmc_status["allocator_feedback_stale"][airborne_mask].astype(bool)
    torque_achieved = astsmc_status["allocator_torque_setpoint_achieved"][airborne_mask].astype(bool)
    feedback_age = astsmc_status["allocator_feedback_age_s"][airborne_mask].astype(float)
    allocation_residual = vector_data(
        astsmc_status,
        ("allocation_residual[0]", "allocation_residual[1]", "allocation_residual[2]"),
    )[airborne_mask]
    allocator_sample_lag = (
        astsmc_status["timestamp_sample"].astype(float)[airborne_mask]
        - astsmc_status["allocator_timestamp_sample"].astype(float)[airborne_mask]
    ) / 1e6
    residual_norm = np.linalg.norm(allocation_residual, axis=1)
    result["allocator"] = {
        "feedback_valid_fraction": fraction(feedback_valid),
        "stale_count": int(np.sum(feedback_stale)),
        "stale_fraction": fraction(feedback_stale),
        "torque_achieved_fraction": fraction(torque_achieved),
        "feedback_age_s": distribution(feedback_age),
        "sample_lag_s": distribution(allocator_sample_lag),
        "future_timestamp_count": int(np.sum(allocator_sample_lag < 0.0)),
        "residual_norm": distribution(residual_norm),
        "residual_axes": {
            axis: distribution(np.abs(allocation_residual[:, index]))
            for index, axis in enumerate(AXES)
        },
    }

    torque_sample_timestamps = torque_setpoint["timestamp_sample"].astype(np.uint64)
    torque_by_sample = {
        int(timestamp): index for index, timestamp in enumerate(torque_sample_timestamps)
    }
    torque_values = vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]"))
    matched_ast_indexes = []
    matched_torque_indexes = []

    for ast_index in airborne_indexes:
        torque_index = torque_by_sample.get(int(astsmc_status["timestamp_sample"][ast_index]))

        if torque_index is not None:
            matched_ast_indexes.append(ast_index)
            matched_torque_indexes.append(torque_index)

    result["torque_timestamp_match_count"] = len(matched_ast_indexes)
    result["torque_timestamp_match_fraction"] = (
        len(matched_ast_indexes) / result["airborne_samples"]
    )

    if matched_ast_indexes:
        matched_ast_indexes = np.asarray(matched_ast_indexes, dtype=int)
        matched_torque_indexes = np.asarray(matched_torque_indexes, dtype=int)

        for axis, axis_name in enumerate(AXES):
            mismatch = (
                astsmc_status[f"torque_limited[{axis}]"][matched_ast_indexes].astype(float)
                - torque_values[matched_torque_indexes, axis]
            )
            result["command_mismatch"][axis_name] = distribution(np.abs(mismatch))

    result["runtime_valid"] = bool(
        result["identity_samples"] > 0
        and result["controller_type_fraction"] == 1.0
        and result["astsmc_valid_fraction"] == 1.0
        and result["configuration_valid_fraction"] == 1.0
    )
    return result


def logged_controller_parameters(ulog, at_timestamp_s=None):
    names = [
        "MC_RATE_CTRL_T", "MC_BAT_SCALE_EN", "SDLOG_PROFILE",
        "MC_MSMC_CFG", "MC_AST_CFG", "MIS_TKO_ALT_MAX",
        "MC_MPC_J_R", "MC_MPC_J_P", "MC_MPC_J_Y",
        "MC_MPC_EFF_R", "MC_MPC_EFF_P", "MC_MPC_EFF_Y",
        "MC_MPC_Q_R", "MC_MPC_Q_P", "MC_MPC_Q_Y",
        "MC_MPC_R_R", "MC_MPC_R_P", "MC_MPC_R_Y",
        "MC_MPC_DU_R", "MC_MPC_DU_P", "MC_MPC_DU_Y",
        "MC_MPC_TAU", "MC_MPC_HORIZON", "MC_MPC_SLEW", "MC_MPC_GYRO",
        "MC_MPC_TMAX_R", "MC_MPC_TMAX_P", "MC_MPC_TMAX_Y",
        "MC_AST_J_R", "MC_AST_J_P", "MC_AST_J_Y",
        "MC_AST_EFF_R", "MC_AST_EFF_P", "MC_AST_EFF_Y",
        "MC_AST_K1_R", "MC_AST_K1_P", "MC_AST_K1_Y",
        "MC_AST_K2_R", "MC_AST_K2_P", "MC_AST_K2_Y",
        "MC_AST_TMAX_R", "MC_AST_TMAX_P", "MC_AST_TMAX_Y",
        "MC_AST_RACC_R", "MC_AST_RACC_P", "MC_AST_RACC_Y",
        "MC_AST_RJERK_R", "MC_AST_RJERK_P", "MC_AST_RJERK_Y",
        "MC_AST_TRES_R", "MC_AST_TRES_P", "MC_AST_TRES_Y",
        "MC_AST_TAU_R", "MC_AST_TAU_P", "MC_AST_TAU_Y",
        "MC_AST_SLEW_R", "MC_AST_SLEW_P", "MC_AST_SLEW_Y",
        "MC_AST_DU_R", "MC_AST_DU_P", "MC_AST_DU_Y",
        "MC_AST_SBD_R", "MC_AST_SBD_P", "MC_AST_SBD_Y", "MC_AST_TRK_B",
        "MC_AST_RFF", "MC_AST_RFF_RP", "MC_AST_GYRO", "MC_AST_DT_MIN", "MC_AST_DT_MAX",
        "MC_AST_REC_ERR",
        "MC_MSMC_J_R", "MC_MSMC_J_P", "MC_MSMC_J_Y",
        "MC_MSMC_EFF_R", "MC_MSMC_EFF_P", "MC_MSMC_EFF_Y",
        "MC_MSMC_C_R", "MC_MSMC_C_P", "MC_MSMC_C_Y",
        "MC_MSMC_ETA_R", "MC_MSMC_ETA_P", "MC_MSMC_ETA_Y",
        "MC_MSMC_BND_R", "MC_MSMC_BND_P", "MC_MSMC_BND_Y",
        "MC_MSMC_KS_R", "MC_MSMC_KS_P", "MC_MSMC_KS_Y",
        "MC_MSMC_RSPD_L", "MC_SMC_LPF", "MC_SMC_SLEW",
        "MC_MSMC_ILIM_R", "MC_MSMC_ILIM_P", "MC_MSMC_ILIM_Y",
        "MC_MSMC_TMAX_R", "MC_MSMC_TMAX_P", "MC_MSMC_TMAX_Y",
    ]
    parameters = {name: ulog.initial_parameters.get(name) for name in names}

    for timestamp, name, value in ulog.changed_parameters:
        if name in parameters and (at_timestamp_s is None or timestamp / 1e6 <= at_timestamp_s):
            parameters[name] = value

    return parameters


def trajectory_setpoints(ulog):
    trajectory = dataset(ulog, "trajectory_setpoint")
    timestamps = trajectory["timestamp"].astype(float) / 1e6
    setpoints = np.column_stack(
        [
            trajectory["position[0]"],
            trajectory["position[1]"],
            trajectory["position[2]"],
        ]
    ).astype(float)

    valid = np.isfinite(setpoints).all(axis=1)
    return timestamps[valid], setpoints[valid]


def final_windows(takeoff_hold_s, move_s, hold_s, final_s, waypoints):
    windows = {"takeoff": (takeoff_hold_s - final_s, takeoff_hold_s)}
    segment_start = takeoff_hold_s

    for name, _target in waypoints[1:]:
        segment_end = segment_start + move_s + hold_s
        windows[name] = (segment_end - final_s, segment_end)
        segment_start = segment_end

    return windows


def analyze_log(path, args):
    ulog = ULog(str(path))
    waypoints = square_waypoints(args.side, args.altitude)

    local_position = dataset(ulog, "vehicle_local_position")
    vehicle_status = dataset(ulog, "vehicle_status")
    control_allocator = dataset(ulog, "control_allocator_status")
    actuator_motors = dataset(ulog, "actuator_motors")
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    rates_setpoint = dataset(ulog, "vehicle_rates_setpoint")
    torque_setpoint = dataset(ulog, "vehicle_torque_setpoint")
    land_detected = dataset(ulog, "vehicle_land_detected")
    astsmc_status = optional_dataset(ulog, "astsmc_status")
    astsmc_safety_status = optional_dataset(ulog, "astsmc_safety_status")

    offboard_start_s, offboard_end_s = first_offboard_window(vehicle_status)
    planned_square_end_s = offboard_start_s + args.takeoff_hold + (args.move + args.hold) * 4.0
    yaw_sequence_start_s = planned_square_end_s
    planned_maneuver_end_s = planned_square_end_s

    if args.yaw_step_deg > 0.0:
        planned_maneuver_end_s += 3.0 * args.yaw_hold

    position_timestamps = local_position["timestamp"].astype(float) / 1e6
    positions = vector_data(local_position, ("x", "y", "z"))

    setpoint_timestamps, setpoints = trajectory_setpoints(ulog)
    sampled_setpoints, valid_setpoints = zero_order_hold(position_timestamps, setpoint_timestamps, setpoints)
    trajectory_error = np.linalg.norm(positions - sampled_setpoints, axis=1)

    tracking_mask = (
        (position_timestamps >= offboard_start_s + args.takeoff_hold)
        & (position_timestamps <= min(offboard_end_s, planned_square_end_s))
        & valid_setpoints
    )
    all_offboard_mask = (
        (position_timestamps >= offboard_start_s)
        & (position_timestamps <= min(offboard_end_s, planned_square_end_s))
        & valid_setpoints
    )

    rate_timestamps = sample_timestamps(angular_velocity)
    rates = vector_data(angular_velocity, ("xyz[0]", "xyz[1]", "xyz[2]"))
    rate_sp_timestamps = sample_timestamps(rates_setpoint)
    rate_setpoints = vector_data(rates_setpoint, ("roll", "pitch", "yaw"))
    sampled_rate_setpoints, valid_rate_setpoints = zero_order_hold(
        rate_timestamps, rate_sp_timestamps, rate_setpoints
    )
    rate_mask = (
        (rate_timestamps >= offboard_start_s)
        & (rate_timestamps <= min(offboard_end_s, planned_square_end_s))
        & valid_rate_setpoints
        & np.isfinite(rates).all(axis=1)
    )
    rate_errors = sampled_rate_setpoints - rates
    rate_error_stats = {
        axis: stats(np.abs(rate_errors[rate_mask, index]))
        for index, axis in enumerate(AXES)
    }
    rate_stats = {
        axis: stats(np.abs(rates[rate_mask, index]))
        for index, axis in enumerate(AXES)
    }
    rate_oscillation = {
        axis: high_frequency_metrics(
            rate_timestamps[rate_mask], rates[rate_mask, index], args.rate_osc_cutoff
        )
        for index, axis in enumerate(AXES)
    }

    torque_timestamps = sample_timestamps(torque_setpoint)
    torque_mask = (
        (torque_timestamps >= offboard_start_s)
        & (torque_timestamps <= min(offboard_end_s, planned_maneuver_end_s))
    )
    torque_window_timestamps = torque_timestamps[torque_mask]
    torques = vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]"))[torque_mask]
    parameters = logged_controller_parameters(ulog, offboard_start_s)
    controller_parameter_changes = [
        (timestamp / 1e6, name, value)
        for timestamp, name, value in ulog.changed_parameters
        if name in parameters and offboard_start_s < timestamp / 1e6 <= offboard_end_s
    ]
    controller_type = int(parameters["MC_RATE_CTRL_T"])
    limit_prefix = torque_limit_prefix(controller_type)
    torque_limits = np.array(
        [parameters.get(f"{limit_prefix}_{suffix}") for suffix in ("R", "P", "Y")],
        dtype=float,
    )
    torque_limits[~np.isfinite(torque_limits)] = 1.0
    torque_limits = np.clip(torque_limits, 1e-6, 1.0)
    torque_stats = {
        axis: stats(np.abs(torques[:, index]))
        for index, axis in enumerate(AXES)
    }
    torque_dynamics = {
        axis: torque_variation_metrics(
            torque_window_timestamps, torques[:, index], args.rate_osc_cutoff
        )
        for index, axis in enumerate(AXES)
    }
    torque_limit_fraction = {
        axis: float(np.mean(np.abs(torques[:, index]) >= args.torque_limit_ratio * torque_limits[index]))
        for index, axis in enumerate(AXES)
    }

    yaw_steps = None

    if args.yaw_step_deg > 0.0:
        yaw_steps = yaw_step_metrics(
            ulog,
            yaw_sequence_start_s,
            args.yaw_hold,
            np.radians(args.yaw_step_deg),
            args.yaw_final_s,
            np.radians(args.yaw_settle_threshold_deg),
            args.yaw_settle_dwell,
        )
        yaw_position_mask = (
            (position_timestamps >= yaw_steps["start_s"])
            & (position_timestamps <= yaw_steps["end_s"])
            & valid_setpoints
        )
        yaw_steps["position_error"] = stats(trajectory_error[yaw_position_mask])

    rate_control_status = optional_dataset(ulog, "rate_ctrl_status")
    smc_status_valid = controller_type != 2
    smc_status_samples = 0
    smc_diagnostics = None

    if controller_type == 2 and rate_control_status is not None:
        required_status_fields = {
            "controller_type", "model_valid", "smc_surface[0]", "smc_surface_filtered[0]",
            "smc_torque_raw[0]", "smc_torque_limited[0]",
        }

        if required_status_fields.issubset(rate_control_status):
            status_timestamps = rate_control_status["timestamp"].astype(float) / 1e6
            status_mask = (
                (status_timestamps >= offboard_start_s)
                & (status_timestamps <= min(offboard_end_s, planned_maneuver_end_s))
            )
            smc_status_samples = int(np.sum(status_mask))
            smc_status_valid = smc_status_samples > 0 and bool(
                np.all(rate_control_status["controller_type"][status_mask] == 2)
                and np.all(rate_control_status["model_valid"][status_mask])
            )

            if smc_status_samples > 0:
                smc_diagnostics = {
                    "surface_max": [
                        float(np.max(np.abs(rate_control_status[f"smc_surface[{axis}]"][status_mask])))
                        for axis in range(3)
                    ],
                    "raw_torque_max": [
                        float(np.max(np.abs(rate_control_status[f"smc_torque_raw[{axis}]"][status_mask])))
                        for axis in range(3)
                    ],
                    "limited_torque_max": [
                        float(np.max(np.abs(rate_control_status[f"smc_torque_limited[{axis}]"][status_mask])))
                        for axis in range(3)
                    ],
                    "limit_fraction": [
                        float(
                            np.mean(
                                np.abs(rate_control_status[f"smc_torque_limited[{axis}]"][status_mask])
                                >= args.torque_limit_ratio * torque_limits[axis]
                            )
                        )
                        for axis in range(3)
                    ],
                }

    astsmc_diagnostics = None

    if controller_type == 3:
        astsmc_diagnostics = analyze_astsmc_diagnostics(
            astsmc_status,
            astsmc_safety_status,
            rate_control_status,
            land_detected,
            torque_setpoint,
            parameters,
            offboard_start_s,
            offboard_end_s,
            planned_maneuver_end_s,
            torque_limits,
            yaw_steps["transition_times"] if yaw_steps is not None else None,
            angular_velocity=angular_velocity,
            rate_osc_cutoff=args.rate_osc_cutoff,
        )

    windows = final_windows(args.takeoff_hold, args.move, args.hold, args.final_s, waypoints)
    final_errors = []
    square_final_errors = []
    takeoff_final_errors = []
    waypoint_results = []

    for name, _target in waypoints:
        window_start, window_end = windows[name]
        waypoint_mask = (
            (position_timestamps >= offboard_start_s + window_start)
            & (position_timestamps <= offboard_start_s + window_end)
            & valid_setpoints
        )
        errors = trajectory_error[waypoint_mask]
        final_errors.extend(errors.tolist())

        if name == "takeoff":
            takeoff_final_errors.extend(errors.tolist())

        else:
            square_final_errors.extend(errors.tolist())

        waypoint_results.append((name, stats(errors)))

    allocator_timestamps = control_allocator["timestamp"].astype(float) / 1e6
    allocator_mask = (allocator_timestamps >= offboard_start_s) & (allocator_timestamps <= offboard_end_s)
    torque_achieved = bool(np.all(control_allocator["torque_setpoint_achieved"][allocator_mask]))
    thrust_achieved = bool(np.all(control_allocator["thrust_setpoint_achieved"][allocator_mask]))

    unallocated_torque = vector_data(
        control_allocator, ("unallocated_torque[0]", "unallocated_torque[1]", "unallocated_torque[2]")
    )
    unallocated_thrust = vector_data(
        control_allocator, ("unallocated_thrust[0]", "unallocated_thrust[1]", "unallocated_thrust[2]")
    )

    motor_timestamps = actuator_motors["timestamp"].astype(float) / 1e6
    motor_mask = (motor_timestamps >= offboard_start_s) & (motor_timestamps <= offboard_end_s)
    motors = np.column_stack([actuator_motors[f"control[{index}]"] for index in range(4)]).astype(float)
    motors = motors[motor_mask]
    motors = motors[np.isfinite(motors)]

    return {
        "path": path,
        "offboard_duration_s": offboard_end_s - offboard_start_s,
        "track_all": stats(trajectory_error[all_offboard_mask]),
        "track_square": stats(trajectory_error[tracking_mask]),
        "final": stats(final_errors),
        "final_square": stats(square_final_errors),
        "final_takeoff": stats(takeoff_final_errors),
        "waypoints": waypoint_results,
        "rate_error": rate_error_stats,
        "rate": rate_stats,
        "rate_oscillation": rate_oscillation,
        "rate_timestamp_source": sample_timestamp_field(angular_velocity),
        "controller_type": controller_type,
        "torque": torque_stats,
        "torque_dynamics": torque_dynamics,
        "torque_limits": dict(zip(AXES, torque_limits.tolist())),
        "torque_limit_fraction": torque_limit_fraction,
        "yaw_steps": yaw_steps,
        "smc_status_valid": smc_status_valid,
        "smc_status_samples": smc_status_samples,
        "smc_diagnostics": smc_diagnostics,
        "astsmc_diagnostics": astsmc_diagnostics,
        "parameters": parameters,
        "controller_parameter_changes": controller_parameter_changes,
        "torque_all": torque_achieved,
        "thrust_all": thrust_achieved,
        "failsafe": bool(np.any(vehicle_status["failsafe"])),
        "unallocated_torque_max": float(np.max(np.linalg.norm(unallocated_torque[allocator_mask], axis=1))),
        "unallocated_thrust_max": float(np.max(np.linalg.norm(unallocated_thrust[allocator_mask], axis=1))),
        "motor_min": float(np.min(motors)),
        "motor_max": float(np.max(motors)),
        "motor_near_low": int(np.sum(motors < args.motor_low)),
        "motor_near_high": int(np.sum(motors > args.motor_high)),
    }


def print_result(result, final_s, compact):
    if compact:
        yaw_summary = ""

        if result["yaw_steps"] is not None:
            yaw_summary = (
                f",yaw_excursion_deg={np.degrees(result['yaw_steps']['commanded_excursion']):.2f}"
                f",yaw_error_rms_deg={np.degrees(result['yaw_steps']['error']['rms']):.2f}"
                f",yaw_rate_max_dps={np.degrees(result['yaw_steps']['max_rate']):.2f}"
                f",yaw_position_max={result['yaw_steps']['position_error']['max']:.3f}"
            )

        print(
            f"{result['path'].name},"
            f"track_rms={result['track_square']['rms']:.4f},"
            f"track_p95={result['track_square']['p95']:.4f},"
            f"track_max={result['track_square']['max']:.4f},"
            f"final_mean={result['final']['mean']:.4f},"
            f"final_p95={result['final']['p95']:.4f},"
            f"final_max={result['final']['max']:.4f},"
            f"square_final_mean={result['final_square']['mean']:.4f},"
            f"square_final_max={result['final_square']['max']:.4f},"
            f"rate_err_rms={max(result['rate_error'][axis]['rms'] for axis in AXES):.4f},"
            f"rate_osc_rms={max(result['rate_oscillation'][axis]['rms'] for axis in AXES):.4f},"
            f"torque_hf_rms={max(result['torque_dynamics'][axis]['high_frequency']['rms'] for axis in AXES):.4f},"
            f"torque_tv_rate={max(result['torque_dynamics'][axis]['variation_per_s'] for axis in AXES):.4f},"
            f"torque_limit_frac={max(result['torque_limit_fraction'].values()):.4f},"
            f"torque_all={result['torque_all']},"
            f"parameter_changes={len(result['controller_parameter_changes'])},"
            f"failsafe={result['failsafe']},"
            f"motor_max={result['motor_max']:.3f},"
            f"smc_status_valid={result['smc_status_valid']},"
            f"astsmc_runtime_valid={result['astsmc_diagnostics']['runtime_valid'] if result['astsmc_diagnostics'] else None}"
            f"{yaw_summary}"
        )
        return

    print(f"\n== {result['path'].name} offboard={result['offboard_duration_s']:.3f}s")
    print(f"track_all_vs_trajectory_sp    {format_stats(result['track_all'])}")
    print(f"track_square_vs_trajectory_sp {format_stats(result['track_square'])}")
    print(f"final{final_s:g}_vs_logged_setpoint  {format_stats(result['final'])}")
    print(f"square_final{final_s:g}_vs_logged_setpoint {format_stats(result['final_square'])}")
    print(f"takeoff_final{final_s:g}                 {format_stats(result['final_takeoff'])}")
    print(
        "health "
        f"torque_all={result['torque_all']} thrust_all={result['thrust_all']} "
        f"failsafe={result['failsafe']} parameter_changes={len(result['controller_parameter_changes'])} "
        f"unalloc_torque_max={result['unallocated_torque_max']:.5f} "
        f"unalloc_thrust_max={result['unallocated_thrust_max']:.5f}"
    )
    print(
        "motors "
        f"min={result['motor_min']:.3f} max={result['motor_max']:.3f} "
        f"near_low={result['motor_near_low']} near_high={result['motor_near_high']}"
    )
    print("inner_loop:")

    for axis in AXES:
        oscillation = result["rate_oscillation"][axis]
        print(
            f"  {axis:5s} rate_error {format_stats(result['rate_error'][axis])} "
            f"rate_rms={result['rate'][axis]['rms']:.4f} "
            f"osc_rms={oscillation['rms']:.4f} "
            f"peak={oscillation['peak_hz']:.2f}Hz/{oscillation['peak_amplitude']:.4f}rad/s "
            f"torque_rms={result['torque'][axis]['rms']:.4f} "
            f"torque_hf_rms={result['torque_dynamics'][axis]['high_frequency']['rms']:.4f} "
            f"torque_tv_rate={result['torque_dynamics'][axis]['variation_per_s']:.4f}/s "
            f"limit={result['torque_limits'][axis]:.4f} "
            f"at_limit={100.0 * result['torque_limit_fraction'][axis]:.2f}%"
        )

    if result["yaw_steps"] is not None:
        yaw_steps = result["yaw_steps"]
        print(
            "yaw_steps "
            f"excursion={np.degrees(yaw_steps['commanded_excursion']):.2f}deg "
            f"error_rms={np.degrees(yaw_steps['error']['rms']):.2f}deg "
            f"max_rate={np.degrees(yaw_steps['max_rate']):.2f}deg/s "
            f"position_error={format_stats(yaw_steps['position_error'])}"
        )

        for index, segment in enumerate(yaw_steps["segments"]):
            print(
                f"  segment{index + 1} target={np.degrees(segment['target']):.2f}deg "
                f"final_error_max={np.degrees(segment['final_error']['max']):.2f}deg "
                f"settling={segment['settling_s']:.3f}s"
            )

    print(
        f"smc_status valid={result['smc_status_valid']} samples={result['smc_status_samples']} "
        f"diagnostics={result['smc_diagnostics']}"
    )

    if result["astsmc_diagnostics"] is not None:
        ast = result["astsmc_diagnostics"]
        print(
            "astsmc_status "
            f"runtime_valid={ast['runtime_valid']} samples={ast['airborne_samples']} "
            f"rate={ast['sample_rate_hz']:.2f}Hz dt_invalid={ast['dt_invalid_count']} "
            f"nonfinite_rows={ast['nonfinite_rows']} missing={ast['missing_fields']} "
            f"feedback_valid={ast['allocator'].get('feedback_valid_fraction', np.nan):.4f} "
            f"feedback_stale={ast['allocator'].get('stale_fraction', np.nan):.4f} "
            f"residual_max={ast['allocator'].get('residual_norm', {}).get('max', np.nan):.6f} "
            f"torque_match={ast['torque_timestamp_match_fraction']:.4f}"
        )

        print(
            "astsmc_rate_alignment "
            f"source={ast['rate_timestamp_source']} "
            f"matched={ast['rate_timestamp_match_count']}/{ast['airborne_samples']} "
            f"identity_max={ast['rate_error_identity_max']:.9f}"
        )

        for axis in AXES:
            diagnostics = ast["axes"].get(axis)

            if diagnostics is not None:
                components = diagnostics["torque_components"]
                tracking = ast["tracking"].get(axis, {})
                print(
                    f"  {axis:5s} internal_sat={100.0 * diagnostics['internal_saturation_fraction']:.2f}% "
                    f"variation_active={100.0 * diagnostics['variation_regularization_active_fraction']:.2f}% "
                    f"variation_rms={diagnostics['variation_regularization']['rms']:.6f} "
                    f"variation_max={diagnostics['variation_regularization']['max']:.6f} "
                    f"slide_rms={diagnostics['sliding_variable']['rms']:.5f} "
                    f"raw_err_rms={tracking.get('raw_error', {}).get('rms', np.nan):.5f} "
                    f"shape_lag_rms={tracking.get('reference_shaping_lag', {}).get('rms', np.nan):.5f} "
                    f"conditioned_err_rms={tracking.get('conditioned_error', {}).get('rms', np.nan):.5f} "
                    f"nominal_rms={components['nominal']['rms']:.5f} "
                    f"residual_rms={components['residual']['rms']:.5f} "
                    f"total_rms={components['total']['rms']:.5f} "
                    f"nominal_tv_rate={components['nominal']['variation_per_s']:.5f}/s "
                    f"residual_tv_rate={components['residual']['variation_per_s']:.5f}/s "
                    f"total_tv_rate={components['total']['variation_per_s']:.5f}/s "
                    f"decomposition_error={diagnostics['decomposition_error']['max']:.6f} "
                    f"command_mismatch={ast['command_mismatch'].get(axis, {}).get('max', np.nan):.6f}"
                )

    print("controller_parameters:")

    for name, value in result["parameters"].items():
        if value is not None:
            print(f"  {name}={value}")

    if result["controller_parameter_changes"]:
        print("controller_parameter_changes:")

        for timestamp, name, value in result["controller_parameter_changes"]:
            print(f"  t={timestamp:.6f}s {name}={value}")

    print("waypoint_final:")

    for name, waypoint_stats in result["waypoints"]:
        print(f"  {name:7s} {format_stats(waypoint_stats)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--takeoff-hold", type=float, default=8.0)
    parser.add_argument("--move", type=float, default=3.2)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--side", type=float, default=0.6)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--yaw-step-deg", type=float, default=0.0)
    parser.add_argument("--yaw-hold", type=float, default=3.0)
    parser.add_argument("--yaw-final-s", type=float, default=0.5)
    parser.add_argument("--yaw-settle-threshold-deg", type=float, default=5.0)
    parser.add_argument("--yaw-settle-dwell", type=float, default=0.25)
    parser.add_argument("--final-s", type=float, default=1.0)
    parser.add_argument("--motor-low", type=float, default=0.05)
    parser.add_argument("--motor-high", type=float, default=0.95)
    parser.add_argument("--rate-osc-cutoff", type=float, default=4.0)
    parser.add_argument("--torque-limit-ratio", type=float, default=0.98)
    parser.add_argument("--compact", action="store_true")
    parsed_args = parser.parse_args()

    for log_path in parsed_args.logs:
        print_result(analyze_log(log_path, parsed_args), parsed_args.final_s, parsed_args.compact)


if __name__ == "__main__":
    main()
