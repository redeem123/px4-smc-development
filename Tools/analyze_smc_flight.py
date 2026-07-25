#!/usr/bin/env python3
"""Check SMC rate-loop health in real or simulated PX4 ULogs."""

import argparse
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


ARMING_STATE_ARMED = 2
AXES = ("roll", "pitch", "yaw")


def dataset(ulog, name):
    return next(data.data for data in ulog.data_list if data.name == name and data.multi_id == 0)


def optional_dataset(ulog, name):
    return next(
        (data.data for data in ulog.data_list if data.name == name and data.multi_id == 0),
        None,
    )


def vector_data(data, fields):
    return np.column_stack([data[field] for field in fields]).astype(float)


def zero_order_hold(sample_timestamps, source_timestamps, values, fill_value=np.nan):
    indexes = np.searchsorted(source_timestamps, sample_timestamps, side="right") - 1
    valid = indexes >= 0
    sampled = np.full((len(sample_timestamps),) + values.shape[1:], fill_value, dtype=float)
    sampled[valid] = values[indexes[valid]]
    return sampled


def reset_safe_counter_delta(values, indexes):
    values = np.asarray(values, dtype=np.uint64)
    indexes = np.asarray(indexes, dtype=int)

    if len(indexes) == 0:
        return 0

    first = indexes[0]
    previous = values[first - 1] if first > 0 else values[first]
    delta = 0

    for current in values[indexes]:
        delta += int(current - previous) if current >= previous else int(current)
        previous = current

    return delta


def exclude_astsmc_ground_containment(sample_timestamps, mask, astsmc_safety_status):
    if astsmc_safety_status is None or not {
        "timestamp_sample",
        "ground_containment_active",
    }.issubset(astsmc_safety_status):
        return mask

    containment = zero_order_hold(
        sample_timestamps,
        astsmc_safety_status["timestamp_sample"].astype(float) / 1e6,
        astsmc_safety_status["ground_containment_active"].astype(float),
        0.0,
    ) > 0.5
    return mask & ~containment


def quaternion_yaw(data, prefix):
    quaternion = vector_data(data, tuple(f"{prefix}[{index}]" for index in range(4)))
    w, x, y, z = quaternion.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrapped_angle(values):
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def yaw_attitude_summary(attitude, attitude_setpoint, start_s, end_s):
    if attitude is None or attitude_setpoint is None:
        return {"available": False}

    attitude_fields = {"timestamp", "q[0]", "q[1]", "q[2]", "q[3]"}
    setpoint_fields = {"timestamp", "q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"}

    if not attitude_fields.issubset(attitude) or not setpoint_fields.issubset(attitude_setpoint):
        return {"available": False}

    timestamps = attitude["timestamp"].astype(float) / 1e6
    setpoint_timestamps = attitude_setpoint["timestamp"].astype(float) / 1e6
    yaw = quaternion_yaw(attitude, "q")
    yaw_setpoint = quaternion_yaw(attitude_setpoint, "q_d")
    sampled_setpoint = zero_order_hold(timestamps, setpoint_timestamps, yaw_setpoint)
    window = (
        (timestamps >= start_s)
        & (timestamps <= end_s)
        & np.isfinite(yaw)
        & np.isfinite(sampled_setpoint)
    )

    if not np.any(window):
        return {"available": False}

    error = wrapped_angle(sampled_setpoint[window] - yaw[window])
    window_timestamps = timestamps[window]
    final_window = window_timestamps >= max(window_timestamps[0], window_timestamps[-1] - 1.0)
    return {
        "available": True,
        "error_rms_deg": float(np.degrees(np.sqrt(np.mean(error * error)))),
        "error_abs_max_deg": float(np.degrees(np.max(np.abs(error)))),
        "final_error_abs_deg": float(np.degrees(np.median(np.abs(error[final_window])))),
    }


def true_segments(mask, timestamps, minimum_duration_s):
    indexes = np.flatnonzero(mask)

    if len(indexes) == 0:
        return []

    breaks = np.flatnonzero(np.diff(indexes) > 1)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(indexes) - 1]
    segments = []

    for start, end in zip(starts, ends):
        first = int(indexes[start])
        last = int(indexes[end])
        duration = float(timestamps[last] - timestamps[first])

        if duration >= minimum_duration_s:
            segments.append((first, last))

    return segments


def high_frequency_metrics(timestamps, values, cutoff_hz):
    timestamps = np.asarray(timestamps, dtype=float)
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(timestamps) & np.isfinite(values)
    timestamps = timestamps[valid]
    values = values[valid]

    if len(values) < 8 or cutoff_hz <= 0.0:
        return {"rms": math.nan, "peak_hz": math.nan, "peak_amplitude": math.nan}

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
            "peak_hz": math.nan,
            "peak_amplitude": math.nan,
        }

    uniform_timestamps = np.arange(timestamps[0], timestamps[-1], median_dt)

    if len(uniform_timestamps) < 8:
        return {"rms": math.nan, "peak_hz": math.nan, "peak_amplitude": math.nan}

    uniform_residual = np.interp(uniform_timestamps, timestamps, residual)
    uniform_residual -= np.mean(uniform_residual)
    window = np.hanning(len(uniform_residual))
    spectrum = np.fft.rfft(uniform_residual * window)
    frequencies = np.fft.rfftfreq(len(uniform_residual), median_dt)
    frequency_mask = frequencies >= cutoff_hz

    if not np.any(frequency_mask):
        peak_hz = math.nan
        peak_amplitude = math.nan

    else:
        candidates = np.flatnonzero(frequency_mask)
        peak_index = candidates[np.argmax(np.abs(spectrum[frequency_mask]))]
        peak_hz = float(frequencies[peak_index])
        peak_amplitude = float(2.0 * np.abs(spectrum[peak_index]) / max(np.sum(window), 1.0))

    return {
        "rms": float(np.sqrt(np.mean(residual * residual))),
        "peak_hz": peak_hz,
        "peak_amplitude": peak_amplitude,
    }


def astsmc_safety_summary(
    astsmc_status,
    astsmc_safety_status,
    start_s,
    end_s,
    roll_pitch_residual_card=None,
):
    summary = {
        "present": astsmc_status is not None,
        "schema_complete": False,
        "runtime_fault_latched": None,
        "runtime_fault_reason": None,
        "state_recovery_count_delta": None,
        "selective_release_count_delta": None,
        "quiet_anchor_count_delta": None,
        "quiet_anchor_active_fraction": None,
        "deep_quiet_active_fraction": None,
        "trim_valid": None,
        "trim_confidence_max": None,
        "ground_containment_count_delta": None,
        "valid_update_count_delta": None,
        "nominal_limit_count_delta": None,
        "residual_limit_count_delta": None,
        "residual_limit_occupancy": None,
        "roll_pitch_residual_authority_base": None,
        "roll_pitch_residual_authority_max": None,
        "roll_pitch_residual_authority_extension_fraction": None,
        "roll_pitch_residual_authority_full_fraction": None,
        "wrong_direction_escape_count": 0,
    }

    if astsmc_status is None:
        return summary

    core_fields = {"timestamp_sample"}

    for axis in range(3):
        core_fields.update(
            {
                f"sliding_variable[{axis}]",
                f"reaching_input_limited[{axis}]",
            }
        )

    if not core_fields.issubset(astsmc_status):
        return summary

    safety_fields = {
        "runtime_fault_latched",
        "runtime_fault_reason",
        "ground_containment_count",
    }

    for axis in range(3):
        safety_fields.update(
            {
                f"state_recovery_count[{axis}]",
                f"state_recovery_active[{axis}]",
            }
        )

    legacy_safety_schema = safety_fields.issubset(astsmc_status)
    separate_safety_schema = (
        astsmc_safety_status is not None
        and safety_fields.issubset(astsmc_safety_status)
        and "timestamp_sample" in astsmc_safety_status
    )
    summary["schema_complete"] = legacy_safety_schema or separate_safety_schema
    timestamps = astsmc_status["timestamp_sample"].astype(float) / 1e6
    window = (timestamps >= start_s) & (timestamps <= end_s)

    if not np.any(window):
        return summary

    indexes = np.flatnonzero(window)
    safety_status = astsmc_status if legacy_safety_schema else astsmc_safety_status
    safety_timestamps = (
        safety_status["timestamp_sample"].astype(float) / 1e6
        if summary["schema_complete"]
        else np.array([])
    )
    safety_window = (safety_timestamps >= start_s) & (safety_timestamps <= end_s)
    safety_indexes = np.flatnonzero(safety_window)

    def counter_delta(field):
        return reset_safe_counter_delta(safety_status[field], safety_indexes)

    if summary["schema_complete"] and len(safety_indexes) > 0:
        summary["runtime_fault_latched"] = bool(
            np.any(safety_status["runtime_fault_latched"][safety_window])
        )
        summary["runtime_fault_reason"] = int(
            np.bitwise_or.reduce(
                safety_status["runtime_fault_reason"][safety_window].astype(np.uint32),
                initial=np.uint32(0),
            )
        )
        summary["ground_containment_count_delta"] = counter_delta(
            "ground_containment_count"
        )
        summary["state_recovery_count_delta"] = 0

        selective_fields = {
            "selective_release_count[0]",
            "selective_release_count[1]",
            "trim_valid[0]",
            "trim_valid[1]",
        }

        if selective_fields.issubset(safety_status):
            summary["selective_release_count_delta"] = [
                counter_delta(f"selective_release_count[{axis}]") for axis in range(2)
            ]
            summary["trim_valid"] = [
                bool(np.any(safety_status[f"trim_valid[{axis}]"][safety_window]))
                for axis in range(2)
            ]

            quiet_anchor_fields = {
                "quiet_anchor_active[0]",
                "quiet_anchor_active[1]",
                "quiet_anchor_count[0]",
                "quiet_anchor_count[1]",
            }

            if quiet_anchor_fields.issubset(safety_status):
                summary["quiet_anchor_count_delta"] = [
                    counter_delta(f"quiet_anchor_count[{axis}]") for axis in range(2)
                ]
                summary["quiet_anchor_active_fraction"] = [
                    float(
                        np.mean(
                            safety_status[f"quiet_anchor_active[{axis}]"][safety_window]
                        )
                    )
                    for axis in range(2)
                ]

            if all(f"deep_quiet_active[{axis}]" in safety_status for axis in range(2)):
                summary["deep_quiet_active_fraction"] = [
                    float(
                        np.mean(
                            safety_status[f"deep_quiet_active[{axis}]"][safety_window]
                        )
                    )
                    for axis in range(2)
                ]

            if all(f"trim_confidence[{axis}]" in safety_status for axis in range(2)):
                summary["trim_confidence_max"] = [
                    float(
                        np.max(
                            safety_status[f"trim_confidence[{axis}]"][safety_window],
                            initial=0.0,
                        )
                    )
                    for axis in range(2)
                ]

    counter_fields = {"valid_update_count"}
    for axis in range(3):
        counter_fields.update(
            {
                f"nominal_limit_count[{axis}]",
                f"residual_limit_count[{axis}]",
            }
        )

    if counter_fields.issubset(astsmc_status):
        summary["valid_update_count_delta"] = reset_safe_counter_delta(
            astsmc_status["valid_update_count"], indexes
        )
        summary["nominal_limit_count_delta"] = []
        summary["residual_limit_count_delta"] = []

        for axis in range(3):
            summary["nominal_limit_count_delta"].append(
                reset_safe_counter_delta(
                    astsmc_status[f"nominal_limit_count[{axis}]"], indexes
                )
            )
            summary["residual_limit_count_delta"].append(
                reset_safe_counter_delta(
                    astsmc_status[f"residual_limit_count[{axis}]"], indexes
                )
            )

        valid_updates = summary["valid_update_count_delta"]
        summary["residual_limit_occupancy"] = [
            count / valid_updates if valid_updates > 0 else math.nan
            for count in summary["residual_limit_count_delta"]
        ]

    authority_fields = {"residual_authority[0]", "residual_authority[1]"}

    if authority_fields.issubset(astsmc_status):
        base_authority = []
        maximum_authority = []
        extension_fraction = []
        full_fraction = []

        for axis in range(2):
            authority = astsmc_status[f"residual_authority[{axis}]"][window].astype(float)
            authority = authority[np.isfinite(authority)]

            if len(authority) == 0:
                base_authority.append(math.nan)
                maximum_authority.append(math.nan)
                extension_fraction.append(math.nan)
                full_fraction.append(math.nan)
                continue

            if roll_pitch_residual_card is not None:
                base, maximum = roll_pitch_residual_card[axis]
                base = float(base)
                maximum = float(maximum)
            else:
                base = float(np.min(authority))
                maximum = float(np.max(authority))

            span = maximum - base
            base_authority.append(base)
            maximum_authority.append(maximum)
            extension_fraction.append(
                float(np.mean(np.maximum(authority - base, 0.0) / span))
                if span > 1e-7
                else 0.0
            )
            full_fraction.append(
                float(np.mean(authority >= maximum - max(1e-6, 0.01 * max(maximum, 1e-6))))
                if span > 1e-7
                else 0.0
            )

        summary["roll_pitch_residual_authority_base"] = base_authority
        summary["roll_pitch_residual_authority_max"] = maximum_authority
        summary["roll_pitch_residual_authority_extension_fraction"] = extension_fraction
        summary["roll_pitch_residual_authority_full_fraction"] = full_fraction

    for axis in range(3):
        if summary["schema_complete"] and len(safety_indexes) > 0:
            summary["state_recovery_count_delta"] += counter_delta(
                f"state_recovery_count[{axis}]"
            )
            recovery_times = safety_timestamps[safety_status[f"state_recovery_active[{axis}]"].astype(bool)]
            recovery_active = np.zeros(np.sum(window), dtype=bool)

            if len(recovery_times) > 0:
                recovery_active = np.any(
                    np.abs(timestamps[window, None] - recovery_times[None, :]) <= 0.01,
                    axis=1,
                )

        else:
            recovery_active = np.zeros(np.sum(window), dtype=bool)

        sliding = astsmc_status[f"sliding_variable[{axis}]"][window].astype(float)
        reaching = astsmc_status[f"reaching_input_limited[{axis}]"][window].astype(float)
        summary["wrong_direction_escape_count"] += int(
            np.sum((np.abs(sliding) > 1.0) & (sliding * reaching < -1e-7) & ~recovery_active)
        )

    return summary


def astsmc_allocator_summary(astsmc_allocator_status, start_s, end_s):
    summary = {
        "present": astsmc_allocator_status is not None,
        "schema_complete": False,
        "usable_feedback_fraction": None,
        "publication_age_max_s": None,
        "sample_age_max_s": None,
        "effective_authority_fraction": None,
        "effective_authority_max": None,
        "release_count_delta": None,
        "backoff_count_delta": None,
        "fallback_count_delta": None,
        "residual_at_backoff_max": None,
        "headroom_schema_complete": False,
        "headroom_request_fraction": None,
        "headroom_honor_fraction": None,
        "headroom_request_count_delta": None,
        "roll_pitch_miss_count_delta": None,
        "roll_pitch_achieved_fraction_mean": None,
        "roll_pitch_achieved_fraction_min": None,
        "roll_pitch_allocation_loss_longest_s": None,
        "roll_conditioning_fraction": None,
        "pitch_conditioning_fraction": None,
    }

    required_fields = {
        "timestamp_sample",
        "publication_age_s",
        "sample_age_s",
        "allocation_residual_norm",
        "yaw_residual_authority[0]",
        "yaw_residual_authority[1]",
        "yaw_residual_authority[2]",
        "event_count[2]",
        "event_count[3]",
        "state_flags",
    }

    if astsmc_allocator_status is None or not required_fields.issubset(astsmc_allocator_status):
        return summary

    timestamps = astsmc_allocator_status["timestamp_sample"].astype(float) / 1e6
    window = (timestamps >= start_s) & (timestamps <= end_s)
    indexes = np.flatnonzero(window)

    if len(indexes) == 0:
        return summary

    state_flags = astsmc_allocator_status["state_flags"][window].astype(np.uint16)
    publication_age = astsmc_allocator_status["publication_age_s"][window].astype(float)
    sample_age = astsmc_allocator_status["sample_age_s"][window].astype(float)
    residual_norm = astsmc_allocator_status["allocation_residual_norm"][window].astype(float)
    base_authority = astsmc_allocator_status["yaw_residual_authority[0]"][window].astype(float)
    maximum_authority = astsmc_allocator_status["yaw_residual_authority[1]"][window].astype(float)
    effective_authority = astsmc_allocator_status["yaw_residual_authority[2]"][window].astype(float)
    usable = (state_flags & 2) != 0
    release_active = (state_flags & 64) != 0
    backoff_active = (state_flags & 32) != 0
    valid_publication_age = publication_age[np.isfinite(publication_age)]
    valid_sample_age = sample_age[np.isfinite(sample_age)]
    valid_effective_authority = effective_authority[np.isfinite(effective_authority)]
    authority_range = maximum_authority - base_authority
    extension_occupancy = np.zeros_like(effective_authority)
    adaptable = np.isfinite(authority_range) & (authority_range > 1e-7)
    extension_occupancy[adaptable] = np.clip(
        (effective_authority[adaptable] - base_authority[adaptable])
        / authority_range[adaptable],
        0.0,
        1.0,
    )
    backoff_edges = backoff_active & ~np.r_[False, backoff_active[:-1]]
    backoff_residual = residual_norm[backoff_edges & np.isfinite(residual_norm)]

    summary.update(
        {
            "schema_complete": True,
            "usable_feedback_fraction": float(np.mean(usable)),
            "publication_age_max_s": (
                float(np.max(valid_publication_age))
                if len(valid_publication_age) > 0
                else math.nan
            ),
            "sample_age_max_s": (
                float(np.max(valid_sample_age))
                if len(valid_sample_age) > 0
                else math.nan
            ),
            "effective_authority_fraction": float(np.mean(extension_occupancy)),
            "effective_authority_max": (
                float(np.max(valid_effective_authority))
                if len(valid_effective_authority) > 0
                else math.nan
            ),
            "release_count_delta": int(np.sum(release_active & ~np.r_[False, release_active[:-1]])),
            "backoff_count_delta": reset_safe_counter_delta(
                astsmc_allocator_status["event_count[2]"], indexes
            ),
            "fallback_count_delta": reset_safe_counter_delta(
                astsmc_allocator_status["event_count[3]"], indexes
            ),
            "residual_at_backoff_max": (
                float(np.max(backoff_residual)) if len(backoff_residual) > 0 else math.nan
            ),
        }
    )

    headroom_fields = {
        "requested_allocation_policy",
        "applied_allocation_policy",
        "roll_pitch_achieved_fraction",
        "event_count[4]",
        "event_count[5]",
    }

    if headroom_fields.issubset(astsmc_allocator_status):
        requested_policy = astsmc_allocator_status["requested_allocation_policy"][window].astype(np.uint8)
        applied_policy = astsmc_allocator_status["applied_allocation_policy"][window].astype(np.uint8)
        achieved_fraction = astsmc_allocator_status["roll_pitch_achieved_fraction"][window].astype(float)
        headroom_requested = usable & (requested_policy == 1)
        headroom_applied = usable & (applied_policy == 1)
        honored_requests = headroom_requested & headroom_applied
        valid_achieved_fraction = achieved_fraction[usable & np.isfinite(achieved_fraction)]
        roll_pitch_loss = usable & np.isfinite(achieved_fraction) & (achieved_fraction < 0.995)
        loss_segments = true_segments(roll_pitch_loss, timestamps[window], minimum_duration_s=0.0)
        longest_loss = max(
            (float(timestamps[window][last] - timestamps[window][first]) for first, last in loss_segments),
            default=0.0,
        )
        requested_count = int(np.sum(headroom_requested))
        summary.update(
            {
                "headroom_schema_complete": True,
                "headroom_request_fraction": float(np.mean(headroom_requested)),
                "headroom_honor_fraction": (
                    float(np.sum(honored_requests) / requested_count)
                    if requested_count > 0
                    else 1.0
                ),
                "headroom_request_count_delta": reset_safe_counter_delta(
                    astsmc_allocator_status["event_count[4]"], indexes
                ),
                "roll_pitch_miss_count_delta": reset_safe_counter_delta(
                    astsmc_allocator_status["event_count[5]"], indexes
                ),
                "roll_pitch_achieved_fraction_mean": (
                    float(np.mean(valid_achieved_fraction))
                    if len(valid_achieved_fraction) > 0
                    else math.nan
                ),
                "roll_pitch_achieved_fraction_min": (
                    float(np.min(valid_achieved_fraction))
                    if len(valid_achieved_fraction) > 0
                    else math.nan
                ),
                "roll_pitch_allocation_loss_longest_s": longest_loss,
                "roll_conditioning_fraction": float(np.mean((state_flags & 512) != 0)),
                "pitch_conditioning_fraction": float(np.mean((state_flags & 1024) != 0)),
            }
        )

    return summary


def command_release_metrics(timestamps, rate_setpoint, rate, torque, high_threshold=0.5, low_threshold=0.15):
    timestamps = np.asarray(timestamps, dtype=float)
    rate_setpoint = np.asarray(rate_setpoint, dtype=float)
    rate = np.asarray(rate, dtype=float)
    torque = np.asarray(torque, dtype=float)
    releases = []
    large_command_active = False
    large_command_sign = 0.0

    for index in range(1, len(timestamps)):
        current = rate_setpoint[index]
        high_command = abs(current) >= high_threshold
        previous_large_command_sign = large_command_sign
        returned_to_deadband = large_command_active and abs(current) <= low_threshold
        reversed_direction = (
            large_command_active
            and high_command
            and previous_large_command_sign * current < 0.0
        )

        if not (returned_to_deadband or reversed_direction):
            if high_command:
                large_command_active = True
                large_command_sign = np.sign(current)
            continue

        if returned_to_deadband:
            large_command_active = False

        else:
            large_command_active = True
            large_command_sign = np.sign(current)

        end = np.searchsorted(timestamps, timestamps[index] + 1.0, side="right")
        window = slice(index, max(index + 1, end))
        window_timestamps = timestamps[window]
        window_rate = rate[window]
        window_setpoint = rate_setpoint[window]
        window_torque = torque[window]
        old_sign = previous_large_command_sign
        opposite_rate = np.maximum(-old_sign * window_rate, 0.0)
        wrong_direction_torque = old_sign * window_torque > 0.0
        settled = np.abs(window_rate - window_setpoint) <= 0.10
        settling_time = math.nan

        for local_index in range(len(settled)):
            dwell_end = np.searchsorted(
                window_timestamps,
                window_timestamps[local_index] + 0.20,
                side="left",
            )

            if dwell_end > local_index and np.all(settled[local_index:dwell_end]):
                settling_time = float(window_timestamps[local_index] - window_timestamps[0])
                break

        releases.append(
            {
                "time_s": float(timestamps[index]),
                "peak_opposite_rate": float(np.max(opposite_rate)),
                "absolute_rate_error_integral": float(
                    np.trapezoid(np.abs(window_setpoint - window_rate), window_timestamps)
                ),
                "settling_time_s": settling_time,
                "wrong_direction_torque_fraction": float(np.mean(wrong_direction_torque)),
            }
        )

    return releases


def analyze_segment(
    rate_timestamps,
    rates,
    rate_setpoints,
    torques,
    first,
    last,
    terminal_window_s,
    cutoff_hz,
    growth_floor,
):
    end_s = rate_timestamps[last]
    start_s = rate_timestamps[first]
    terminal_start_s = max(start_s, end_s - terminal_window_s)
    terminal_mask = (
        (rate_timestamps >= terminal_start_s)
        & (rate_timestamps <= end_s)
        & np.isfinite(rates).all(axis=1)
        & np.isfinite(rate_setpoints).all(axis=1)
    )
    terminal_timestamps = rate_timestamps[terminal_mask]
    terminal_rates = rates[terminal_mask]
    terminal_errors = rate_setpoints[terminal_mask] - terminal_rates
    terminal_torques = torques[terminal_mask]
    midpoint_s = terminal_start_s + 0.5 * (end_s - terminal_start_s)
    first_half = terminal_timestamps < midpoint_s
    second_half = ~first_half
    axes = {}

    for index, axis in enumerate(AXES):
        oscillation = high_frequency_metrics(
            terminal_timestamps, terminal_rates[:, index], cutoff_hz
        )
        first_metrics = high_frequency_metrics(
            terminal_timestamps[first_half], terminal_rates[first_half, index], cutoff_hz
        )
        second_metrics = high_frequency_metrics(
            terminal_timestamps[second_half], terminal_rates[second_half, index], cutoff_hz
        )
        axes[axis] = {
            "rate_error_rms": float(np.sqrt(np.mean(terminal_errors[:, index] ** 2))),
            "rate_abs_max": float(np.max(np.abs(terminal_rates[:, index]))),
            "torque_abs_max": float(np.max(np.abs(terminal_torques[:, index]))),
            "oscillation_rms": oscillation["rms"],
            "peak_hz": oscillation["peak_hz"],
            "peak_amplitude": oscillation["peak_amplitude"],
            "first_half_oscillation_rms": first_metrics["rms"],
            "second_half_oscillation_rms": second_metrics["rms"],
            "growth_ratio": second_metrics["rms"] / max(first_metrics["rms"], growth_floor),
            "command_releases": command_release_metrics(
                rate_timestamps[first : last + 1],
                rate_setpoints[first : last + 1, index],
                rates[first : last + 1, index],
                torques[first : last + 1, index],
            ),
        }

    return {
        "start_s": float(start_s),
        "end_s": float(end_s),
        "duration_s": float(end_s - start_s),
        "terminal_duration_s": float(end_s - terminal_start_s),
        "axes": axes,
    }


def analyze_log(path, terminal_window_s=6.0, cutoff_hz=4.0, growth_floor=0.02):
    ulog = ULog(str(path))
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    rates_setpoint = dataset(ulog, "vehicle_rates_setpoint")
    torque_setpoint = dataset(ulog, "vehicle_torque_setpoint")
    vehicle_status = dataset(ulog, "vehicle_status")
    land_detected = dataset(ulog, "vehicle_land_detected")

    rate_timestamps = angular_velocity["timestamp"].astype(float) / 1e6
    rates = vector_data(angular_velocity, ("xyz[0]", "xyz[1]", "xyz[2]"))

    setpoint_timestamps = rates_setpoint["timestamp"].astype(float) / 1e6
    setpoints = vector_data(rates_setpoint, ("roll", "pitch", "yaw"))
    sampled_setpoints = zero_order_hold(rate_timestamps, setpoint_timestamps, setpoints)

    torque_timestamps = torque_setpoint["timestamp"].astype(float) / 1e6
    torque_values = vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]"))
    sampled_torques = zero_order_hold(rate_timestamps, torque_timestamps, torque_values)

    status_timestamps = vehicle_status["timestamp"].astype(float) / 1e6
    armed_values = (vehicle_status["arming_state"] == ARMING_STATE_ARMED).astype(float)
    armed = zero_order_hold(rate_timestamps, status_timestamps, armed_values, 0.0) > 0.5
    offboard = None

    if "nav_state" in vehicle_status:
        offboard_values = (vehicle_status["nav_state"] == 14).astype(float)
        offboard = zero_order_hold(rate_timestamps, status_timestamps, offboard_values, 0.0) > 0.5

    land_timestamps = land_detected["timestamp"].astype(float) / 1e6
    landed_values = land_detected["landed"].astype(float)
    landed = zero_order_hold(rate_timestamps, land_timestamps, landed_values, 1.0) > 0.5
    valid = np.isfinite(sampled_setpoints).all(axis=1) & np.isfinite(sampled_torques).all(axis=1)
    analysis_mask = armed & ~landed & valid

    if offboard is not None and np.any(analysis_mask & offboard):
        analysis_mask &= offboard

    segments = true_segments(analysis_mask, rate_timestamps, minimum_duration_s=0.5)

    if not segments:
        raise RuntimeError("no armed-airborne segment of at least 0.5 s found")

    astsmc_status = optional_dataset(ulog, "astsmc_status")
    astsmc_safety_status = optional_dataset(ulog, "astsmc_safety_status")
    astsmc_allocator_status = optional_dataset(ulog, "astsmc_allocator_status")
    attitude = optional_dataset(ulog, "vehicle_attitude")
    attitude_setpoint = optional_dataset(ulog, "vehicle_attitude_setpoint")
    controller_type = int(ulog.initial_parameters.get("MC_RATE_CTRL_T", -1))
    roll_pitch_residual_extension = float(
        ulog.initial_parameters.get("MC_AST_RP_EXT", 0.0)
    )
    roll_pitch_residual_card = [
        (
            float(ulog.initial_parameters.get(f"MC_AST_TRES_{suffix}", math.nan)),
            min(
                float(ulog.initial_parameters.get(f"MC_AST_TMAX_{suffix}", math.nan)),
                float(ulog.initial_parameters.get(f"MC_AST_TRES_{suffix}", math.nan))
                + roll_pitch_residual_extension,
            ),
        )
        for suffix in ("R", "P")
    ]

    if controller_type == 3:
        analysis_mask = exclude_astsmc_ground_containment(
            rate_timestamps, analysis_mask, astsmc_safety_status
        )
        segments = true_segments(analysis_mask, rate_timestamps, minimum_duration_s=0.5)

        if not segments:
            raise RuntimeError("no active Type-3 segment of at least 0.5 s found")

    results = []

    for first, last in segments:
        segment = analyze_segment(
            rate_timestamps,
            rates,
            sampled_setpoints,
            sampled_torques,
            first,
            last,
            terminal_window_s,
            cutoff_hz,
            growth_floor,
        )
        segment["astsmc_safety"] = astsmc_safety_summary(
            astsmc_status,
            astsmc_safety_status,
            segment["start_s"],
            segment["end_s"],
            roll_pitch_residual_card,
        )
        segment["astsmc_allocator"] = astsmc_allocator_summary(
            astsmc_allocator_status,
            segment["start_s"],
            segment["end_s"],
        )
        segment["yaw_attitude"] = yaw_attitude_summary(
            attitude,
            attitude_setpoint,
            segment["start_s"],
            segment["end_s"],
        )
        results.append(segment)

    return {"path": path, "segments": results}


def check_result(
    result,
    max_oscillation_rms_rp,
    max_rate_error_rms_rp,
    max_growth_ratio,
    max_oscillation_rms_yaw=0.06,
    max_rate_error_rms_yaw=0.10,
    max_residual_limit_occupancy=0.05,
    max_final_yaw_error_deg=5.0,
):
    failures = []

    for segment_index, segment in enumerate(result["segments"], start=1):
        print(
            f"segment {segment_index}: airborne={segment['duration_s']:.3f}s "
            f"terminal={segment['terminal_duration_s']:.3f}s"
        )

        astsmc_safety = segment.get("astsmc_safety", {})

        if astsmc_safety.get("present"):
            print(
                "  ASTSMC "
                f"schema_complete={astsmc_safety['schema_complete']} "
                f"fault={astsmc_safety['runtime_fault_latched']} "
                f"reason={astsmc_safety['runtime_fault_reason']} "
                f"recoveries={astsmc_safety['state_recovery_count_delta']} "
                f"ground_containment={astsmc_safety['ground_containment_count_delta']} "
                f"selective_release={astsmc_safety.get('selective_release_count_delta')} "
                f"quiet_anchor={astsmc_safety.get('quiet_anchor_count_delta')} "
                f"quiet_anchor_fraction={astsmc_safety.get('quiet_anchor_active_fraction')} "
                f"deep_quiet_fraction={astsmc_safety.get('deep_quiet_active_fraction')} "
                f"trim_valid={astsmc_safety.get('trim_valid')} "
                f"trim_confidence_max={astsmc_safety.get('trim_confidence_max')} "
                f"residual_occupancy={astsmc_safety['residual_limit_occupancy']} "
                f"wrong_direction_escapes={astsmc_safety['wrong_direction_escape_count']}"
            )
            safety_checks = [
                ("safety schema complete", astsmc_safety["schema_complete"]),
                (
                    "wrong-direction large-error escape absent",
                    astsmc_safety["wrong_direction_escape_count"] == 0,
                ),
            ]

            if astsmc_safety["schema_complete"]:
                safety_checks.extend(
                    [
                        ("runtime fault clear", not astsmc_safety["runtime_fault_latched"]),
                        ("state recovery absent", astsmc_safety["state_recovery_count_delta"] == 0),
                    ]
                )

            for label, passed in safety_checks:
                print(f"    {'PASS' if passed else 'FAIL'} ASTSMC {label}")

                if not passed:
                    failures.append(f"segment {segment_index} ASTSMC {label}")

            roll_pitch_authority = astsmc_safety.get(
                "roll_pitch_residual_authority_max"
            )

            if roll_pitch_authority is not None:
                print(
                    "    ASTSMC roll/pitch residual authority "
                    f"base={astsmc_safety['roll_pitch_residual_authority_base']} "
                    f"max={roll_pitch_authority} "
                    "extension_fraction="
                    f"{astsmc_safety['roll_pitch_residual_authority_extension_fraction']} "
                    "full_fraction="
                    f"{astsmc_safety['roll_pitch_residual_authority_full_fraction']}"
                )

            occupancy = astsmc_safety.get("residual_limit_occupancy")

            if occupancy is not None:
                for axis, value in zip(AXES, occupancy):
                    passed = np.isfinite(value) and value <= max_residual_limit_occupancy
                    print(
                        f"    {'PASS' if passed else 'FAIL'} ASTSMC {axis} residual-limit occupancy: "
                        f"{value:.4f} <= {max_residual_limit_occupancy:.4f}"
                    )

                    if not passed:
                        failures.append(
                            f"segment {segment_index} ASTSMC {axis} residual-limit occupancy"
                        )

        astsmc_allocator = segment.get("astsmc_allocator", {})

        if astsmc_allocator.get("present"):
            print(
                "  ASTSMC allocator "
                f"schema_complete={astsmc_allocator['schema_complete']} "
                f"usable_fraction={astsmc_allocator['usable_feedback_fraction']} "
                f"publication_age_max={astsmc_allocator['publication_age_max_s']} "
                f"sample_age_max={astsmc_allocator['sample_age_max_s']} "
                f"authority_fraction={astsmc_allocator['effective_authority_fraction']} "
                f"authority_max={astsmc_allocator['effective_authority_max']} "
                f"release_episodes={astsmc_allocator['release_count_delta']} "
                f"backoff={astsmc_allocator['backoff_count_delta']} "
                f"fallback={astsmc_allocator['fallback_count_delta']} "
                f"residual_at_backoff_max={astsmc_allocator['residual_at_backoff_max']} "
                f"headroom_schema={astsmc_allocator['headroom_schema_complete']} "
                f"headroom_request_fraction={astsmc_allocator['headroom_request_fraction']} "
                f"headroom_honor_fraction={astsmc_allocator['headroom_honor_fraction']} "
                f"rp_achieved_mean={astsmc_allocator['roll_pitch_achieved_fraction_mean']} "
                f"rp_achieved_min={astsmc_allocator['roll_pitch_achieved_fraction_min']} "
                f"rp_loss_longest={astsmc_allocator['roll_pitch_allocation_loss_longest_s']} "
                f"roll_conditioning={astsmc_allocator['roll_conditioning_fraction']} "
                f"pitch_conditioning={astsmc_allocator['pitch_conditioning_fraction']}"
            )

        for axis in AXES:
            metrics = segment["axes"][axis]
            print(
                f"  {axis:5s} rate_error_rms={metrics['rate_error_rms']:.4f} "
                f"rate_max={metrics['rate_abs_max']:.4f} "
                f"osc_rms={metrics['oscillation_rms']:.4f} "
                f"peak={metrics['peak_hz']:.2f}Hz/{metrics['peak_amplitude']:.4f}rad/s "
                f"growth={metrics['growth_ratio']:.2f}x "
                f"torque_max={metrics['torque_abs_max']:.4f}"
            )

            if axis == "yaw":
                checks = (
                    ("terminal oscillation RMS", metrics["oscillation_rms"], max_oscillation_rms_yaw),
                    ("terminal rate-error RMS", metrics["rate_error_rms"], max_rate_error_rms_yaw),
                    ("terminal oscillation growth", metrics["growth_ratio"], max_growth_ratio),
                )

            else:
                checks = (
                    ("terminal oscillation RMS", metrics["oscillation_rms"], max_oscillation_rms_rp),
                    ("terminal rate-error RMS", metrics["rate_error_rms"], max_rate_error_rms_rp),
                    ("terminal oscillation growth", metrics["growth_ratio"], max_growth_ratio),
                )

            for label, value, limit in checks:
                passed = np.isfinite(value) and value <= limit
                print(f"    {'PASS' if passed else 'FAIL'} {label}: {value:.4f} <= {limit:.4f}")

                if not passed:
                    failures.append(f"segment {segment_index} {axis} {label}")

            releases = metrics.get("command_releases", [])

            if axis in ("roll", "pitch"):
                print(f"    command releases={len(releases)}")

                for release in releases:
                    settled = release["settling_time_s"]
                    settling_passed = np.isfinite(settled) and settled <= 1.0
                    wrong_direction_passed = release["wrong_direction_torque_fraction"] <= 0.05
                    print(
                        f"      t={release['time_s']:.3f}s "
                        f"opposite_rate={release['peak_opposite_rate']:.4f}rad/s "
                        f"IAE={release['absolute_rate_error_integral']:.4f} "
                        f"settling={settled:.3f}s "
                        f"wrong_torque={release['wrong_direction_torque_fraction']:.4f}"
                    )

                    if not settling_passed:
                        failures.append(f"segment {segment_index} {axis} command-release settling")

                    if not wrong_direction_passed:
                        failures.append(f"segment {segment_index} {axis} command-release wrong torque")

        yaw_attitude = segment.get("yaw_attitude", {"available": False})

        if yaw_attitude.get("available"):
            final_error = yaw_attitude["final_error_abs_deg"]
            passed = np.isfinite(final_error) and final_error <= max_final_yaw_error_deg
            print(
                f"  yaw attitude error_rms={yaw_attitude['error_rms_deg']:.2f}deg "
                f"max={yaw_attitude['error_abs_max_deg']:.2f}deg "
                f"final={final_error:.2f}deg"
            )
            print(
                f"    {'PASS' if passed else 'FAIL'} final yaw attitude error: "
                f"{final_error:.2f} <= {max_final_yaw_error_deg:.2f} deg"
            )

            if not passed:
                failures.append(f"segment {segment_index} yaw final attitude error")

        else:
            print("  yaw attitude gate unavailable (attitude/setpoint topic missing)")

    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--terminal-window", type=float, default=6.0)
    parser.add_argument("--rate-osc-cutoff", type=float, default=4.0)
    parser.add_argument("--growth-floor", type=float, default=0.02)
    parser.add_argument("--max-rate-osc-rms-rp", type=float, default=0.08)
    parser.add_argument("--max-rate-error-rms-rp", type=float, default=0.12)
    parser.add_argument("--max-growth-ratio", type=float, default=2.5)
    parser.add_argument("--max-rate-osc-rms-yaw", type=float, default=0.06)
    parser.add_argument("--max-rate-error-rms-yaw", type=float, default=0.10)
    parser.add_argument("--max-residual-limit-occupancy", type=float, default=0.05)
    parser.add_argument("--max-final-yaw-error-deg", type=float, default=5.0)
    args = parser.parse_args()
    all_failures = []

    for path in args.logs:
        print(f"\n== {path.name}")
        result = analyze_log(
            path,
            terminal_window_s=args.terminal_window,
            cutoff_hz=args.rate_osc_cutoff,
            growth_floor=args.growth_floor,
        )
        failures = check_result(
            result,
            max_oscillation_rms_rp=args.max_rate_osc_rms_rp,
            max_rate_error_rms_rp=args.max_rate_error_rms_rp,
            max_growth_ratio=args.max_growth_ratio,
            max_oscillation_rms_yaw=args.max_rate_osc_rms_yaw,
            max_rate_error_rms_yaw=args.max_rate_error_rms_yaw,
            max_residual_limit_occupancy=args.max_residual_limit_occupancy,
            max_final_yaw_error_deg=args.max_final_yaw_error_deg,
        )

        if failures:
            all_failures.extend(f"{path.name}: {failure}" for failure in failures)

    if all_failures:
        print("\nAIRBORNE HEALTH FAILED")

        for failure in all_failures:
            print(f"- {failure}")

        raise SystemExit(1)

    print("\nAIRBORNE HEALTH PASSED")


if __name__ == "__main__":
    main()
