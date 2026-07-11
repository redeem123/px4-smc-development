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

    land_timestamps = land_detected["timestamp"].astype(float) / 1e6
    landed_values = land_detected["landed"].astype(float)
    landed = zero_order_hold(rate_timestamps, land_timestamps, landed_values, 1.0) > 0.5
    valid = np.isfinite(sampled_setpoints).all(axis=1) & np.isfinite(sampled_torques).all(axis=1)
    segments = true_segments(armed & ~landed & valid, rate_timestamps, minimum_duration_s=0.5)

    if not segments:
        raise RuntimeError("no armed-airborne segment of at least 0.5 s found")

    results = [
        analyze_segment(
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
        for first, last in segments
    ]
    return {"path": path, "segments": results}


def check_result(result, max_oscillation_rms_rp, max_rate_error_rms_rp, max_growth_ratio):
    failures = []

    for segment_index, segment in enumerate(result["segments"], start=1):
        print(
            f"segment {segment_index}: airborne={segment['duration_s']:.3f}s "
            f"terminal={segment['terminal_duration_s']:.3f}s"
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
                continue

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
