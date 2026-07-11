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


def logged_controller_parameters(ulog, at_timestamp_s=None):
    names = [
        "MC_RATE_CTRL_T",
        "MC_MSMC_CFG", "MIS_TKO_ALT_MAX",
        "MC_MPC_J_R", "MC_MPC_J_P", "MC_MPC_J_Y",
        "MC_MPC_EFF_R", "MC_MPC_EFF_P", "MC_MPC_EFF_Y",
        "MC_MPC_Q_R", "MC_MPC_Q_P", "MC_MPC_Q_Y",
        "MC_MPC_R_R", "MC_MPC_R_P", "MC_MPC_R_Y",
        "MC_MPC_DU_R", "MC_MPC_DU_P", "MC_MPC_DU_Y",
        "MC_MPC_TAU", "MC_MPC_HORIZON", "MC_MPC_SLEW", "MC_MPC_GYRO",
        "MC_MPC_TMAX_R", "MC_MPC_TMAX_P", "MC_MPC_TMAX_Y",
        "MC_MSMC_J_R", "MC_MSMC_J_P", "MC_MSMC_J_Y",
        "MC_MSMC_EFF_R", "MC_MSMC_EFF_P", "MC_MSMC_EFF_Y",
        "MC_MSMC_C_R", "MC_MSMC_C_P", "MC_MSMC_C_Y",
        "MC_MSMC_ETA_R", "MC_MSMC_ETA_P", "MC_MSMC_ETA_Y",
        "MC_MSMC_BND_R", "MC_MSMC_BND_P", "MC_MSMC_BND_Y",
        "MC_MSMC_KS_R", "MC_MSMC_KS_P", "MC_MSMC_KS_Y",
        "MC_MSMC_RSPD_L", "MC_SMC_LPF", "MC_SMC_SLEW",
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

    rate_timestamps = angular_velocity["timestamp"].astype(float) / 1e6
    rates = vector_data(angular_velocity, ("xyz[0]", "xyz[1]", "xyz[2]"))
    rate_sp_timestamps = rates_setpoint["timestamp"].astype(float) / 1e6
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

    torque_timestamps = torque_setpoint["timestamp"].astype(float) / 1e6
    torque_mask = (
        (torque_timestamps >= offboard_start_s)
        & (torque_timestamps <= min(offboard_end_s, planned_maneuver_end_s))
    )
    torques = vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]"))[torque_mask]
    parameters = logged_controller_parameters(ulog, offboard_start_s)
    controller_parameter_changes = [
        (timestamp / 1e6, name, value)
        for timestamp, name, value in ulog.changed_parameters
        if name in parameters and offboard_start_s < timestamp / 1e6 <= offboard_end_s
    ]
    controller_type = parameters["MC_RATE_CTRL_T"]
    limit_prefix = "MC_MPC_TMAX" if controller_type == 1 else "MC_MSMC_TMAX"
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
        "torque": torque_stats,
        "torque_limits": dict(zip(AXES, torque_limits.tolist())),
        "torque_limit_fraction": torque_limit_fraction,
        "yaw_steps": yaw_steps,
        "smc_status_valid": smc_status_valid,
        "smc_status_samples": smc_status_samples,
        "smc_diagnostics": smc_diagnostics,
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
            f"torque_limit_frac={max(result['torque_limit_fraction'].values()):.4f},"
            f"torque_all={result['torque_all']},"
            f"parameter_changes={len(result['controller_parameter_changes'])},"
            f"failsafe={result['failsafe']},"
            f"motor_max={result['motor_max']:.3f},"
            f"smc_status_valid={result['smc_status_valid']}"
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
    parser.add_argument("--takeoff-hold", type=float, default=7.0)
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
