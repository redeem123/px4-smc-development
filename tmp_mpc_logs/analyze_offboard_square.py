#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


OFFBOARD_NAV_STATE = 14
ARMING_STATE_ARMED = 2

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
    sampled = np.full((len(sample_timestamps), 3), np.nan)
    sampled[valid_indexes] = setpoints[indexes[valid_indexes]]
    valid_samples = np.isfinite(sampled).all(axis=1)
    return sampled, valid_samples


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

    offboard_start_s, offboard_end_s = first_offboard_window(vehicle_status)
    planned_square_end_s = offboard_start_s + args.takeoff_hold + (args.move + args.hold) * 4.0

    position_timestamps = local_position["timestamp"].astype(float) / 1e6
    positions = np.column_stack(
        [
            local_position["x"],
            local_position["y"],
            local_position["z"],
        ]
    ).astype(float)

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

    windows = final_windows(args.takeoff_hold, args.move, args.hold, args.final_s, waypoints)
    final_errors = []
    waypoint_results = []

    for name, target in waypoints:
        window_start, window_end = windows[name]
        waypoint_mask = (
            (position_timestamps >= offboard_start_s + window_start)
            & (position_timestamps <= offboard_start_s + window_end)
        )
        errors = np.linalg.norm(positions[waypoint_mask] - target, axis=1)
        final_errors.extend(errors.tolist())
        waypoint_results.append((name, stats(errors)))

    allocator_timestamps = control_allocator["timestamp"].astype(float) / 1e6
    allocator_mask = (allocator_timestamps >= offboard_start_s) & (allocator_timestamps <= offboard_end_s)
    torque_achieved = bool(np.all(control_allocator["torque_setpoint_achieved"][allocator_mask]))
    thrust_achieved = bool(np.all(control_allocator["thrust_setpoint_achieved"][allocator_mask]))

    unallocated_torque = np.column_stack(
        [
            control_allocator["unallocated_torque[0]"],
            control_allocator["unallocated_torque[1]"],
            control_allocator["unallocated_torque[2]"],
        ]
    ).astype(float)
    unallocated_thrust = np.column_stack(
        [
            control_allocator["unallocated_thrust[0]"],
            control_allocator["unallocated_thrust[1]"],
            control_allocator["unallocated_thrust[2]"],
        ]
    ).astype(float)

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
        "waypoints": waypoint_results,
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
        print(
            f"{result['path'].name},"
            f"track_rms={result['track_square']['rms']:.4f},"
            f"track_p95={result['track_square']['p95']:.4f},"
            f"track_max={result['track_square']['max']:.4f},"
            f"final_mean={result['final']['mean']:.4f},"
            f"final_p95={result['final']['p95']:.4f},"
            f"final_max={result['final']['max']:.4f},"
            f"torque_all={result['torque_all']},"
            f"failsafe={result['failsafe']},"
            f"motor_max={result['motor_max']:.3f}"
        )
        return

    print(f"\n== {result['path'].name} offboard={result['offboard_duration_s']:.3f}s")
    print(f"track_all_vs_trajectory_sp    {format_stats(result['track_all'])}")
    print(f"track_square_vs_trajectory_sp {format_stats(result['track_square'])}")
    print(f"final{final_s:g}_vs_fixed_waypoints  {format_stats(result['final'])}")
    print(
        "health "
        f"torque_all={result['torque_all']} thrust_all={result['thrust_all']} "
        f"failsafe={result['failsafe']} "
        f"unalloc_torque_max={result['unallocated_torque_max']:.5f} "
        f"unalloc_thrust_max={result['unallocated_thrust_max']:.5f}"
    )
    print(
        "motors "
        f"min={result['motor_min']:.3f} max={result['motor_max']:.3f} "
        f"near_low={result['motor_near_low']} near_high={result['motor_near_high']}"
    )
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
    parser.add_argument("--final-s", type=float, default=1.0)
    parser.add_argument("--motor-low", type=float, default=0.05)
    parser.add_argument("--motor-high", type=float, default=0.95)
    parser.add_argument("--compact", action="store_true")
    parsed_args = parser.parse_args()

    for log_path in parsed_args.logs:
        print_result(analyze_log(log_path, parsed_args), parsed_args.final_s, parsed_args.compact)


if __name__ == "__main__":
    main()
