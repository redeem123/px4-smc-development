#!/usr/bin/env python3
"""Estimate UAV985 torque effectiveness and actuator lag from a PX4 ULog."""

import argparse
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


ARMING_STATE_ARMED = 2
OFFBOARD_NAV_STATE = 14
AXES = ("roll", "pitch", "yaw")


def parse_vector(value):
    values = np.array([float(item) for item in value.split(",")], dtype=float)

    if values.shape != (3,) or not np.isfinite(values).all() or np.any(values <= 0.0):
        raise argparse.ArgumentTypeError("expected three positive comma-separated values")

    return values


def dataset(ulog, name):
    try:
        return next(data.data for data in ulog.data_list if data.name == name)
    except StopIteration as exc:
        raise RuntimeError(f"required ULog topic is missing: {name}") from exc


def optional_dataset(ulog, name):
    return next((data.data for data in ulog.data_list if data.name == name), None)


def vector_data(data, fields):
    return np.column_stack([data[field] for field in fields]).astype(float)


def zero_order_hold(sample_timestamps, source_timestamps, source_values):
    indexes = np.searchsorted(source_timestamps, sample_timestamps, side="right") - 1
    valid = indexes >= 0
    result = np.full((len(sample_timestamps),) + source_values.shape[1:], np.nan)
    result[valid] = source_values[indexes[valid]]
    return result


def simulate_actuator(timestamps, commands, time_constant):
    state = np.empty_like(commands)
    state[0] = commands[0]

    for index in range(1, len(commands)):
        dt = np.clip(timestamps[index] - timestamps[index - 1], 1e-5, 0.1)
        alpha = np.exp(-dt / time_constant)
        state[index] = alpha * state[index - 1] + (1.0 - alpha) * commands[index - 1]

    return state


def fit_axis(timestamps, command, rate, physical_torque, tau_candidates):
    best = None

    for time_constant in tau_candidates:
        actuator_state = simulate_actuator(timestamps, command, time_constant)
        regressors = np.column_stack([actuator_state, -rate, np.ones(len(rate))])
        coefficients, _, _, _ = np.linalg.lstsq(regressors, physical_torque, rcond=None)
        prediction = regressors @ coefficients
        residual = physical_torque - prediction
        residual_sum = float(residual @ residual)

        if coefficients[0] <= 0.0:
            continue

        if best is None or residual_sum < best[0]:
            total_sum = float(np.sum((physical_torque - np.mean(physical_torque)) ** 2))
            r_squared = 1.0 - residual_sum / total_sum if total_sum > 1e-12 else np.nan
            best = (residual_sum, time_constant, coefficients, r_squared)

    if best is None:
        raise RuntimeError("no positive-effectiveness fit found")

    _, time_constant, coefficients, r_squared = best
    return {
        "tau": float(time_constant),
        "tau_at_boundary": bool(time_constant == tau_candidates[0] or time_constant == tau_candidates[-1]),
        "effectiveness": float(coefficients[0]),
        "damping": float(coefficients[1]),
        "bias": float(coefficients[2]),
        "r_squared": float(r_squared),
        "command_std": float(np.std(command)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--inertia", type=parse_vector, default=np.array([0.0135, 0.0118, 0.0170]))
    parser.add_argument("--tau-min", type=float, default=0.005)
    parser.add_argument("--tau-max", type=float, default=0.080)
    parser.add_argument("--tau-steps", type=int, default=151)
    parser.add_argument("--start", type=float, default=0.0, help="seconds after the first armed sample")
    parser.add_argument("--end", type=float, help="seconds after the first armed sample")
    parser.add_argument("--offboard-only", action="store_true")
    parser.add_argument("--axis", choices=(*AXES, "all"), default="all")
    parser.add_argument("--min-command-std", type=float, default=0.01)
    parser.add_argument("--min-r-squared", type=float, default=0.5)
    parser.add_argument("--min-damping", type=float, default=0.0)
    parser.add_argument("--max-effectiveness", type=float, default=10.0)
    parser.add_argument("--motor-margin", type=float, default=0.01)
    parser.add_argument("--min-airborne-seconds", type=float, default=2.0)
    parser.add_argument("--max-thrust-span", type=float, default=0.25)
    args = parser.parse_args()

    if args.tau_min <= 0.0 or args.tau_max <= args.tau_min or args.tau_steps < 2:
        raise ValueError("invalid actuator time-constant search range")

    if (
        args.min_command_std < 0.0
        or not 0.0 <= args.min_r_squared <= 1.0
        or not np.isfinite(args.min_damping)
        or not np.isfinite(args.max_effectiveness)
        or args.max_effectiveness <= 0.0
        or not 0.0 <= args.motor_margin < 0.5
        or args.min_airborne_seconds <= 0.0
        or args.max_thrust_span <= 0.0
    ):
        raise ValueError("invalid identification acceptance threshold")

    ulog = ULog(str(args.log))
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    torque_setpoint = dataset(ulog, "vehicle_torque_setpoint")
    vehicle_status = dataset(ulog, "vehicle_status")
    land_detected = dataset(ulog, "vehicle_land_detected")
    allocator_status = dataset(ulog, "control_allocator_status")
    thrust_setpoint = dataset(ulog, "vehicle_thrust_setpoint")
    actuator_motors = optional_dataset(ulog, "actuator_motors")

    timestamps = angular_velocity["timestamp"].astype(float) / 1e6
    rates = vector_data(angular_velocity, ("xyz[0]", "xyz[1]", "xyz[2]"))
    accelerations = vector_data(
        angular_velocity, ("xyz_derivative[0]", "xyz_derivative[1]", "xyz_derivative[2]")
    )
    torque_timestamps = torque_setpoint["timestamp"].astype(float) / 1e6
    commands = zero_order_hold(
        timestamps,
        torque_timestamps,
        vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]")),
    )
    status_timestamps = vehicle_status["timestamp"].astype(float) / 1e6
    arming_state = zero_order_hold(
        timestamps, status_timestamps, vehicle_status["arming_state"].astype(float)[:, np.newaxis]
    )[:, 0]
    nav_state = zero_order_hold(
        timestamps, status_timestamps, vehicle_status["nav_state"].astype(float)[:, np.newaxis]
    )[:, 0]
    armed = arming_state == ARMING_STATE_ARMED
    landed = zero_order_hold(
        timestamps,
        land_detected["timestamp"].astype(float) / 1e6,
        land_detected["landed"].astype(float)[:, np.newaxis],
    )[:, 0]
    allocator_timestamps = allocator_status["timestamp"].astype(float) / 1e6
    torque_achieved = zero_order_hold(
        timestamps,
        allocator_timestamps,
        allocator_status["torque_setpoint_achieved"].astype(float)[:, np.newaxis],
    )[:, 0]
    thrust_achieved = zero_order_hold(
        timestamps,
        allocator_timestamps,
        allocator_status["thrust_setpoint_achieved"].astype(float)[:, np.newaxis],
    )[:, 0]
    thrust_commands = zero_order_hold(
        timestamps,
        thrust_setpoint["timestamp"].astype(float) / 1e6,
        vector_data(thrust_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]")),
    )
    collective_thrust = np.linalg.norm(thrust_commands, axis=1)
    motor_commands = None

    if actuator_motors is not None:
        motor_fields = tuple(f"control[{index}]" for index in range(4))
        motor_commands = zero_order_hold(
            timestamps,
            actuator_motors["timestamp"].astype(float) / 1e6,
            vector_data(actuator_motors, motor_fields),
        )

    if args.offboard_only:
        armed &= nav_state == OFFBOARD_NAV_STATE

    if not np.any(armed):
        raise RuntimeError("no matching armed samples found")

    first_armed = timestamps[np.flatnonzero(armed)[0]]
    mask = armed & (landed == 0.0) & (torque_achieved == 1.0) & (thrust_achieved == 1.0)
    mask &= timestamps >= first_armed + args.start

    if args.end is not None:
        mask &= timestamps <= first_armed + args.end

    mask &= np.isfinite(rates).all(axis=1)
    mask &= np.isfinite(accelerations).all(axis=1)
    mask &= np.isfinite(commands).all(axis=1)
    mask &= np.isfinite(collective_thrust)

    if motor_commands is not None:
        mask &= np.isfinite(motor_commands).all(axis=1)

        if args.motor_margin > 0.0:
            mask &= np.all(motor_commands > args.motor_margin, axis=1)
            mask &= np.all(motor_commands < 1.0 - args.motor_margin, axis=1)

    timestamps = timestamps[mask]
    rates = rates[mask]
    accelerations = accelerations[mask]
    commands = commands[mask]
    collective_thrust = collective_thrust[mask]

    if motor_commands is not None:
        motor_commands = motor_commands[mask]

    if len(timestamps) < 100:
        raise RuntimeError(f"only {len(timestamps)} usable samples; at least 100 are required")

    duration = timestamps[-1] - timestamps[0]

    if duration < args.min_airborne_seconds:
        raise RuntimeError(
            f"only {duration:.3f}s of usable airborne data; at least {args.min_airborne_seconds:.3f}s is required"
        )

    thrust_low, thrust_median, thrust_high = np.percentile(collective_thrust, (5.0, 50.0, 95.0))
    thrust_span = thrust_high - thrust_low

    if thrust_span > args.max_thrust_span:
        raise RuntimeError(
            f"collective thrust span {thrust_span:.3f} exceeds {args.max_thrust_span:.3f}; "
            "identify separate local operating windows"
        )

    angular_momentum = rates * args.inertia
    gyro_torque = np.cross(rates, angular_momentum)
    physical_torque = accelerations * args.inertia + gyro_torque
    tau_candidates = np.linspace(args.tau_min, args.tau_max, args.tau_steps)

    print(f"log={args.log}")
    print(f"samples={len(timestamps)} duration={duration:.3f}s")
    print(
        "operating-window: "
        f"collective_thrust_p05={thrust_low:.4f} median={thrust_median:.4f} "
        f"p95={thrust_high:.4f} span={thrust_span:.4f}"
    )

    if motor_commands is not None:
        motor_low, motor_high = np.percentile(motor_commands, (5.0, 95.0))
        print(f"motor-command-window: p05={motor_low:.4f} p95={motor_high:.4f}")
    else:
        print("motor-command-window: unavailable (actuator_motors topic missing)")

    print("model: J*omega_dot + omega x (J*omega) = effectiveness*actuator - damping*omega + bias")

    estimates = []
    identified = []

    selected_axes = AXES if args.axis == "all" else (args.axis,)

    for axis_index, axis in enumerate(AXES):
        if axis not in selected_axes:
            estimates.append(None)
            identified.append(False)
            continue

        estimate = fit_axis(
            timestamps,
            commands[:, axis_index],
            rates[:, axis_index],
            physical_torque[:, axis_index],
            tau_candidates,
        )
        estimates.append(estimate)
        rejection_reasons = []

        if estimate["command_std"] < args.min_command_std:
            rejection_reasons.append("low-excitation")

        if not np.isfinite(estimate["r_squared"]) or estimate["r_squared"] < args.min_r_squared:
            rejection_reasons.append("low-R2")

        if estimate["damping"] < args.min_damping:
            rejection_reasons.append("nonphysical-damping")

        if estimate["effectiveness"] > args.max_effectiveness:
            rejection_reasons.append("effectiveness-out-of-range")

        if estimate["tau_at_boundary"]:
            rejection_reasons.append("tau-search-boundary")

        axis_identified = not rejection_reasons
        identified.append(axis_identified)
        print(
            f"{axis:5s} eff={estimate['effectiveness']:.6f} Nm/unit "
            f"tau={estimate['tau']:.4f}s damping={estimate['damping']:.6f} Nms "
            f"bias={estimate['bias']:.6f} Nm R2={estimate['r_squared']:.3f} "
            f"command_std={estimate['command_std']:.4f} "
            f"identification={'ACCEPTED' if axis_identified else 'REJECTED'} "
            f"reasons={','.join(rejection_reasons) if rejection_reasons else 'none'}"
        )

        if axis_identified:
            suffix = "RPY"[axis_index]
            print(
                f"recommended-{axis}: MC_MPC_EFF_{suffix}={estimate['effectiveness']:.6f} "
                f"MC_MSMC_EFF_{suffix}={estimate['effectiveness']:.6f}"
            )

    requested_identified = [identified[AXES.index(axis)] for axis in selected_axes]

    if all(requested_identified):
        accepted_estimates = [estimates[AXES.index(axis)] for axis in selected_axes]

        if len(accepted_estimates) == 3:
            print(f"recommended-shared: MC_MPC_TAU={np.median([item['tau'] for item in accepted_estimates]):.4f}")

        print(
            "review the inertia source, operating window, and every SMC gain/limit; "
            "set MC_MSMC_CFG=1 manually only after completing that review"
        )
        print("IDENTIFICATION PASSED")
        return 0

    accepted_axes = ", ".join(axis for axis in selected_axes if identified[AXES.index(axis)]) or "none"
    rejected_axes = ", ".join(axis for axis in selected_axes if not identified[AXES.index(axis)])
    print(f"accepted axes: {accepted_axes}")
    print(f"rejected axes: {rejected_axes}")
    print("IDENTIFICATION INCOMPLETE: no combined controller parameter recommendation was produced.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
