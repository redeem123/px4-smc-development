#!/usr/bin/env python3
"""Estimate ASTSMC residual disturbance and authority bounds from PX4 ULogs."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


ARMING_STATE_ARMED = 2
AXES = ("roll", "pitch", "yaw")
SUFFIXES = ("R", "P", "Y")


class VelocityReference:
    """Scalar port of PX4 VelocitySmoothing for offline card analysis."""

    def __init__(self, acceleration_limit, jerk_limit, initial_velocity=0.0):
        self.acceleration_limit = float(acceleration_limit)
        self.jerk_limit = float(jerk_limit)
        self.acceleration = 0.0
        self.velocity = float(initial_velocity)
        self.position = 0.0

    @staticmethod
    def _sign(value):
        return 1 if value > 0.0 else (-1 if value < 0.0 else 0)

    def _velocity_at_zero_acceleration(self):
        velocity = self.velocity

        if abs(self.acceleration) > np.finfo(float).eps:
            jerk = -self._sign(self.acceleration) * self.jerk_limit
            duration = -self.acceleration / jerk
            velocity += self.acceleration * duration + 0.5 * jerk * duration * duration

        return velocity

    def _compute_t1(self, acceleration, delta_velocity, jerk):
        discriminant = 2.0 * acceleration * acceleration + 4.0 * jerk * delta_velocity

        if discriminant < 0.0:
            return 0.0

        root = math.sqrt(discriminant)
        candidates = (
            (-acceleration + 0.5 * root) / jerk,
            (-acceleration - 0.5 * root) / jerk,
        )
        duration = 0.0

        for candidate in candidates:
            t3 = acceleration / jerk + candidate

            if candidate >= 0.0 and t3 >= 0.0:
                duration = candidate
                break

        acceleration_at_t1 = acceleration + jerk * duration

        if acceleration_at_t1 > self.acceleration_limit:
            duration = (self.acceleration_limit - acceleration) / jerk
        elif acceleration_at_t1 < -self.acceleration_limit:
            duration = (-self.acceleration_limit - acceleration) / jerk

        return max(duration, 0.0)

    @staticmethod
    def _compute_t2(t1, t3, acceleration, delta_velocity, jerk):
        denominator = acceleration + jerk * t1

        if abs(denominator) <= np.finfo(float).eps:
            return 0.0

        duration = (
            -0.5 * t1 * t1 * jerk
            - t1 * t3 * jerk
            - t1 * acceleration
            + 0.5 * t3 * t3 * jerk
            - t3 * acceleration
            + delta_velocity
        ) / denominator
        return max(duration, 0.0)

    @staticmethod
    def _evaluate(jerk, acceleration, velocity, position, duration):
        duration_squared = duration * duration
        return (
            jerk,
            acceleration + jerk * duration,
            velocity + acceleration * duration + 0.5 * jerk * duration_squared,
            position
            + velocity * duration
            + 0.5 * acceleration * duration_squared
            + jerk * duration_squared * duration / 6.0,
        )

    def update(self, target, dt):
        direction = self._sign(float(target) - self._velocity_at_zero_acceleration())

        if direction == 0:
            direction = self._sign(self.acceleration)

        jerk = direction * self.jerk_limit
        delta_velocity = float(target) - self.velocity

        if abs(jerk) <= np.finfo(float).eps:
            t1 = t2 = t3 = 0.0
        else:
            t1 = self._compute_t1(self.acceleration, delta_velocity, jerk)
            t3 = max(self.acceleration / jerk + t1, 0.0)
            t2 = self._compute_t2(t1, t3, self.acceleration, delta_velocity, jerk)

        remaining = float(dt)
        initial_acceleration = self.acceleration
        initial_velocity = self.velocity
        initial_position = self.position
        first_duration = min(remaining, t1)
        state = self._evaluate(jerk, initial_acceleration, initial_velocity, initial_position, first_duration)
        remaining -= first_duration

        if remaining > 0.0:
            second_duration = min(remaining, t2)
            state = self._evaluate(0.0, state[1], state[2], state[3], second_duration)
            remaining -= second_duration

        if remaining > 0.0:
            third_duration = min(remaining, t3)
            state = self._evaluate(-jerk, state[1], state[2], state[3], third_duration)
            remaining -= third_duration

        if remaining > 0.0:
            state = self._evaluate(0.0, 0.0, state[2], state[3], remaining)

        current_jerk, self.acceleration, self.velocity, self.position = state
        return self.velocity, self.acceleration, current_jerk


def parse_vector(value):
    values = np.asarray([float(item) for item in value.split(",")], dtype=float)

    if values.shape != (3,) or not np.isfinite(values).all() or np.any(values <= 0.0):
        raise argparse.ArgumentTypeError("expected three positive comma-separated values")

    return values


def dataset(ulog, name):
    try:
        return next(data.data for data in ulog.data_list if data.name == name and data.multi_id == 0)
    except StopIteration as exc:
        raise RuntimeError(f"required ULog topic is missing: {name}") from exc


def optional_dataset(ulog, name):
    try:
        return dataset(ulog, name)
    except RuntimeError:
        return None


def vector_data(data, fields):
    missing = [field for field in fields if field not in data]

    if missing:
        raise RuntimeError(f"required ULog fields are missing: {', '.join(missing)}")

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


def zero_order_hold(sample_times, source_times, source_values, fill_value=np.nan):
    indexes = np.searchsorted(source_times, sample_times, side="right") - 1
    valid = indexes >= 0
    result = np.full((len(sample_times),) + source_values.shape[1:], fill_value, dtype=float)
    result[valid] = source_values[indexes[valid]]
    return result


def logged_parameter(ulog, name):
    value = ulog.initial_parameters.get(name)

    for _timestamp, changed_name, changed_value in ulog.changed_parameters:
        if changed_name == name:
            value = changed_value

    return value


def card_scalar(ulog, name, minimum=None, maximum=None):
    value = logged_parameter(ulog, name)

    if value is None or not np.isfinite(float(value)):
        raise RuntimeError(f"missing or invalid parameter {name}; provide an explicit CLI override")

    value = float(value)

    if minimum is not None and value < minimum:
        raise RuntimeError(f"parameter {name} is below its valid range")

    if maximum is not None and value > maximum:
        raise RuntimeError(f"parameter {name} is above its valid range")

    return value


def changed_card_parameters(ulog):
    return sorted(
        {
            name
            for _timestamp, name, _value in ulog.changed_parameters
            if name == "MC_RATE_CTRL_T" or name.startswith("MC_AST_")
        }
    )


def card_vector(ulog, prefix):
    values = []

    for suffix in SUFFIXES:
        name = f"{prefix}_{suffix}"
        value = logged_parameter(ulog, name)

        if value is None or not np.isfinite(float(value)) or float(value) <= 0.0:
            raise RuntimeError(f"missing or invalid parameter {name}; provide an explicit CLI override")

        values.append(float(value))

    return np.asarray(values, dtype=float)


def model_disturbance(measured_acceleration, gyro_rate, total_torque, effectiveness, inertia):
    control_gain = effectiveness / inertia
    return measured_acceleration + gyro_rate - control_gain * total_torque


def tracking_residual_disturbance(
    reference_acceleration,
    gyro_rate,
    nominal_torque,
    identified_model_disturbance,
    effectiveness,
    inertia,
):
    control_gain = effectiveness / inertia
    return (
        reference_acceleration
        + gyro_rate
        - control_gain * nominal_torque
        - identified_model_disturbance
    )


def robust_derivative(timestamps, values, window_s=0.1):
    timestamps = np.asarray(timestamps, dtype=float)
    values = np.asarray(values, dtype=float)
    derivative = np.full_like(values, np.nan)
    half_window = 0.5 * window_s

    for index, timestamp in enumerate(timestamps):
        first = np.searchsorted(timestamps, timestamp - half_window, side="left")
        last = np.searchsorted(timestamps, timestamp + half_window, side="right")

        if last - first < 3:
            continue

        local_time = timestamps[first:last] - timestamp
        local_values = values[first:last]
        denominator = float(np.sum(local_time * local_time))

        if denominator > np.finfo(float).eps:
            derivative[index] = np.sum(
                local_time[:, np.newaxis] * local_values,
                axis=0,
            ) / denominator

    return derivative


def simulate_references(timestamps, targets, measured_rates, acceleration_limits, jerk_limits, dt_min, dt_max):
    references = [
        VelocityReference(acceleration_limits[axis], jerk_limits[axis], measured_rates[0, axis])
        for axis in range(3)
    ]
    shaped = np.empty_like(targets)
    acceleration = np.empty_like(targets)
    jerk = np.empty_like(targets)
    valid = np.ones(len(timestamps), dtype=bool)
    shaped[0] = measured_rates[0]
    acceleration[0] = 0.0
    jerk[0] = 0.0

    for index in range(1, len(timestamps)):
        dt = timestamps[index] - timestamps[index - 1]

        if not np.isfinite(dt) or dt < dt_min or dt > dt_max:
            valid[index] = False
            shaped[index] = shaped[index - 1]
            acceleration[index] = acceleration[index - 1]
            jerk[index] = jerk[index - 1]
            continue

        for axis in range(3):
            shaped[index, axis], acceleration[index, axis], jerk[index, axis] = references[axis].update(
                targets[index, axis], dt
            )

    return shaped, acceleration, jerk, valid


def exact_aligned_vector(reference_data, sampled_data, fields):
    reference_indexes, sampled_indexes = exact_timestamp_indexes(
        reference_data[sample_timestamp_field(reference_data)],
        sampled_data["timestamp_sample"],
    )
    values = np.full((len(sampled_data["timestamp_sample"]), len(fields)), np.nan)
    values[sampled_indexes] = vector_data(reference_data, fields)[reference_indexes]
    return values, sampled_indexes


def logged_astsmc_signals(astsmc_status):
    required = {
        "timestamp_sample",
        "dt_valid",
        "configuration_valid",
        "total_bound_valid",
        "reference_reset_count",
        "allocator_feedback_valid",
        "allocator_feedback_stale",
        "allocator_torque_setpoint_achieved",
    }

    for axis in range(3):
        for prefix in (
            "rate_setpoint_shaped",
            "reference_acceleration",
            "reference_jerk",
            "nominal_torque_raw",
            "nominal_torque_limited",
            "residual_torque_raw",
            "residual_torque_limited",
            "variation_weight",
            "variation_regularization",
            "torque_raw",
            "torque_limited",
            "nominal_saturation",
            "internal_saturation",
        ):
            required.add(f"{prefix}[{axis}]")

    missing = sorted(required.difference(astsmc_status))

    if missing:
        raise RuntimeError(f"ASTSMC status fields are missing: {', '.join(missing)}")

    return {
        "timestamps": astsmc_status["timestamp_sample"].astype(float) / 1e6,
        "dt_valid": astsmc_status["dt_valid"].astype(bool),
        "configuration_valid": astsmc_status["configuration_valid"].astype(bool),
        "total_bound_valid": astsmc_status["total_bound_valid"].astype(bool),
        "reference_reset_count": astsmc_status["reference_reset_count"].astype(np.uint32),
        "allocator_feedback_valid": astsmc_status["allocator_feedback_valid"].astype(bool),
        "allocator_feedback_stale": astsmc_status["allocator_feedback_stale"].astype(bool),
        "allocator_torque_setpoint_achieved": astsmc_status[
            "allocator_torque_setpoint_achieved"
        ].astype(bool),
        "shaped": vector_data(
            astsmc_status,
            tuple(f"rate_setpoint_shaped[{axis}]" for axis in range(3)),
        ),
        "acceleration": vector_data(
            astsmc_status,
            tuple(f"reference_acceleration[{axis}]" for axis in range(3)),
        ),
        "jerk": vector_data(
            astsmc_status,
            tuple(f"reference_jerk[{axis}]" for axis in range(3)),
        ),
        "nominal_raw": vector_data(
            astsmc_status,
            tuple(f"nominal_torque_raw[{axis}]" for axis in range(3)),
        ),
        "nominal": vector_data(
            astsmc_status,
            tuple(f"nominal_torque_limited[{axis}]" for axis in range(3)),
        ),
        "residual_raw": vector_data(
            astsmc_status,
            tuple(f"residual_torque_raw[{axis}]" for axis in range(3)),
        ),
        "residual": vector_data(
            astsmc_status,
            tuple(f"residual_torque_limited[{axis}]" for axis in range(3)),
        ),
        "variation_weight": vector_data(
            astsmc_status,
            tuple(f"variation_weight[{axis}]" for axis in range(3)),
        ),
        "variation_regularization": vector_data(
            astsmc_status,
            tuple(f"variation_regularization[{axis}]" for axis in range(3)),
        ),
        "total_raw": vector_data(
            astsmc_status,
            tuple(f"torque_raw[{axis}]" for axis in range(3)),
        ),
        "total": vector_data(
            astsmc_status,
            tuple(f"torque_limited[{axis}]" for axis in range(3)),
        ),
        "nominal_saturation": vector_data(
            astsmc_status,
            tuple(f"nominal_saturation[{axis}]" for axis in range(3)),
        ).astype(bool),
        "residual_saturation": vector_data(
            astsmc_status,
            tuple(f"internal_saturation[{axis}]" for axis in range(3)),
        ).astype(bool),
        "actuator_constraint": (
            vector_data(
                astsmc_status,
                tuple(f"actuator_constraint[{axis}]" for axis in range(3)),
            ).astype(bool)
            if all(f"actuator_constraint[{axis}]" in astsmc_status for axis in range(3))
            else np.zeros((len(astsmc_status["timestamp_sample"]), 3), dtype=bool)
        ),
        "actuator_constraint_available": all(
            f"actuator_constraint[{axis}]" in astsmc_status for axis in range(3)
        ),
    }


def distribution(values, percentile):
    absolute = np.abs(np.asarray(values, dtype=float))
    absolute = absolute[np.isfinite(absolute)]

    if len(absolute) == 0:
        return {"p50": math.nan, "p95": math.nan, "robust": math.nan, "max": math.nan}

    return {
        "p50": float(np.percentile(absolute, 50.0)),
        "p95": float(np.percentile(absolute, 95.0)),
        "robust": float(np.percentile(absolute, percentile)),
        "max": float(np.max(absolute)),
    }


def vector_distribution(values, percentile):
    values = np.asarray(values, dtype=float)

    if values.ndim != 2:
        raise ValueError("vector distribution expects a two-dimensional array")

    finite = np.isfinite(values).all(axis=1)
    return distribution(np.linalg.norm(values[finite], axis=1), percentile)


def sample_accounting(exclusions, retained=None):
    if not exclusions:
        raise ValueError("at least one exclusion mask is required")

    retained = {} if retained is None else retained
    sample_count = len(next(iter(exclusions.values())))
    remaining = np.ones(sample_count, dtype=bool)
    overlapping = {}
    exclusive = {}

    for name, values in exclusions.items():
        mask = np.asarray(values, dtype=bool)

        if len(mask) != sample_count:
            raise ValueError("sample-accounting masks must have equal lengths")

        overlapping[name] = int(np.sum(mask))
        exclusive[name] = int(np.sum(remaining & mask))
        remaining &= ~mask

    retained_counts = {}

    for name, values in retained.items():
        mask = np.asarray(values, dtype=bool)

        if len(mask) != sample_count:
            raise ValueError("sample-accounting masks must have equal lengths")

        retained_counts[name] = int(np.sum(mask))

    return remaining, {
        "population_samples": int(sample_count),
        "overlapping_exclusions": overlapping,
        "exclusive_exclusions": exclusive,
        "retained_diagnostics": retained_counts,
        "operational_samples": int(np.sum(remaining)),
    }


def masked_robust_derivative(timestamps, values, mask, window_s):
    timestamps = np.asarray(timestamps, dtype=float)
    values = np.asarray(values, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    derivative = np.full_like(values, np.nan)
    indexes = np.flatnonzero(mask)

    if len(indexes) == 0:
        return derivative

    breaks = np.flatnonzero(np.diff(indexes) > 1) + 1

    for segment in np.split(indexes, breaks):
        if len(segment) >= 3:
            derivative[segment] = robust_derivative(
                timestamps[segment], values[segment], window_s
            )

    return derivative


def screened_recommendation(authority, total_authority, conditions):
    blockers = [name for name, passed in conditions.items() if not bool(passed)]
    supported = bool(np.isfinite(authority) and authority <= total_authority and not blockers)

    if not np.isfinite(authority):
        blockers.append("screened_authority_finite")
    elif authority > total_authority:
        blockers.append("screened_authority_within_total")

    return {
        "recommendation_supported": supported,
        "blocking_conditions": blockers,
        "recommended_residual_authority": float(authority) if supported else None,
    }


def analyze_log(path, args):
    ulog = ULog(str(path))
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    rates_setpoint = dataset(ulog, "vehicle_rates_setpoint")
    torque_setpoint = dataset(ulog, "vehicle_torque_setpoint")
    vehicle_status = dataset(ulog, "vehicle_status")
    land_detected = dataset(ulog, "vehicle_land_detected")
    astsmc_status = optional_dataset(ulog, "astsmc_status")
    actuator_motors = optional_dataset(ulog, "actuator_motors")

    controller_type = int(card_scalar(ulog, "MC_RATE_CTRL_T"))
    card_acknowledged = int(card_scalar(ulog, "MC_AST_CFG"))
    card_changes = changed_card_parameters(ulog)

    if controller_type != 3 or card_acknowledged != 1:
        raise RuntimeError("log does not contain an acknowledged type-3 controller card")

    if card_changes:
        raise RuntimeError(f"controller card changed in flight: {', '.join(card_changes)}")

    timestamps = sample_timestamps(angular_velocity)
    rates = vector_data(angular_velocity, ("xyz[0]", "xyz[1]", "xyz[2]"))
    logged_acceleration = vector_data(
        angular_velocity,
        ("xyz_derivative[0]", "xyz_derivative[1]", "xyz_derivative[2]"),
    )
    targets = zero_order_hold(
        timestamps,
        sample_timestamps(rates_setpoint),
        vector_data(rates_setpoint, ("roll", "pitch", "yaw")),
    )
    commands = zero_order_hold(
        timestamps,
        sample_timestamps(torque_setpoint),
        vector_data(torque_setpoint, ("xyz[0]", "xyz[1]", "xyz[2]")),
    )
    status_times = sample_timestamps(vehicle_status)
    armed = zero_order_hold(
        timestamps,
        status_times,
        (vehicle_status["arming_state"] == ARMING_STATE_ARMED).astype(float)[:, np.newaxis],
        0.0,
    )[:, 0] > 0.5
    land_times = sample_timestamps(land_detected)
    landed_values = land_detected["landed"].astype(float)[:, np.newaxis]
    landed = zero_order_hold(timestamps, land_times, landed_values, 1.0)[:, 0] > 0.5
    maybe_landed_values = land_detected["maybe_landed"].astype(float)[:, np.newaxis]
    maybe_landed = zero_order_hold(
        timestamps, land_times, maybe_landed_values, 1.0
    )[:, 0] > 0.5

    inertia = args.inertia if args.inertia is not None else card_vector(ulog, "MC_AST_J")
    effectiveness = args.effectiveness if args.effectiveness is not None else card_vector(ulog, "MC_AST_EFF")
    acceleration_limits = (
        args.reference_acceleration
        if args.reference_acceleration is not None
        else card_vector(ulog, "MC_AST_RACC")
    )
    jerk_limits = (
        args.reference_jerk
        if args.reference_jerk is not None
        else card_vector(ulog, "MC_AST_RJERK")
    )
    total_authority = (
        args.total_authority
        if args.total_authority is not None
        else card_vector(ulog, "MC_AST_TMAX")
    )
    k1 = args.k1 if args.k1 is not None else card_vector(ulog, "MC_AST_K1")
    k2 = args.k2 if args.k2 is not None else card_vector(ulog, "MC_AST_K2")
    residual_authority = (
        args.residual_authority
        if args.residual_authority is not None
        else card_vector(ulog, "MC_AST_TRES")
    )
    reference_feedforward = (
        args.reference_feedforward
        if args.reference_feedforward is not None
        else card_scalar(ulog, "MC_AST_RFF", 0.0, 1.0)
    )
    gyro_compensation = (
        args.gyro_compensation
        if args.gyro_compensation is not None
        else card_scalar(ulog, "MC_AST_GYRO", 0.0, 1.0)
    )
    dt_min = args.dt_min if args.dt_min is not None else card_scalar(ulog, "MC_AST_DT_MIN", 0.0)
    dt_max = args.dt_max if args.dt_max is not None else card_scalar(ulog, "MC_AST_DT_MAX", dt_min)
    nominal_authority = total_authority - residual_authority

    if np.any(nominal_authority < 0.0):
        raise RuntimeError("residual authority exceeds total authority")

    finite_base = (
        np.isfinite(rates).all(axis=1)
        & np.isfinite(logged_acceleration).all(axis=1)
        & np.isfinite(targets).all(axis=1)
        & np.isfinite(commands).all(axis=1)
    )
    airborne_finite = armed & ~landed & finite_base

    if np.sum(airborne_finite) < args.min_samples:
        raise RuntimeError(
            f"only {np.sum(airborne_finite)} finite airborne samples; {args.min_samples} required"
        )

    first = np.flatnonzero(airborne_finite)[0]
    start_time = timestamps[first] + args.start
    steady_window = timestamps >= start_time

    if args.end is not None:
        steady_window &= timestamps <= timestamps[first] + args.end

    shaped_reconstructed, acceleration_reconstructed, jerk_reconstructed, reference_valid = simulate_references(
        timestamps,
        targets,
        rates,
        acceleration_limits,
        jerk_limits,
        dt_min,
        dt_max,
    )
    shaped = shaped_reconstructed
    reference_acceleration = acceleration_reconstructed
    reference_jerk = jerk_reconstructed
    gyro_reconstructed = np.cross(rates, rates * inertia)
    nominal_raw = (
        reference_feedforward * reference_acceleration * inertia
        + gyro_compensation * gyro_reconstructed
    ) / effectiveness
    nominal_limited = np.clip(nominal_raw, -nominal_authority, nominal_authority)
    logged_residual_torque = None
    signal_source = "offline_reconstruction"
    reference_disagreement = {"rms": math.nan, "max": math.nan}
    unavailable_diagnostics = []
    status_matched = np.ones(len(timestamps), dtype=bool)
    dt_valid = reference_valid.copy()
    configuration_valid = np.ones(len(timestamps), dtype=bool)
    total_bound_valid = np.ones(len(timestamps), dtype=bool)
    allocator_feedback_valid = np.ones(len(timestamps), dtype=bool)
    allocator_feedback_stale = np.zeros(len(timestamps), dtype=bool)
    allocator_torque_achieved = np.ones(len(timestamps), dtype=bool)
    reset_transition = np.zeros(len(timestamps), dtype=bool)
    nominal_saturation = np.any(np.abs(nominal_raw - nominal_limited) > 1e-6, axis=1)
    residual_saturation = np.zeros(len(timestamps), dtype=bool)
    actuator_constraint = np.zeros(len(timestamps), dtype=bool)
    variation_regularization = np.zeros((len(timestamps), 3))
    variation_regularization_active = np.zeros(len(timestamps), dtype=bool)
    total_command_saturation = np.zeros(len(timestamps), dtype=bool)

    if astsmc_status is not None:
        logged = logged_astsmc_signals(astsmc_status)

        if not logged["actuator_constraint_available"]:
            unavailable_diagnostics.append("actuator_constraint")

        logged_indexes, selected_indexes = exact_timestamp_indexes(
            astsmc_status["timestamp_sample"],
            angular_velocity[sample_timestamp_field(angular_velocity)],
        )
        eligible_status = steady_window & armed & ~landed
        eligible_indexes = np.flatnonzero(eligible_status)
        status_in_window = (
            (astsmc_status["timestamp_sample"] >= angular_velocity[sample_timestamp_field(angular_velocity)][eligible_indexes[0]])
            & (astsmc_status["timestamp_sample"] <= angular_velocity[sample_timestamp_field(angular_velocity)][eligible_indexes[-1]])
        )
        expected_status_samples = int(np.sum(status_in_window))
        matched_eligible = int(np.sum(eligible_status[selected_indexes]))
        match_fraction = (
            matched_eligible / expected_status_samples
            if expected_status_samples > 0
            else 0.0
        )

        if match_fraction < args.min_status_match_fraction:
            raise RuntimeError(
                f"only {match_fraction:.3f} of selected gyro samples match astsmc_status timestamps"
            )

        aligned = {}
        vector_names = (
            "shaped",
            "acceleration",
            "jerk",
            "nominal_raw",
            "nominal",
            "residual_raw",
            "residual",
            "variation_weight",
            "variation_regularization",
            "total_raw",
            "total",
        )

        for name in vector_names:
            values = np.full_like(rates, np.nan)
            values[selected_indexes] = logged[name][logged_indexes]
            aligned[name] = values

        status_matched = np.zeros(len(timestamps), dtype=bool)
        status_matched[selected_indexes] = True

        def align_boolean(name, default=False):
            values = np.full(len(timestamps), default, dtype=bool)
            values[selected_indexes] = logged[name][logged_indexes]
            return values

        dt_valid = align_boolean("dt_valid")
        configuration_valid = align_boolean("configuration_valid")
        total_bound_valid = align_boolean("total_bound_valid")
        allocator_feedback_valid = align_boolean("allocator_feedback_valid")
        allocator_feedback_stale = align_boolean("allocator_feedback_stale", True)
        allocator_torque_achieved = align_boolean("allocator_torque_setpoint_achieved")
        aligned_nominal_saturation = np.zeros((len(timestamps), 3), dtype=bool)
        aligned_nominal_saturation[selected_indexes] = logged["nominal_saturation"][logged_indexes]
        nominal_saturation = np.any(aligned_nominal_saturation, axis=1)
        aligned_residual_saturation = np.zeros((len(timestamps), 3), dtype=bool)
        aligned_residual_saturation[selected_indexes] = logged["residual_saturation"][logged_indexes]
        residual_saturation = np.any(aligned_residual_saturation, axis=1)
        aligned_actuator_constraint = np.zeros((len(timestamps), 3), dtype=bool)
        aligned_actuator_constraint[selected_indexes] = logged["actuator_constraint"][logged_indexes]
        actuator_constraint = np.any(aligned_actuator_constraint, axis=1)
        variation_regularization = aligned["variation_regularization"]
        variation_regularization_active = np.any(
            np.abs(variation_regularization) > 1e-8,
            axis=1,
        )
        hard_total_projection = (
            np.abs(aligned["total_raw"] - aligned["total"]) > 1e-6
        ) & ~(
            np.abs(aligned["variation_regularization"]) > 1e-8
        )
        total_command_saturation = np.any(
            hard_total_projection,
            axis=1,
        ) | ~total_bound_valid
        reset_events = np.zeros(len(logged["reference_reset_count"]), dtype=bool)
        reset_events[1:] = np.diff(
            logged["reference_reset_count"].astype(np.int64)
        ) != 0
        reset_transition[selected_indexes] = reset_events[logged_indexes]
        reference_error = aligned["shaped"] - shaped_reconstructed
        finite_reference_error = (
            np.isfinite(reference_error).all(axis=1) & status_matched & dt_valid
        )

        if np.any(finite_reference_error):
            reference_disagreement = {
                "rms": float(np.sqrt(np.mean(reference_error[finite_reference_error] ** 2))),
                "max": float(np.max(np.abs(reference_error[finite_reference_error]))),
            }

        if reference_disagreement["max"] > args.max_reference_disagreement:
            raise RuntimeError(
                "logged and reconstructed shaped references disagree by "
                f"{reference_disagreement['max']:.6f} rad/s"
            )

        shaped = aligned["shaped"]
        reference_acceleration = aligned["acceleration"]
        reference_jerk = aligned["jerk"]
        nominal_raw = aligned["nominal_raw"]
        nominal_limited = aligned["nominal"]
        logged_residual_torque = aligned["residual"]
        commands = aligned["total"]
        signal_source = "astsmc_status"

    elif not args.allow_reconstruction:
        raise RuntimeError("astsmc_status is required unless --allow-reconstruction is set")

    gyro = np.cross(rates, rates * inertia)
    identified_model_disturbance = model_disturbance(
        logged_acceleration,
        gyro / inertia,
        commands,
        effectiveness,
        inertia,
    )
    residual_disturbance = tracking_residual_disturbance(
        reference_acceleration,
        gyro / inertia,
        nominal_limited,
        identified_model_disturbance,
        effectiveness,
        inertia,
    )
    inferred_residual_torque = (
        logged_residual_torque
        if logged_residual_torque is not None
        else commands - nominal_limited
    )
    finite_signals = (
        finite_base
        & status_matched
        & np.isfinite(reference_acceleration).all(axis=1)
        & np.isfinite(reference_jerk).all(axis=1)
        & np.isfinite(nominal_raw).all(axis=1)
        & np.isfinite(nominal_limited).all(axis=1)
        & np.isfinite(commands).all(axis=1)
        & np.isfinite(inferred_residual_torque).all(axis=1)
        & np.isfinite(residual_disturbance).all(axis=1)
    )
    saturated_motors = np.zeros(len(timestamps), dtype=bool)
    finite_motors = np.zeros(len(timestamps), dtype=bool)

    if actuator_motors is not None:
        motor_fields = tuple(
            field
            for index in range(12)
            if (field := f"control[{index}]") in actuator_motors
            and np.isfinite(actuator_motors[field]).any()
        )

        if motor_fields:
            motor_commands = zero_order_hold(
                timestamps,
                sample_timestamps(actuator_motors),
                vector_data(actuator_motors, motor_fields),
            )
            finite_motors = np.isfinite(motor_commands).all(axis=1)
            saturated_motors = finite_motors & np.any(
                (motor_commands <= args.motor_margin)
                | (motor_commands >= 1.0 - args.motor_margin),
                axis=1,
            )

    exclusions_before_derivative = {
        "disarmed": ~armed,
        "confirmed_landed": landed,
        "outside_steady_window": ~steady_window,
        "missing_or_nonfinite_alignment": ~finite_signals,
        "invalid_timing_or_held_update": ~dt_valid,
        "configuration_invalid": ~configuration_valid,
        "total_bound_invalid": ~total_bound_valid,
        "reference_reset_transition": reset_transition,
        "allocator_feedback_invalid": ~allocator_feedback_valid,
        "allocator_feedback_stale": allocator_feedback_stale,
        "allocator_torque_unachieved": ~allocator_torque_achieved,
        "nominal_saturation": nominal_saturation,
        "residual_saturation": residual_saturation,
        "actuator_constraint": actuator_constraint,
        "total_command_saturation": total_command_saturation,
        "motor_saturation": saturated_motors,
    }
    preliminary_mask, _ = sample_accounting(exclusions_before_derivative)
    derivative_population = preliminary_mask | (
        variation_regularization_active
        & armed
        & ~landed
        & steady_window
        & finite_signals
        & dt_valid
        & configuration_valid
        & total_bound_valid
        & ~reset_transition
        & allocator_feedback_valid
        & ~allocator_feedback_stale
        & allocator_torque_achieved
        & ~nominal_saturation
        & ~residual_saturation
        & ~actuator_constraint
        & ~total_command_saturation
        & ~saturated_motors
    )
    residual_derivative = masked_robust_derivative(
        timestamps,
        residual_disturbance,
        derivative_population,
        args.derivative_window,
    )
    derivative_unavailable = ~np.isfinite(residual_derivative).all(axis=1)
    exclusions = dict(exclusions_before_derivative)
    exclusions["derivative_unavailable"] = derivative_unavailable
    operational, accounting = sample_accounting(
        exclusions,
        retained={
            "maybe_landed": maybe_landed & ~landed,
            "variation_regularized": variation_regularization_active,
            "finite_motor_samples": finite_motors,
        },
    )

    if np.sum(operational) < args.min_samples:
        raise RuntimeError(
            f"only {np.sum(operational)} operational controller samples; {args.min_samples} required"
        )

    adverse = armed & ~landed & steady_window & finite_signals & ~operational
    adverse_envelopes = {
        "samples": int(np.sum(adverse)),
        "axes": {
            axis_name: {
                "residual_disturbance": distribution(
                    residual_disturbance[adverse, axis], args.percentile
                ),
                "inferred_residual_torque": distribution(
                    inferred_residual_torque[adverse, axis], args.percentile
                ),
                "nominal_torque": distribution(
                    nominal_limited[adverse, axis], args.percentile
                ),
                "total_torque": distribution(commands[adverse, axis], args.percentile),
            }
            for axis, axis_name in enumerate(AXES)
        },
        "vector_norms": {
            "residual_disturbance": vector_distribution(
                residual_disturbance[adverse], args.percentile
            ),
            "inferred_residual_torque": vector_distribution(
                inferred_residual_torque[adverse], args.percentile
            ),
            "nominal_torque": vector_distribution(
                nominal_limited[adverse], args.percentile
            ),
            "total_torque": vector_distribution(commands[adverse], args.percentile),
        },
    }
    motor_population = armed & ~landed & steady_window & finite_motors
    motor_saturation_fraction = (
        float(np.mean(saturated_motors[motor_population]))
        if np.any(motor_population)
        else math.nan
    )
    timestamp_bound = dt_max
    axes = {}

    for axis, axis_name in enumerate(AXES):
        disturbance_stats = distribution(
            residual_disturbance[operational, axis], args.percentile
        )
        derivative_stats = distribution(
            residual_derivative[operational, axis], args.percentile
        )
        inferred_torque_stats = distribution(
            inferred_residual_torque[operational, axis], args.percentile
        )
        nominal_torque_stats = distribution(
            nominal_limited[operational, axis], args.percentile
        )
        total_torque_stats = distribution(commands[operational, axis], args.percentile)
        disturbance_bound = disturbance_stats["robust"] * args.safety_margin
        derivative_bound = derivative_stats["robust"] * args.safety_margin
        authority_from_disturbance = (
            disturbance_bound + k2[axis] * timestamp_bound
        ) / (effectiveness[axis] / inertia[axis])
        screened_authority = max(
            inferred_torque_stats["robust"] * args.safety_margin,
            authority_from_disturbance,
        )
        available_reaching = effectiveness[axis] / inertia[axis] * residual_authority[axis]
        denominator = available_reaching - disturbance_bound - k2[axis] * timestamp_bound
        required_k1 = (
            math.sqrt(
                2.0
                * k2[axis]
                * (available_reaching + disturbance_bound)
                / denominator
            )
            if denominator > 0.0
            else math.inf
        )
        nominal_acceleration_demand = (
            reference_feedforward
            * inertia[axis]
            / effectiveness[axis]
            * acceleration_limits[axis]
        )
        observed_gyro_demand = float(
            np.max(np.abs(gyro[operational, axis] / effectiveness[axis]))
        )
        conditions = {
            "k2_gt_L": bool(k2[axis] > derivative_bound),
            "authority_gt_W_plus_k2_dt": bool(
                available_reaching > disturbance_bound + k2[axis] * timestamp_bound
            ),
            "k1_sufficient": bool(k1[axis] > required_k1),
            "nominal_full_envelope_fits": bool(
                nominal_acceleration_demand + observed_gyro_demand
                <= nominal_authority[axis]
            ),
            "no_observed_saturation": bool(
                accounting["overlapping_exclusions"]["nominal_saturation"] == 0
                and accounting["overlapping_exclusions"]["residual_saturation"] == 0
                and accounting["overlapping_exclusions"]["actuator_constraint"] == 0
                and accounting["overlapping_exclusions"]["total_command_saturation"] == 0
                and accounting["overlapping_exclusions"]["motor_saturation"] == 0
            ),
            "required_diagnostics_available": not unavailable_diagnostics,
        }
        recommendation = screened_recommendation(
            screened_authority,
            total_authority[axis],
            conditions,
        )
        axes[axis_name] = {
            "disturbance": disturbance_stats,
            "disturbance_derivative": derivative_stats,
            "inferred_residual_torque": inferred_torque_stats,
            "nominal_torque": nominal_torque_stats,
            "total_torque": total_torque_stats,
            "bound_W": float(disturbance_bound),
            "bound_L": float(derivative_bound),
            "configured_residual_authority": float(residual_authority[axis]),
            "screened_residual_authority": float(screened_authority),
            "recommendation": recommendation,
            "nominal_saturation_fraction": float(
                np.mean(
                    np.abs(nominal_raw[operational, axis] - nominal_limited[operational, axis])
                    > 1e-6
                )
            ),
            "residual_saturation_fraction": float(
                np.mean(
                    np.abs(inferred_residual_torque[operational, axis])
                    >= residual_authority[axis] - 1e-6
                )
            ),
            "authority_feasibility": {
                "nominal_acceleration_demand": float(nominal_acceleration_demand),
                "available_nominal_authority": float(nominal_authority[axis]),
                "observed_gyro_demand_max": observed_gyro_demand,
                "residual_reaching_authority": float(available_reaching),
            },
            "conditions": {
                **conditions,
                "k1_required": float(required_k1),
            },
        }

    vector_aggregates = {
        "residual_disturbance": vector_distribution(
            residual_disturbance[operational], args.percentile
        ),
        "residual_disturbance_derivative": vector_distribution(
            residual_derivative[operational], args.percentile
        ),
        "inferred_residual_torque": vector_distribution(
            inferred_residual_torque[operational], args.percentile
        ),
        "nominal_torque": vector_distribution(
            nominal_limited[operational], args.percentile
        ),
        "total_torque": vector_distribution(commands[operational], args.percentile),
    }
    selected_timestamps = timestamps[operational]
    return {
        "log": str(path),
        "samples": int(np.sum(operational)),
        "duration_s": float(selected_timestamps[-1] - selected_timestamps[0]),
        "population": {
            "definition": (
                "finite, armed, confirmed-airborne, steady-window, valid-timing/card/bound/allocator, "
                "non-reset, unsaturated samples"
            ),
            **accounting,
            "adverse_envelopes": adverse_envelopes,
        },
        "quality": {
            "signal_source": signal_source,
            "reference_disagreement": reference_disagreement,
            "motor_saturation_fraction": motor_saturation_fraction,
            "variation_regularization": {
                "active_samples": int(np.sum(variation_regularization_active)),
                "active_fraction": float(np.mean(variation_regularization_active)),
                "axes": {
                    axis_name: distribution(
                        variation_regularization[:, axis], args.percentile
                    )
                    for axis, axis_name in enumerate(AXES)
                },
                "vector_norm": vector_distribution(
                    variation_regularization, args.percentile
                ),
                "population_treatment": (
                    "retained in the operational population and reported explicitly; "
                    "not classified as hard saturation"
                ),
            },
            "card_changes": card_changes,
            "unavailable_diagnostics": unavailable_diagnostics,
        },
        "card": {
            "inertia": inertia.tolist(),
            "effectiveness": effectiveness.tolist(),
            "reference_acceleration": acceleration_limits.tolist(),
            "reference_jerk": jerk_limits.tolist(),
            "total_authority": total_authority.tolist(),
            "residual_authority": residual_authority.tolist(),
            "k1": k1.tolist(),
            "k2": k2.tolist(),
            "reference_feedforward": float(reference_feedforward),
            "gyro_compensation": float(gyro_compensation),
            "dt_min": float(dt_min),
            "dt_max": float(dt_max),
        },
        "axes": axes,
        "vector_aggregates": vector_aggregates,
    }


def aggregate(results, percentile, safety_margin):
    axes = {}

    for axis in AXES:
        screened = [result["axes"][axis]["screened_residual_authority"] for result in results]
        disturbance = [result["axes"][axis]["bound_W"] for result in results]
        derivative = [result["axes"][axis]["bound_L"] for result in results]
        supported = all(
            result["axes"][axis]["recommendation"]["recommendation_supported"]
            for result in results
        )
        blockers = sorted(
            {
                blocker
                for result in results
                for blocker in result["axes"][axis]["recommendation"]["blocking_conditions"]
            }
        )
        screened_index = int(np.argmax(screened))
        disturbance_index = int(np.argmax(disturbance))
        derivative_index = int(np.argmax(derivative))
        axes[axis] = {
            "recommendation_supported": bool(supported),
            "blocking_conditions": blockers,
            "screened_residual_authority": float(max(screened)),
            "recommended_residual_authority": float(max(screened)) if supported else None,
            "screened_residual_authority_log": results[screened_index]["log"],
            "maximum_bound_W": float(max(disturbance)),
            "maximum_bound_W_log": results[disturbance_index]["log"],
            "maximum_bound_L": float(max(derivative)),
            "maximum_bound_L_log": results[derivative_index]["log"],
        }

    return {
        "method": {
            "name": "empirically identified operational bound",
            "percentile": float(percentile),
            "safety_margin": float(safety_margin),
            "note": (
                "Sampled operational screening only; not a global disturbance bound, "
                "formal stability proof, or real-flight authorization."
            ),
        },
        "logs": results,
        "aggregate": {
            "recommendation_supported": bool(
                all(metrics["recommendation_supported"] for metrics in axes.values())
            ),
            "axes": axes,
        },
    }


def print_report(report):
    method = report["method"]
    print(
        f"{method['name']}: percentile={method['percentile']:.3f} "
        f"margin={method['safety_margin']:.3f}"
    )

    for result in report["logs"]:
        quality = result["quality"]
        population = result["population"]
        print(
            f"log={result['log']} samples={result['samples']}/{population['population_samples']} "
            f"duration={result['duration_s']:.3f}s source={quality['signal_source']} "
            f"reference_max={quality['reference_disagreement']['max']:.6f} "
            f"motor_sat={quality['motor_saturation_fraction']:.6f} "
            f"variation_regularized="
            f"{quality['variation_regularization']['active_fraction']:.6f}"
        )
        exclusions = ", ".join(
            f"{name}={count}"
            for name, count in population["exclusive_exclusions"].items()
            if count
        )
        print(f"  exclusions: {exclusions or 'none'}")

        for axis in AXES:
            metrics = result["axes"][axis]
            conditions = metrics["conditions"]
            feasibility = metrics["authority_feasibility"]
            recommendation = metrics["recommendation"]
            print(
                f"  {axis:5s} W={metrics['bound_W']:.4f} L={metrics['bound_L']:.4f} "
                f"T_res={metrics['configured_residual_authority']:.4f} "
                f"screened={metrics['screened_residual_authority']:.4f} "
                f"supported={recommendation['recommendation_supported']} "
                f"nominal_demand={feasibility['nominal_acceleration_demand']:.4f}/"
                f"{feasibility['available_nominal_authority']:.4f} "
                f"K2>L={conditions['k2_gt_L']} authority={conditions['authority_gt_W_plus_k2_dt']} "
                f"K1>{conditions['k1_required']:.4f}={conditions['k1_sufficient']}"
            )

    print(
        "aggregate recommendation: "
        f"supported={report['aggregate']['recommendation_supported']}"
    )

    for axis in AXES:
        metrics = report["aggregate"]["axes"][axis]
        recommendation = metrics["recommended_residual_authority"]
        value = f"{recommendation:.4f}" if recommendation is not None else "none"
        print(
            f"  {axis}: recommended_residual_authority={value} "
            f"screened={metrics['screened_residual_authority']:.4f} "
            f"blockers={','.join(metrics['blocking_conditions']) or 'none'}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--inertia", type=parse_vector)
    parser.add_argument("--effectiveness", type=parse_vector)
    parser.add_argument("--reference-acceleration", type=parse_vector)
    parser.add_argument("--reference-jerk", type=parse_vector)
    parser.add_argument("--total-authority", type=parse_vector)
    parser.add_argument("--residual-authority", type=parse_vector)
    parser.add_argument("--k1", type=parse_vector)
    parser.add_argument("--k2", type=parse_vector)
    parser.add_argument("--reference-feedforward", type=float)
    parser.add_argument("--gyro-compensation", type=float)
    parser.add_argument("--dt-min", type=float)
    parser.add_argument("--dt-max", type=float)
    parser.add_argument("--allow-reconstruction", action="store_true")
    parser.add_argument("--min-status-match-fraction", type=float, default=0.99)
    parser.add_argument("--max-reference-disagreement", type=float, default=0.03)
    parser.add_argument("--motor-margin", type=float, default=0.01)
    parser.add_argument("--max-motor-saturation-fraction", type=float, default=0.0)
    parser.add_argument("--derivative-window", type=float, default=0.1)
    parser.add_argument("--percentile", type=float, default=99.5)
    parser.add_argument("--safety-margin", type=float, default=1.25)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float)
    parser.add_argument("--min-samples", type=int, default=500)
    parser.add_argument("--json", type=Path, help="write the full machine-readable report")
    args = parser.parse_args()

    if (
        not 50.0 <= args.percentile < 100.0
        or args.safety_margin < 1.0
        or (
            args.reference_feedforward is not None
            and not 0.0 <= args.reference_feedforward <= 1.0
        )
        or (
            args.gyro_compensation is not None
            and not 0.0 <= args.gyro_compensation <= 1.0
        )
        or (args.dt_min is not None and args.dt_min <= 0.0)
        or (args.dt_max is not None and args.dt_max <= 0.0)
        or (
            args.dt_min is not None
            and args.dt_max is not None
            and args.dt_min > args.dt_max
        )
        or not 0.0 < args.min_status_match_fraction <= 1.0
        or args.max_reference_disagreement < 0.0
        or not 0.0 <= args.motor_margin < 0.5
        or not 0.0 <= args.max_motor_saturation_fraction <= 1.0
        or args.derivative_window <= 0.0
        or args.min_samples < 2
    ):
        raise ValueError("invalid residual-bound analysis option")

    results = [analyze_log(path, args) for path in args.logs]
    report = aggregate(results, args.percentile, args.safety_margin)
    print_report(report)

    if args.json is not None:
        args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
