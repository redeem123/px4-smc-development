#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import subprocess
import sys
from argparse import Namespace
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "Tools"
sys.path.insert(0, str(TOOLS_DIR))

from analyze_smc_flight import analyze_log as analyze_airborne_log
from analyze_smc_flight import check_result as check_airborne_result
from analyze_offboard_square import AXES, analyze_log, print_result
from run_uav985_trajectory_grid import ACCEPTANCE_LOG_PROFILE, GridConfig, run_one


DEFAULT_INERTIA = (0.040461, 0.035366, 0.050951)
DEFAULT_EFFECTIVENESS = (6.978, 6.978, 1.163)
DEFAULT_TORQUE_LIMITS = {
    1: (0.20, 0.20, 0.10),
    2: (0.20, 0.20, 0.15),
    3: (0.20, 0.20, 0.15),
}
METRIC_REGISTRY = {
    "track_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.05,
        "absolute_margin": 0.002,
    },
    "track_p95": {
        "direction": "lower",
        "class": "non_inferiority",
        "relative_margin": 0.05,
        "absolute_margin": 0.003,
    },
    "track_max": {
        "direction": "lower",
        "class": "non_inferiority",
        "relative_margin": 0.05,
        "absolute_margin": 0.005,
    },
    "roll_rate_error_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.10,
        "absolute_margin": 0.01,
    },
    "pitch_rate_error_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.10,
        "absolute_margin": 0.01,
    },
    "roll_rate_osc_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.01,
    },
    "pitch_rate_osc_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.01,
    },
    "roll_torque_hf_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.002,
    },
    "pitch_torque_hf_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.002,
    },
    "roll_torque_variation_per_s": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "pitch_torque_variation_per_s": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "terminal_roll_rate_error_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "terminal_pitch_rate_error_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "terminal_roll_oscillation_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "terminal_pitch_oscillation_rms": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.15,
        "absolute_margin": 0.005,
    },
    "yaw_final_error_max_deg": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.05,
        "absolute_margin": 0.25,
    },
    "yaw_settling_max_s": {
        "direction": "lower",
        "class": "primary",
        "relative_margin": 0.05,
        "absolute_margin": 0.10,
    },
    "unallocated_torque_max": {
        "direction": "lower",
        "class": "invariant",
        "relative_margin": 0.0,
        "absolute_margin": 0.0,
    },
}
DEFAULT_METRIC_SPEC = {
    "direction": "lower",
    "class": "descriptive",
    "relative_margin": 0.0,
    "absolute_margin": 0.0,
}


def parse_vector(value):
    values = tuple(float(item) for item in value.split(","))

    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")

    return values


def parse_campaign_log(value):
    parts = value.split(",", 2)

    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "campaign log must be round,controller,path"
        )

    round_index = int(parts[0]) - 1
    controller = int(parts[1])

    if round_index < 0:
        raise argparse.ArgumentTypeError("campaign-log round is one-based")

    if controller not in (1, 2, 3):
        raise argparse.ArgumentTypeError("campaign-log controller must be 1, 2, or 3")

    return round_index, controller, Path(parts[2])


def looks_like_local_udp(connection):
    return connection.startswith("udp:") and (
        "127.0.0.1" in connection or "localhost" in connection
    )


def torque_limits_for_controller(args, controller=None):
    controller = args.controller if controller is None else controller
    return args.expected_torque_limit or DEFAULT_TORQUE_LIMITS[controller]


def expected_controller_parameters(args, controller=None):
    controller = args.controller if controller is None else controller
    torque_limits = torque_limits_for_controller(args, controller)
    expected_parameters = {
        "MC_RATE_CTRL_T": controller,
        "MIS_TKO_ALT_MAX": 1.0,
    }

    if controller == 1:
        expected_parameters.update(
            {
                "MC_MPC_J_R": args.expected_inertia[0],
                "MC_MPC_J_P": args.expected_inertia[1],
                "MC_MPC_J_Y": args.expected_inertia[2],
                "MC_MPC_EFF_R": args.expected_effectiveness[0],
                "MC_MPC_EFF_P": args.expected_effectiveness[1],
                "MC_MPC_EFF_Y": args.expected_effectiveness[2],
                "MC_MPC_Q_R": args.expected_mpc_q[0],
                "MC_MPC_Q_P": args.expected_mpc_q[1],
                "MC_MPC_Q_Y": args.expected_mpc_q[2],
                "MC_MPC_R_R": args.expected_mpc_r[0],
                "MC_MPC_R_P": args.expected_mpc_r[1],
                "MC_MPC_R_Y": args.expected_mpc_r[2],
                "MC_MPC_DU_R": args.expected_mpc_du[0],
                "MC_MPC_DU_P": args.expected_mpc_du[1],
                "MC_MPC_DU_Y": args.expected_mpc_du[2],
                "MC_MPC_TAU": args.expected_actuator_tau,
                "MC_MPC_HORIZON": args.expected_horizon,
                "MC_MPC_SLEW": args.expected_mpc_slew,
                "MC_MPC_GYRO": args.expected_mpc_gyro,
                "MC_MPC_TMAX_R": torque_limits[0],
                "MC_MPC_TMAX_P": torque_limits[1],
                "MC_MPC_TMAX_Y": torque_limits[2],
            }
        )

    elif controller == 2:
        expected_parameters.update(
            {
                "MC_MSMC_CFG": 1,
                "MC_MSMC_J_R": args.expected_inertia[0],
                "MC_MSMC_J_P": args.expected_inertia[1],
                "MC_MSMC_J_Y": args.expected_inertia[2],
                "MC_MSMC_EFF_R": args.expected_effectiveness[0],
                "MC_MSMC_EFF_P": args.expected_effectiveness[1],
                "MC_MSMC_EFF_Y": args.expected_effectiveness[2],
                "MC_MSMC_C_R": args.expected_smc_c[0],
                "MC_MSMC_C_P": args.expected_smc_c[1],
                "MC_MSMC_C_Y": args.expected_smc_c[2],
                "MC_MSMC_ETA_R": args.expected_smc_eta[0],
                "MC_MSMC_ETA_P": args.expected_smc_eta[1],
                "MC_MSMC_ETA_Y": args.expected_smc_eta[2],
                "MC_MSMC_BND_R": args.expected_smc_boundary[0],
                "MC_MSMC_BND_P": args.expected_smc_boundary[1],
                "MC_MSMC_BND_Y": args.expected_smc_boundary[2],
                "MC_MSMC_KS_R": args.expected_smc_ks[0],
                "MC_MSMC_KS_P": args.expected_smc_ks[1],
                "MC_MSMC_KS_Y": args.expected_smc_ks[2],
                "MC_MSMC_RSPD_L": args.expected_smc_rate_sp_derivative,
                "MC_SMC_LPF": args.expected_smc_lpf,
                "MC_SMC_SLEW": args.expected_smc_slew,
                "MC_MSMC_ILIM_R": args.expected_smc_integral_limit[0],
                "MC_MSMC_ILIM_P": args.expected_smc_integral_limit[1],
                "MC_MSMC_ILIM_Y": args.expected_smc_integral_limit[2],
                "MC_MSMC_TMAX_R": torque_limits[0],
                "MC_MSMC_TMAX_P": torque_limits[1],
                "MC_MSMC_TMAX_Y": torque_limits[2],
            }
        )

    elif controller == 3:
        expected_parameters.update(
            {
                "MC_AST_CFG": 1,
                "MC_BAT_SCALE_EN": 0,
                "MC_AST_J_R": args.expected_inertia[0],
                "MC_AST_J_P": args.expected_inertia[1],
                "MC_AST_J_Y": args.expected_inertia[2],
                "MC_AST_EFF_R": args.expected_effectiveness[0],
                "MC_AST_EFF_P": args.expected_effectiveness[1],
                "MC_AST_EFF_Y": args.expected_effectiveness[2],
                "MC_AST_K1_R": args.expected_ast_k1[0],
                "MC_AST_K1_P": args.expected_ast_k1[1],
                "MC_AST_K1_Y": args.expected_ast_k1[2],
                "MC_AST_K2_R": args.expected_ast_k2[0],
                "MC_AST_K2_P": args.expected_ast_k2[1],
                "MC_AST_K2_Y": args.expected_ast_k2[2],
                "MC_AST_TMAX_R": torque_limits[0],
                "MC_AST_TMAX_P": torque_limits[1],
                "MC_AST_TMAX_Y": torque_limits[2],
                "MC_AST_RACC_R": args.expected_ast_reference_acceleration[0],
                "MC_AST_RACC_P": args.expected_ast_reference_acceleration[1],
                "MC_AST_RACC_Y": args.expected_ast_reference_acceleration[2],
                "MC_AST_RJERK_R": args.expected_ast_reference_jerk[0],
                "MC_AST_RJERK_P": args.expected_ast_reference_jerk[1],
                "MC_AST_RJERK_Y": args.expected_ast_reference_jerk[2],
                "MC_AST_TRES_R": args.expected_ast_residual_torque[0],
                "MC_AST_TRES_P": args.expected_ast_residual_torque[1],
                "MC_AST_TRES_Y": args.expected_ast_residual_torque[2],
                "MC_AST_TAU_R": args.expected_ast_actuator_tau[0],
                "MC_AST_TAU_P": args.expected_ast_actuator_tau[1],
                "MC_AST_TAU_Y": args.expected_ast_actuator_tau[2],
                "MC_AST_SLEW_R": args.expected_ast_slew[0],
                "MC_AST_SLEW_P": args.expected_ast_slew[1],
                "MC_AST_SLEW_Y": args.expected_ast_slew[2],
                "MC_AST_DU_R": args.expected_ast_variation_weight[0],
                "MC_AST_DU_P": args.expected_ast_variation_weight[1],
                "MC_AST_DU_Y": args.expected_ast_variation_weight[2],
                "MC_AST_SBD_R": args.expected_ast_sliding_boundary[0],
                "MC_AST_SBD_P": args.expected_ast_sliding_boundary[1],
                "MC_AST_SBD_Y": args.expected_ast_sliding_boundary[2],
                "MC_AST_TRK_B": args.expected_ast_tracking_blend,
                "MC_AST_RFF": args.expected_ast_reference_feedforward,
                "MC_AST_RFF_RP": args.expected_ast_reference_feedforward_rp,
                "MC_AST_GYRO": args.expected_ast_gyro,
                "MC_AST_DT_MIN": args.expected_ast_dt_min,
                "MC_AST_DT_MAX": args.expected_ast_dt_max,
                "MC_AST_REC_ERR": args.expected_ast_recovery_error,
            }
        )

    else:
        raise ValueError(f"unsupported controller {controller}")

    return expected_parameters


def comparison_schedule(primary_controller, compare_controller, repeat):
    schedule = []

    for round_index in range(repeat):
        controllers = [primary_controller, compare_controller]

        if round_index % 2 == 1:
            controllers.reverse()

        schedule.extend((round_index, controller) for controller in controllers)

    return schedule


def evaluate_numeric_checks(checks):
    failures = []

    for label, value, operator, threshold in checks:
        finite = math.isfinite(float(value))
        passed = finite and (value <= threshold if operator == "<=" else value >= threshold)
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}: {value:.5f} {operator} {threshold:.5f}")

        if not passed:
            failures.append(label)

    return failures


def evaluate_boolean_checks(checks):
    failures = []

    for label, passed in checks:
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}")

        if not passed:
            failures.append(label)

    return failures


def astsmc_threshold_checks(result, args):
    ast = result["astsmc_diagnostics"]

    if ast is None:
        return [], [("ASTSMC diagnostics present", False)]

    numeric_checks = [
        ("ASTSMC diagnostic samples", ast["airborne_samples"], ">=", args.min_ast_samples),
        ("ASTSMC diagnostic rate", ast["sample_rate_hz"], ">=", args.min_ast_rate_hz),
        (
            "ASTSMC invalid-dt fraction",
            ast["dt_invalid_fraction"],
            "<=",
            args.max_ast_invalid_dt_fraction,
        ),
        (
            "ASTSMC allocator feedback-valid fraction",
            ast["allocator"].get("feedback_valid_fraction", math.nan),
            ">=",
            args.min_ast_allocator_feedback_valid_fraction,
        ),
        (
            "ASTSMC stale allocator-feedback fraction",
            ast["allocator"].get("stale_fraction", math.nan),
            "<=",
            args.max_ast_allocator_stale_fraction,
        ),
        (
            "ASTSMC allocator residual norm",
            ast["allocator"].get("residual_norm", {}).get("max", math.nan),
            "<=",
            args.max_ast_allocation_residual,
        ),
        (
            "ASTSMC torque timestamp match fraction",
            ast["torque_timestamp_match_fraction"],
            ">=",
            args.min_ast_torque_timestamp_match_fraction,
        ),
        (
            "ASTSMC consecutive invalid-dt holds",
            ast["dt_invalid_consecutive_count"],
            "<=",
            0,
        ),
        (
            "ASTSMC invalid-dt yaw-transition clusters",
            ast["dt_invalid_transition_cluster_count"],
            "<=",
            0,
        ),
        (
            "ASTSMC total-bound invariant violations",
            ast["total_bound_violation_count_delta"],
            "<=",
            0,
        ),
        (
            "ASTSMC state-recovery interventions",
            ast["state_recovery_count_delta"],
            "<=",
            0,
        ),
        (
            "ASTSMC ground-containment interventions",
            ast["ground_containment_count_delta"],
            "<=",
            0,
        ),
        (
            "ASTSMC wrong-direction large-error escapes",
            ast["wrong_direction_escape_count"],
            "<=",
            0,
        ),
    ]

    for axis in AXES:
        diagnostics = ast["axes"].get(axis, {})
        numeric_checks.extend(
            [
                (
                    f"{axis} ASTSMC internal saturation",
                    diagnostics.get("internal_saturation_fraction", math.nan),
                    "<=",
                    args.max_ast_internal_saturation_fraction,
                ),
                (
                    f"{axis} ASTSMC torque bound excess",
                    diagnostics.get("torque_bound_max_excess", math.nan),
                    "<=",
                    args.torque_limit_tolerance,
                ),
                (
                    f"{axis} ASTSMC residual bound excess",
                    diagnostics.get("residual_bound_max_excess", math.nan),
                    "<=",
                    args.torque_limit_tolerance,
                ),
                (
                    f"{axis} ASTSMC torque decomposition error",
                    diagnostics.get("decomposition_error", {}).get("max", math.nan),
                    "<=",
                    args.torque_limit_tolerance,
                ),
                (
                    f"{axis} ASTSMC published command mismatch",
                    ast["command_mismatch"].get(axis, {}).get("max", math.nan),
                    "<=",
                    args.torque_limit_tolerance,
                ),
            ]
        )

    boolean_checks = [
        ("ASTSMC schema complete", not ast["missing_fields"]),
        ("ASTSMC runtime identity and configuration valid", ast["runtime_valid"]),
        ("ASTSMC recovery threshold valid", ast["recovery_threshold_valid"]),
        ("ASTSMC runtime fault clear", not ast["runtime_fault_latched"]),
        ("ASTSMC ground containment inactive", ast["ground_containment_active_count"] == 0),
        ("ASTSMC diagnostics finite", ast["nonfinite_rows"] == 0),
        ("ASTSMC timing bounds valid and constant", ast["timing_bounds_consistent"]),
        ("ASTSMC accepted dt matches validity and bounds", ast["accepted_dt_consistent"]),
        ("ASTSMC counters monotonic or wrapped", ast["counter_reset_count"] == 0),
        ("ASTSMC sample timestamps monotonic", ast["timestamps_monotonic"]),
        ("ASTSMC publication timestamps valid", ast["publication_before_sample_count"] == 0),
        ("ASTSMC active gains match logged card", ast["gains_match_parameters"]),
        ("ASTSMC total-bound invariant valid", ast["total_bound_violation_count_delta"] == 0),
        (
            "ASTSMC allocator timestamps not in future",
            ast["allocator"].get("future_timestamp_count", 1) == 0,
        ),
    ]
    return numeric_checks, boolean_checks


def check_thresholds(result, args):
    checks = [
        ("tracking RMS", result["track_square"]["rms"], "<=", args.max_track_rms),
        ("tracking p95", result["track_square"]["p95"], "<=", args.max_track_p95),
        ("tracking max", result["track_square"]["max"], "<=", args.max_track_max),
        ("square final mean", result["final_square"]["mean"], "<=", args.max_final_mean),
        ("square final max", result["final_square"]["max"], "<=", args.max_final_max),
        ("takeoff final max", result["final_takeoff"]["max"], "<=", args.max_takeoff_final),
        ("unallocated torque max", result["unallocated_torque_max"], "<=", args.max_unallocated_torque),
        ("unallocated thrust max", result["unallocated_thrust_max"], "<=", args.max_unallocated_thrust),
        ("motor max", result["motor_max"], "<=", args.max_motor),
    ]

    for axis in AXES:
        rate_error_limit = args.max_rate_error_rms_yaw if axis == "yaw" else args.max_rate_error_rms_rp
        oscillation_limit = args.max_rate_osc_rms_yaw if axis == "yaw" else args.max_rate_osc_rms_rp
        checks.extend(
            [
                (f"{axis} rate error RMS", result["rate_error"][axis]["rms"], "<=", rate_error_limit),
                (f"{axis} high-frequency rate RMS", result["rate_oscillation"][axis]["rms"], "<=", oscillation_limit),
                (
                    f"{axis} torque-limit occupancy",
                    result["torque_limit_fraction"][axis],
                    "<=",
                    args.max_torque_limit_fraction,
                ),
                (
                    f"{axis} torque command bound",
                    result["torque"][axis]["max"],
                    "<=",
                    result["torque_limits"][axis] + args.torque_limit_tolerance,
                ),
            ]
        )

        if axis != "yaw":
            checks.append(
                (
                    f"{axis} rate excitation RMS",
                    result["rate"][axis]["rms"],
                    ">=",
                    args.min_rate_excitation_rms_rp,
                )
            )

    if args.controller == 2 and result["smc_diagnostics"] is not None:
        for axis_index, axis in enumerate(AXES):
            checks.append(
                (
                    f"{axis} internal SMC limit occupancy",
                    result["smc_diagnostics"]["limit_fraction"][axis_index],
                    "<=",
                    args.max_smc_limit_fraction,
                )
            )

    if args.yaw_step_deg > 0.0:
        yaw_steps = result["yaw_steps"]

        if yaw_steps is None:
            return ["yaw-step metrics missing"]

        final_error_max = max(segment["final_error"]["max"] for segment in yaw_steps["segments"])
        settling_max = max(
            segment["settling_s"] if math.isfinite(segment["settling_s"]) else math.inf
            for segment in yaw_steps["segments"]
        )
        checks.extend(
            [
                ("yaw commanded excursion", math.degrees(yaw_steps["commanded_excursion"]), ">=", args.min_yaw_excursion_deg),
                ("yaw step error RMS", math.degrees(yaw_steps["error"]["rms"]), "<=", args.max_yaw_error_rms_deg),
                ("yaw final error max", math.degrees(final_error_max), "<=", args.max_yaw_final_error_deg),
                ("yaw settling max", settling_max, "<=", args.max_yaw_settling),
                ("yaw rate excitation", math.degrees(yaw_steps["max_rate"]), ">=", args.min_yaw_rate_deg_s),
                ("yaw-hold position error RMS", yaw_steps["position_error"]["rms"], "<=", args.max_yaw_position_rms),
                ("yaw-hold position error max", yaw_steps["position_error"]["max"], "<=", args.max_yaw_position_max),
            ]
        )

    failures = evaluate_numeric_checks(checks)
    boolean_checks = [
        ("failsafe clear", not result["failsafe"]),
        ("controller parameters unchanged in flight", not result["controller_parameter_changes"]),
        ("thrust achieved", result["thrust_all"]),
        ("no motor high saturation samples", result["motor_near_high"] == 0),
    ]

    if args.require_torque_all:
        boolean_checks.append(("torque achieved", result["torque_all"]))

    else:
        print(f"INFO torque_all={result['torque_all']} gated by unallocated torque threshold")

    if args.controller == 2:
        boolean_checks.append(("SMC runtime status valid", result["smc_status_valid"]))

    if args.controller == 3:
        ast_numeric_checks, ast_boolean_checks = astsmc_threshold_checks(result, args)
        failures.extend(evaluate_numeric_checks(ast_numeric_checks))
        boolean_checks.extend(ast_boolean_checks)

    failures.extend(evaluate_boolean_checks(boolean_checks))
    expected_parameters = expected_controller_parameters(args)

    for name, expected in expected_parameters.items():
        actual = result["parameters"].get(name)
        passed = actual is not None and math.isclose(
            float(actual),
            float(expected),
            rel_tol=args.parameter_relative_tolerance,
            abs_tol=args.parameter_absolute_tolerance,
        )
        status = "PASS" if passed else "FAIL"
        print(f"{status} {name}: logged={actual} expected={expected}")

        if not passed:
            failures.append(name)

    if getattr(args, "validate_live_log_profile", False):
        actual = result["parameters"].get("SDLOG_PROFILE")
        passed = actual is not None and int(actual) == ACCEPTANCE_LOG_PROFILE
        print(f"{'PASS' if passed else 'FAIL'} SDLOG_PROFILE: logged={actual} expected={ACCEPTANCE_LOG_PROFILE}")

        if not passed:
            failures.append("SDLOG_PROFILE")

    return failures


def analysis_args(args, config):
    return Namespace(
        takeoff_hold=args.takeoff_hold,
        move=config.move_s,
        hold=config.hold_s,
        side=args.side,
        altitude=args.altitude,
        final_s=args.final_s,
        motor_low=args.motor_low,
        motor_high=args.motor_high,
        rate_osc_cutoff=args.rate_osc_cutoff,
        torque_limit_ratio=args.torque_limit_ratio,
        yaw_step_deg=args.yaw_step_deg,
        yaw_hold=args.yaw_hold,
        yaw_final_s=args.yaw_final_s,
        yaw_settle_threshold_deg=args.yaw_settle_threshold_deg,
        yaw_settle_dwell=args.yaw_settle_dwell,
    )


def analyze_acceptance_run(log_path, args, config):
    result = analyze_log(log_path, analysis_args(args, config))
    print_result(result, args.final_s, compact=False)
    failures = check_thresholds(result, args)
    print("\nairborne_terminal_health:")
    airborne_result = analyze_airborne_log(
        log_path,
        terminal_window_s=args.terminal_window,
        cutoff_hz=args.rate_osc_cutoff,
        growth_floor=args.terminal_growth_floor,
    )
    failures.extend(
        check_airborne_result(
            airborne_result,
            max_oscillation_rms_rp=args.max_rate_osc_rms_rp,
            max_rate_error_rms_rp=args.max_rate_error_rms_rp,
            max_growth_ratio=args.max_terminal_growth_ratio,
        )
    )
    return result, airborne_result, failures


def comparison_metrics(result, airborne_result):
    yaw_steps = result["yaw_steps"]
    metrics = {
        "track_rms": result["track_square"]["rms"],
        "track_p95": result["track_square"]["p95"],
        "track_max": result["track_square"]["max"],
        "square_final_mean": result["final_square"]["mean"],
        "square_final_max": result["final_square"]["max"],
        "unallocated_torque_max": result["unallocated_torque_max"],
        "motor_max": result["motor_max"],
    }

    for axis in AXES:
        metrics[f"{axis}_rate_error_rms"] = result["rate_error"][axis]["rms"]
        metrics[f"{axis}_rate_osc_rms"] = result["rate_oscillation"][axis]["rms"]
        metrics[f"{axis}_torque_rms"] = result["torque"][axis]["rms"]
        metrics[f"{axis}_torque_peak"] = result["torque"][axis]["max"]
        metrics[f"{axis}_torque_hf_rms"] = result["torque_dynamics"][axis]["high_frequency"]["rms"]
        metrics[f"{axis}_torque_variation_per_s"] = result["torque_dynamics"][axis]["variation_per_s"]
        metrics[f"{axis}_torque_limit_fraction"] = result["torque_limit_fraction"][axis]

        if result["astsmc_diagnostics"] is not None:
            diagnostics = result["astsmc_diagnostics"]
            tracking = diagnostics["tracking"].get(axis, {})
            components = diagnostics["axes"].get(axis, {}).get(
                "torque_components", {}
            )

            for name, key in (
                ("raw_rate_error_rms", "raw_error"),
                ("reference_shaping_lag_rms", "reference_shaping_lag"),
                ("conditioned_rate_error_rms", "conditioned_error"),
            ):
                metrics[f"{axis}_ast_{name}"] = tracking.get(key, {}).get(
                    "rms", math.nan
                )

            for component in ("nominal", "residual", "total"):
                component_metrics = components.get(component, {})

                for metric in (
                    "rms",
                    "peak",
                    "high_frequency_rms",
                    "variation_per_s",
                    "saturation_fraction",
                ):
                    metrics[f"{axis}_ast_{component}_{metric}"] = (
                        component_metrics.get(metric, math.nan)
                    )

    if yaw_steps is not None:
        metrics["yaw_error_rms_deg"] = math.degrees(yaw_steps["error"]["rms"])
        metrics["yaw_final_error_max_deg"] = math.degrees(
            max(segment["final_error"]["max"] for segment in yaw_steps["segments"])
        )
        metrics["yaw_settling_max_s"] = max(
            segment["settling_s"] if math.isfinite(segment["settling_s"]) else math.inf
            for segment in yaw_steps["segments"]
        )
        metrics["yaw_position_max"] = yaw_steps["position_error"]["max"]

    terminal_segment = max(airborne_result["segments"], key=lambda segment: segment["end_s"])

    for axis in ("roll", "pitch"):
        terminal = terminal_segment["axes"][axis]
        metrics[f"terminal_{axis}_rate_error_rms"] = terminal["rate_error_rms"]
        metrics[f"terminal_{axis}_oscillation_rms"] = terminal["oscillation_rms"]
        metrics[f"terminal_{axis}_growth_ratio"] = terminal["growth_ratio"]

    return metrics


def metric_spec(metric_name):
    return METRIC_REGISTRY.get(metric_name, DEFAULT_METRIC_SPEC)


def comparison_margin(reference, spec):
    return max(
        abs(float(reference)) * spec["relative_margin"],
        spec["absolute_margin"],
    )


def paired_metric_decision(primary, compared, metric_name):
    spec = metric_spec(metric_name)

    if not math.isfinite(primary) or not math.isfinite(compared):
        return {
            "status": "invalid",
            "passed": False,
            "delta": math.nan,
            "margin": math.nan,
            "spec": spec,
        }

    direction = spec["direction"]
    delta = primary - compared
    signed_delta = delta if direction == "lower" else -delta
    margin = comparison_margin(compared, spec)

    if spec["class"] in ("descriptive", "invariant"):
        return {
            "status": spec["class"],
            "passed": None,
            "delta": delta,
            "regression": signed_delta,
            "margin": margin,
            "spec": spec,
        }

    return {
        "status": "pass" if signed_delta <= margin else "fail",
        "passed": signed_delta <= margin,
        "delta": delta,
        "regression": signed_delta,
        "margin": margin,
        "spec": spec,
    }


def comparison_decisions(records, primary_controller, compare_controller):
    rounds = sorted(set(record["round"] for record in records))
    decisions = {}

    for round_index in rounds:
        by_controller = {
            record["controller"]: record
            for record in records
            if record["round"] == round_index
        }

        if primary_controller not in by_controller or compare_controller not in by_controller:
            continue

        common_metrics = sorted(
            set(by_controller[primary_controller]["metrics"])
            & set(by_controller[compare_controller]["metrics"])
        )
        decisions[str(round_index)] = {
            metric_name: paired_metric_decision(
                by_controller[primary_controller]["metrics"][metric_name],
                by_controller[compare_controller]["metrics"][metric_name],
                metric_name,
            )
            for metric_name in common_metrics
        }

    return decisions


def print_comparison(records, primary_controller, compare_controller):
    print("\ncomparison summary:")
    grouped = {primary_controller: [], compare_controller: []}

    for record in records:
        grouped[record["controller"]].append(record)

    metric_names = sorted(
        set.intersection(
            *(set(record["metrics"]) for record in records)
        )
    )

    for metric_name in metric_names:
        parts = []

        for controller in (primary_controller, compare_controller):
            values = [record["metrics"][metric_name] for record in grouped[controller]]
            finite_values = [value for value in values if math.isfinite(value)]

            if finite_values:
                parts.append(
                    f"ctrl{controller} median={statistics.median(finite_values):.6f} "
                    f"range=[{min(finite_values):.6f},{max(finite_values):.6f}] "
                    f"values={','.join(f'{value:.6f}' for value in values)}"
                )

        if parts:
            print(f"  {metric_name}: {'; '.join(parts)}")

    print("paired decisions (primary - compare):")
    decisions = comparison_decisions(
        records, primary_controller, compare_controller
    )

    for round_index, round_decisions in decisions.items():
        parts = []

        for metric_name in metric_names:
            decision = round_decisions.get(metric_name)

            if decision is None or decision["status"] == "invalid":
                continue

            parts.append(
                f"{metric_name}={decision['delta']:+.6f} "
                f"margin={decision['margin']:.6f} "
                f"{decision['status'].upper()}[{decision['spec']['class']}]"
            )

        print(f"  round {int(round_index) + 1}: {'; '.join(parts)}")

    return decisions


def json_value(value):
    if isinstance(value, Path):
        return str(value)

    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]

    if isinstance(value, float) and not math.isfinite(value):
        return None

    if hasattr(value, "item"):
        return json_value(value.item())

    return value


def git_state(root):
    def run(*command):
        completed = subprocess.run(
            command,
            cwd=root,
            check=False,
            capture_output=True,
            text=True,
        )
        return completed.stdout.strip() if completed.returncode == 0 else None

    status = run("git", "status", "--short")
    return {
        "head": run("git", "rev-parse", "HEAD"),
        "branch": run("git", "branch", "--show-current"),
        "dirty": bool(status),
        "status_short": status.splitlines() if status else [],
    }


def campaign_artifact(args, config, records, decisions):
    root = Path(__file__).resolve().parents[1]
    controllers = sorted(set(record["controller"] for record in records))
    cards = {
        str(controller): expected_controller_parameters(args, controller)
        for controller in controllers
    }
    return {
        "schema_version": 1,
        "command": [str(argument) for argument in sys.argv],
        "scenario": {
            "sitl_target": args.sitl_target,
            "move_s": config.move_s,
            "hold_s": config.hold_s,
            "accel_scale": config.accel_scale,
            "side_m": args.side,
            "altitude_m": args.altitude,
            "yaw_step_deg": args.yaw_step_deg,
            "yaw_hold_s": args.yaw_hold,
        },
        "git": git_state(root),
        "cards": cards,
        "metric_registry": METRIC_REGISTRY,
        "records": [
            {
                "round": record["round"],
                "controller": record["controller"],
                "log": record["log"],
                "metrics": record["metrics"],
                "failures": record["failures"],
                "logged_parameters": record["result"]["parameters"],
                "rate_timestamp_source": record["result"]["rate_timestamp_source"],
            }
            for record in records
        ],
        "paired_decisions": decisions,
    }


def write_campaign_artifact(path, artifact):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(json_value(artifact), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def build_run_args(args, controller, live):
    run_args = Namespace(**vars(args))
    run_args.controller = controller
    run_args.validate_live_log_profile = live
    apply_expected = controller == 3 or args.apply_expected_parameters or args.compare_controller is not None
    run_args.parameter_overrides = expected_controller_parameters(run_args) if live and apply_expected else {}
    return run_args


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--takeoff-hold", type=float, default=8.0)
    parser.add_argument("--move", type=float, default=3.2)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--side", type=float, default=0.6)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--accel-scale", type=float, default=0.75)
    parser.add_argument("--yaw-step-deg", type=float, default=30.0)
    parser.add_argument("--yaw-hold", type=float, default=3.0)
    parser.add_argument("--yaw-final-s", type=float, default=0.5)
    parser.add_argument("--yaw-settle-threshold-deg", type=float, default=5.0)
    parser.add_argument("--yaw-settle-dwell", type=float, default=0.25)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--log", action="append", type=Path, help="analyze an existing log instead of flying")
    parser.add_argument(
        "--campaign-log",
        action="append",
        type=parse_campaign_log,
        help="analyze an existing paired log as round,controller,path",
    )
    parser.add_argument(
        "--artifact",
        type=Path,
        help="write a machine-readable campaign manifest and result",
    )
    parser.add_argument("--startup-wait", type=float, default=35.0)
    parser.add_argument("--controller", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--compare-controller", type=int, choices=(1, 2, 3))
    parser.add_argument("--sitl-target", choices=("gz_uav985", "gz_uav985_flow"), default="gz_uav985")
    parser.add_argument("--final-s", type=float, default=1.0)
    parser.add_argument("--motor-low", type=float, default=0.05)
    parser.add_argument("--motor-high", type=float, default=0.95)
    parser.add_argument("--max-track-rms", type=float, default=0.05)
    parser.add_argument("--max-track-p95", type=float, default=0.08)
    parser.add_argument("--max-track-max", type=float, default=0.12)
    parser.add_argument("--max-final-mean", type=float, default=0.03)
    parser.add_argument("--max-repeat-final-mean", type=float, default=0.03)
    parser.add_argument("--max-final-max", type=float, default=0.055)
    parser.add_argument("--max-takeoff-final", type=float, default=0.08)
    parser.add_argument("--max-unallocated-torque", type=float, default=0.005)
    parser.add_argument("--max-unallocated-thrust", type=float, default=0.001)
    parser.add_argument("--max-motor", type=float, default=0.95)
    parser.add_argument("--rate-osc-cutoff", type=float, default=4.0)
    parser.add_argument("--torque-limit-ratio", type=float, default=0.98)
    parser.add_argument("--max-rate-error-rms-rp", type=float, default=0.12)
    parser.add_argument("--max-rate-error-rms-yaw", type=float, default=0.10)
    parser.add_argument("--max-rate-osc-rms-rp", type=float, default=0.08)
    parser.add_argument("--max-rate-osc-rms-yaw", type=float, default=0.06)
    parser.add_argument("--terminal-window", type=float, default=6.0)
    parser.add_argument("--terminal-growth-floor", type=float, default=0.02)
    parser.add_argument("--max-terminal-growth-ratio", type=float, default=2.5)
    parser.add_argument("--min-rate-excitation-rms-rp", type=float, default=0.01)
    parser.add_argument("--max-torque-limit-fraction", type=float, default=0.05)
    parser.add_argument("--max-smc-limit-fraction", type=float, default=0.05)
    parser.add_argument("--min-ast-samples", type=int, default=100)
    parser.add_argument("--min-ast-rate-hz", type=float, default=100.0)
    parser.add_argument("--max-ast-invalid-dt-fraction", type=float, default=0.0)
    parser.add_argument("--min-ast-allocator-feedback-valid-fraction", type=float, default=1.0)
    parser.add_argument("--max-ast-allocator-stale-fraction", type=float, default=0.0)
    parser.add_argument("--max-ast-internal-saturation-fraction", type=float, default=0.05)
    parser.add_argument("--max-ast-allocation-residual", type=float, default=0.005)
    parser.add_argument("--min-ast-torque-timestamp-match-fraction", type=float, default=0.99)
    parser.add_argument("--min-yaw-excursion-deg", type=float, default=50.0)
    parser.add_argument("--max-yaw-error-rms-deg", type=float, default=18.0)
    parser.add_argument("--max-yaw-final-error-deg", type=float, default=5.0)
    parser.add_argument("--max-yaw-settling", type=float, default=1.5)
    parser.add_argument("--min-yaw-rate-deg-s", type=float, default=20.0)
    parser.add_argument("--max-yaw-position-rms", type=float, default=0.08)
    parser.add_argument("--max-yaw-position-max", type=float, default=0.15)
    parser.add_argument("--torque-limit-tolerance", type=float, default=0.002)
    parser.add_argument("--expected-inertia", type=parse_vector, default=DEFAULT_INERTIA)
    parser.add_argument("--expected-effectiveness", type=parse_vector, default=DEFAULT_EFFECTIVENESS)
    parser.add_argument(
        "--expected-torque-limit",
        type=parse_vector,
        default=None,
        help="override the controller-specific default torque limits",
    )
    parser.add_argument("--expected-mpc-q", type=parse_vector, default=(1.0, 1.0, 0.7))
    parser.add_argument("--expected-mpc-r", type=parse_vector, default=(1.0, 1.0, 1.0))
    parser.add_argument("--expected-mpc-du", type=parse_vector, default=(0.05, 0.05, 0.05))
    parser.add_argument("--expected-actuator-tau", type=float, default=0.025)
    parser.add_argument("--expected-horizon", type=int, default=8)
    parser.add_argument("--expected-mpc-slew", type=float, default=10.0)
    parser.add_argument("--expected-mpc-gyro", type=float, default=1.0)
    parser.add_argument("--expected-smc-c", type=parse_vector, default=(2.0, 2.0, 2.0))
    parser.add_argument("--expected-smc-eta", type=parse_vector, default=(7.0, 8.2, 1.0))
    parser.add_argument("--expected-smc-boundary", type=parse_vector, default=(0.5, 0.5, 0.20))
    parser.add_argument("--expected-smc-ks", type=parse_vector, default=(1.0, 1.0, 1.0))
    parser.add_argument("--expected-smc-rate-sp-derivative", type=float, default=0.0)
    parser.add_argument("--expected-smc-lpf", type=float, default=20.0)
    parser.add_argument("--expected-smc-slew", type=float, default=15.0)
    parser.add_argument(
        "--expected-smc-integral-limit",
        type=parse_vector,
        default=(0.3, 0.3, 0.3),
    )
    parser.add_argument("--expected-ast-k1", type=parse_vector, default=(3.0, 3.0, 1.5))
    parser.add_argument("--expected-ast-k2", type=parse_vector, default=(4.5, 4.5, 1.5))
    parser.add_argument("--expected-ast-reference-acceleration", type=parse_vector, default=(20.0, 20.0, 5.0))
    parser.add_argument("--expected-ast-reference-jerk", type=parse_vector, default=(100.0, 100.0, 40.0))
    parser.add_argument("--expected-ast-residual-torque", type=parse_vector, default=(0.10, 0.10, 0.05))
    parser.add_argument("--expected-ast-actuator-tau", type=parse_vector, default=(0.0, 0.0, 0.0))
    parser.add_argument("--expected-ast-slew", type=parse_vector, default=(0.0, 0.0, 0.0))
    parser.add_argument("--expected-ast-variation-weight", type=parse_vector, default=(0.0, 0.0, 0.0))
    parser.add_argument("--expected-ast-sliding-boundary", type=parse_vector, default=(0.0, 0.0, 0.0))
    parser.add_argument("--expected-ast-tracking-blend", type=float, default=0.0)
    parser.add_argument("--expected-ast-reference-feedforward", type=float, default=1.0)
    parser.add_argument("--expected-ast-reference-feedforward-rp", type=float, default=1.0)
    parser.add_argument("--expected-ast-gyro", type=float, default=1.0)
    parser.add_argument("--expected-ast-dt-min", type=float, default=0.0005)
    parser.add_argument("--expected-ast-dt-max", type=float, default=0.005)
    parser.add_argument("--expected-ast-recovery-error", type=float, default=1.0)
    parser.add_argument("--parameter-relative-tolerance", type=float, default=0.01)
    parser.add_argument("--parameter-absolute-tolerance", type=float, default=1e-5)
    parser.add_argument("--require-torque-all", action="store_true")
    parser.add_argument(
        "--apply-expected-parameters",
        action="store_true",
        help="apply the expected controller signature to a locally launched UDP SITL instance",
    )
    args = parser.parse_args()

    existing_logs = bool(args.log or args.campaign_log)

    if args.repeat < 1 and not existing_logs:
        raise ValueError("--repeat must be at least 1")

    if args.compare_controller == args.controller:
        raise ValueError("--compare-controller must differ from --controller")

    if args.log and args.campaign_log:
        raise ValueError("--log and --campaign-log cannot be combined")

    if args.compare_controller is not None and args.log:
        raise ValueError("paired existing-log analysis requires --campaign-log")

    if args.campaign_log and args.compare_controller is None:
        raise ValueError("--campaign-log requires --compare-controller")

    if args.apply_expected_parameters and existing_logs:
        raise ValueError("--apply-expected-parameters cannot be used with existing logs")

    live_controllers = {args.controller}

    if args.compare_controller is not None:
        live_controllers.add(args.compare_controller)

    if not existing_logs and (3 in live_controllers or args.apply_expected_parameters or args.compare_controller is not None):
        if not looks_like_local_udp(args.connection):
            raise ValueError("controller-card overrides are restricted to explicit localhost UDP SITL")

    config = GridConfig(args.move, args.hold, args.accel_scale)
    records = []
    all_failures = []

    if args.campaign_log:
        schedule = [
            (round_index, controller, path)
            for round_index, controller, path in args.campaign_log
        ]

    elif args.log:
        schedule = [(index, args.controller, path) for index, path in enumerate(args.log)]

    elif args.compare_controller is not None:
        schedule = [
            (round_index, controller, None)
            for round_index, controller in comparison_schedule(
                args.controller, args.compare_controller, args.repeat
            )
        ]

    else:
        schedule = [(index, args.controller, None) for index in range(args.repeat)]

    for run_index, (round_index, controller, existing_log) in enumerate(schedule, start=1):
        run_args = build_run_args(args, controller, live=existing_log is None)
        print(
            f"\n== acceptance run {run_index}/{len(schedule)}: "
            f"round={round_index + 1} controller={controller} {config.tag}",
            flush=True,
        )
        log_path = existing_log if existing_log is not None else run_one(config, run_args)
        result, airborne_result, failures = analyze_acceptance_run(log_path, run_args, config)
        record = {
            "round": round_index,
            "controller": controller,
            "log": log_path,
            "result": result,
            "airborne": airborne_result,
            "metrics": comparison_metrics(result, airborne_result),
            "failures": failures,
        }
        records.append(record)

        if failures:
            all_failures.append((log_path, failures))

    controllers = sorted(set(record["controller"] for record in records))

    for controller in controllers:
        controller_records = [record for record in records if record["controller"] == controller]

        if len(controller_records) >= 2:
            repeat_final_mean = statistics.mean(
                record["result"]["final_square"]["mean"] for record in controller_records
            )
            passed = repeat_final_mean <= args.max_repeat_final_mean
            print(
                f"{'PASS' if passed else 'FAIL'} controller {controller} repeat square-final mean: "
                f"{repeat_final_mean:.5f} <= {args.max_repeat_final_mean:.5f}"
            )

            if not passed:
                all_failures.append(
                    (Path(f"controller-{controller}-repeat-aggregate"), ["repeat square-final mean"])
                )

    decisions = {}

    if args.compare_controller is not None:
        decisions = print_comparison(
            records, args.controller, args.compare_controller
        )
        comparative_failures = [
            f"round {int(round_index) + 1} {metric_name}"
            for round_index, round_decisions in decisions.items()
            for metric_name, decision in round_decisions.items()
            if decision["passed"] is False
        ]

        if comparative_failures:
            all_failures.append(
                (Path("paired-comparison"), comparative_failures)
            )

    if args.artifact is not None:
        write_campaign_artifact(
            args.artifact,
            campaign_artifact(args, config, records, decisions),
        )
        print(f"\ncampaign artifact: {args.artifact}")

    print("\nacceptance logs:")

    for record in records:
        print(f"controller={record['controller']} round={record['round'] + 1} {record['log']}")

    if all_failures:
        print("\nACCEPTANCE FAILED")

        for log_path, failures in all_failures:
            print(f"{log_path.name}: {', '.join(failures)}")

        return 2

    print(
        "\nACCEPTANCE PASSED "
        f"runs={len(records)} move={args.move:.2f} hold={args.hold:.2f} "
        f"side={args.side:.2f} altitude={args.altitude:.2f} accel_scale={args.accel_scale:.2f}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
