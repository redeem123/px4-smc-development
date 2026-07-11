#!/usr/bin/env python3
import argparse
import math
import sys
from argparse import Namespace
from pathlib import Path

from analyze_offboard_square import AXES, analyze_log, print_result
from run_uav985_trajectory_grid import GridConfig, run_one


def parse_vector(value):
    values = tuple(float(item) for item in value.split(","))

    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")

    return values


def expected_controller_parameters(args):
    expected_parameters = {
        "MC_RATE_CTRL_T": args.controller,
        "MIS_TKO_ALT_MAX": 1.0,
    }

    if args.controller == 1:
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
                "MC_MPC_TMAX_R": args.expected_torque_limit[0],
                "MC_MPC_TMAX_P": args.expected_torque_limit[1],
                "MC_MPC_TMAX_Y": args.expected_torque_limit[2],
            }
        )

    else:
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
                "MC_MSMC_TMAX_R": args.expected_torque_limit[0],
                "MC_MSMC_TMAX_P": args.expected_torque_limit[1],
                "MC_MSMC_TMAX_Y": args.expected_torque_limit[2],
            }
        )

    return expected_parameters


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
                (
                    "yaw commanded excursion",
                    math.degrees(yaw_steps["commanded_excursion"]),
                    ">=",
                    args.min_yaw_excursion_deg,
                ),
                (
                    "yaw step error RMS",
                    math.degrees(yaw_steps["error"]["rms"]),
                    "<=",
                    args.max_yaw_error_rms_deg,
                ),
                (
                    "yaw final error max",
                    math.degrees(final_error_max),
                    "<=",
                    args.max_yaw_final_error_deg,
                ),
                ("yaw settling max", settling_max, "<=", args.max_yaw_settling),
                (
                    "yaw rate excitation",
                    math.degrees(yaw_steps["max_rate"]),
                    ">=",
                    args.min_yaw_rate_deg_s,
                ),
                (
                    "yaw-hold position error RMS",
                    yaw_steps["position_error"]["rms"],
                    "<=",
                    args.max_yaw_position_rms,
                ),
                (
                    "yaw-hold position error max",
                    yaw_steps["position_error"]["max"],
                    "<=",
                    args.max_yaw_position_max,
                ),
            ]
        )

    failures = []

    for label, value, operator, threshold in checks:
        passed = value <= threshold if operator == "<=" else value >= threshold
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}: {value:.5f} {operator} {threshold:.5f}")

        if not passed:
            failures.append(label)

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

    for label, passed in boolean_checks:
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}")

        if not passed:
            failures.append(label)

    expected_parameters = expected_controller_parameters(args)

    for name, expected in expected_parameters.items():
        actual = result["parameters"].get(name)
        passed = actual is not None and math.isclose(
            float(actual), float(expected), rel_tol=args.parameter_relative_tolerance,
            abs_tol=args.parameter_absolute_tolerance
        )
        status = "PASS" if passed else "FAIL"
        print(f"{status} {name}: logged={actual} expected={expected}")

        if not passed:
            failures.append(name)

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--takeoff-hold", type=float, default=7.0)
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
    parser.add_argument("--startup-wait", type=float, default=35.0)
    parser.add_argument("--controller", type=int, choices=(1, 2), default=1)
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
    parser.add_argument("--min-rate-excitation-rms-rp", type=float, default=0.01)
    parser.add_argument("--max-torque-limit-fraction", type=float, default=0.05)
    parser.add_argument("--max-smc-limit-fraction", type=float, default=0.05)
    parser.add_argument("--min-yaw-excursion-deg", type=float, default=50.0)
    parser.add_argument("--max-yaw-error-rms-deg", type=float, default=18.0)
    parser.add_argument("--max-yaw-final-error-deg", type=float, default=5.0)
    parser.add_argument("--max-yaw-settling", type=float, default=1.5)
    parser.add_argument("--min-yaw-rate-deg-s", type=float, default=20.0)
    parser.add_argument("--max-yaw-position-rms", type=float, default=0.08)
    parser.add_argument("--max-yaw-position-max", type=float, default=0.15)
    parser.add_argument("--torque-limit-tolerance", type=float, default=0.002)
    parser.add_argument("--expected-inertia", type=parse_vector, default=(0.0135, 0.0118, 0.0170))
    parser.add_argument("--expected-effectiveness", type=parse_vector, default=(4.03, 4.03, 0.876))
    parser.add_argument("--expected-torque-limit", type=parse_vector, default=(0.20, 0.20, 0.10))
    parser.add_argument("--expected-mpc-q", type=parse_vector, default=(1.0, 1.0, 0.7))
    parser.add_argument("--expected-mpc-r", type=parse_vector, default=(1.0, 1.0, 1.0))
    parser.add_argument("--expected-mpc-du", type=parse_vector, default=(0.05, 0.05, 0.05))
    parser.add_argument("--expected-actuator-tau", type=float, default=0.025)
    parser.add_argument("--expected-horizon", type=int, default=8)
    parser.add_argument("--expected-mpc-slew", type=float, default=10.0)
    parser.add_argument("--expected-mpc-gyro", type=float, default=1.0)
    parser.add_argument("--expected-smc-c", type=parse_vector, default=(3.0, 3.0, 1.5))
    parser.add_argument("--expected-smc-eta", type=parse_vector, default=(6.0, 6.0, 2.0))
    parser.add_argument("--expected-smc-boundary", type=parse_vector, default=(0.15, 0.15, 0.20))
    parser.add_argument("--expected-smc-ks", type=parse_vector, default=(1.5, 1.5, 0.5))
    parser.add_argument("--expected-smc-rate-sp-derivative", type=float, default=20.0)
    parser.add_argument("--expected-smc-lpf", type=float, default=20.0)
    parser.add_argument("--expected-smc-slew", type=float, default=15.0)
    parser.add_argument("--parameter-relative-tolerance", type=float, default=0.01)
    parser.add_argument("--parameter-absolute-tolerance", type=float, default=1e-5)
    parser.add_argument("--require-torque-all", action="store_true")
    parser.add_argument(
        "--apply-expected-parameters",
        action="store_true",
        help="apply the expected controller signature to a locally launched UDP SITL instance",
    )
    args = parser.parse_args()

    if args.repeat < 1 and not args.log:
        raise ValueError("--repeat must be at least 1")

    if args.apply_expected_parameters and args.log:
        raise ValueError("--apply-expected-parameters cannot be used with --log")

    if args.apply_expected_parameters and not args.connection.startswith("udp:"):
        raise ValueError("--apply-expected-parameters is restricted to UDP SITL connections")

    args.parameter_overrides = (
        expected_controller_parameters(args) if args.apply_expected_parameters else {}
    )

    config = GridConfig(args.move, args.hold, args.accel_scale)
    logs = []
    results = []
    all_failures = []

    run_count = len(args.log) if args.log else args.repeat

    for index in range(run_count):
        print(f"\n== acceptance run {index + 1}/{run_count}: controller={args.controller} {config.tag}", flush=True)
        log_path = args.log[index] if args.log else run_one(config, args)
        logs.append(log_path)
        result = analyze_log(log_path, analysis_args(args, config))
        results.append(result)
        print_result(result, args.final_s, compact=False)
        failures = check_thresholds(result, args)

        if failures:
            all_failures.append((log_path, failures))

    if run_count >= 2:
        repeat_final_mean = sum(result["final_square"]["mean"] for result in results) / run_count
        passed = repeat_final_mean <= args.max_repeat_final_mean
        print(
            f"{'PASS' if passed else 'FAIL'} repeat square-final mean: "
            f"{repeat_final_mean:.5f} <= {args.max_repeat_final_mean:.5f}"
        )

        if not passed:
            all_failures.append((Path("repeat-aggregate"), ["repeat square-final mean"]))

    print("\nacceptance logs:")

    for log_path in logs:
        print(log_path)

    if all_failures:
        print("\nACCEPTANCE FAILED")

        for log_path, failures in all_failures:
            print(f"{log_path.name}: {', '.join(failures)}")

        return 2

    print(
        "\nACCEPTANCE PASSED "
        f"runs={run_count} move={args.move:.2f} hold={args.hold:.2f} "
        f"side={args.side:.2f} altitude={args.altitude:.2f} accel_scale={args.accel_scale:.2f}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
