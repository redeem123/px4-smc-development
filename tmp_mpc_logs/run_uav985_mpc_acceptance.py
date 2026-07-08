#!/usr/bin/env python3
import argparse
import sys
from argparse import Namespace

from analyze_offboard_square import analyze_log, print_result
from run_uav985_trajectory_grid import GridConfig, run_one


def check_thresholds(result, args):
    checks = [
        ("tracking RMS", result["track_square"]["rms"], "<=", args.max_track_rms),
        ("tracking p95", result["track_square"]["p95"], "<=", args.max_track_p95),
        ("tracking max", result["track_square"]["max"], "<=", args.max_track_max),
        ("final mean", result["final"]["mean"], "<=", args.max_final_mean),
        ("final max", result["final"]["max"], "<=", args.max_final_max),
        ("unallocated torque max", result["unallocated_torque_max"], "<=", args.max_unallocated_torque),
        ("unallocated thrust max", result["unallocated_thrust_max"], "<=", args.max_unallocated_thrust),
        ("motor max", result["motor_max"], "<=", args.max_motor),
    ]

    failures = []

    for label, value, operator, threshold in checks:
        passed = value <= threshold
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}: {value:.5f} {operator} {threshold:.5f}")

        if not passed:
            failures.append(label)

    boolean_checks = [
        ("failsafe clear", not result["failsafe"]),
        ("thrust achieved", result["thrust_all"]),
        ("no motor high saturation samples", result["motor_near_high"] == 0),
    ]

    if args.require_torque_all:
        boolean_checks.append(("torque achieved", result["torque_all"]))

    else:
        print(f"INFO torque_all={result['torque_all']} gated by unallocated torque threshold")

    for label, passed in boolean_checks:
        status = "PASS" if passed else "FAIL"
        print(f"{status} {label}")

        if not passed:
            failures.append(label)

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
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--startup-wait", type=float, default=35.0)
    parser.add_argument("--final-s", type=float, default=1.0)
    parser.add_argument("--motor-low", type=float, default=0.05)
    parser.add_argument("--motor-high", type=float, default=0.95)
    parser.add_argument("--max-track-rms", type=float, default=0.05)
    parser.add_argument("--max-track-p95", type=float, default=0.08)
    parser.add_argument("--max-track-max", type=float, default=0.12)
    parser.add_argument("--max-final-mean", type=float, default=0.025)
    parser.add_argument("--max-final-max", type=float, default=0.05)
    parser.add_argument("--max-unallocated-torque", type=float, default=0.005)
    parser.add_argument("--max-unallocated-thrust", type=float, default=0.001)
    parser.add_argument("--max-motor", type=float, default=0.95)
    parser.add_argument("--require-torque-all", action="store_true")
    args = parser.parse_args()

    if args.repeat < 1:
        raise ValueError("--repeat must be at least 1")

    config = GridConfig(args.move, args.hold, args.accel_scale)
    logs = []
    all_failures = []

    for index in range(args.repeat):
        print(f"\n== acceptance run {index + 1}/{args.repeat}: {config.tag}", flush=True)
        log_path = run_one(config, args)
        logs.append(log_path)
        result = analyze_log(log_path, analysis_args(args, config))
        print_result(result, args.final_s, compact=False)
        failures = check_thresholds(result, args)

        if failures:
            all_failures.append((log_path, failures))

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
        f"repeat={args.repeat} move={args.move:.2f} hold={args.hold:.2f} "
        f"side={args.side:.2f} altitude={args.altitude:.2f} accel_scale={args.accel_scale:.2f}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
