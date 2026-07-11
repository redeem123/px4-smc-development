#!/usr/bin/env python3
"""Validate a model-based SMC parameter card against a proven PID baseline."""

import argparse
import json
import math
import sys
from pathlib import Path


AXES = (("roll", "R", "ROLL"), ("pitch", "P", "PITCH"), ("yaw", "Y", "YAW"))


def load_parameters(path):
    document = json.loads(path.read_text())
    parameters = document.get("parameters", document)
    return {
        name: entry.get("value") if isinstance(entry, dict) and "value" in entry else entry
        for name, entry in parameters.items()
    }


def finite_parameter(parameters, name):
    if name not in parameters:
        raise RuntimeError(f"required parameter is missing: {name}")

    value = float(parameters[name])

    if not math.isfinite(value):
        raise RuntimeError(f"required parameter is not finite: {name}")

    return value


def axis_gain(parameters, suffix, pid_axis):
    inertia = finite_parameter(parameters, f"MC_MSMC_J_{suffix}")
    effectiveness = finite_parameter(parameters, f"MC_MSMC_EFF_{suffix}")
    surface_gain = finite_parameter(parameters, f"MC_MSMC_C_{suffix}")
    reaching_gain = finite_parameter(parameters, f"MC_MSMC_ETA_{suffix}")
    boundary = finite_parameter(parameters, f"MC_MSMC_BND_{suffix}")
    linear_gain = finite_parameter(parameters, f"MC_MSMC_KS_{suffix}")
    pid_p = finite_parameter(parameters, f"MC_{pid_axis}RATE_P")
    pid_k = finite_parameter(parameters, f"MC_{pid_axis}RATE_K")

    if inertia <= 0.0 or effectiveness <= 0.0 or boundary <= 0.0 or pid_p <= 0.0 or pid_k <= 0.0:
        raise RuntimeError(f"{pid_axis.lower()} model and PID scaling parameters must be positive")

    smc_gain = inertia / effectiveness * (surface_gain + reaching_gain / boundary + linear_gain)
    pid_gain = pid_p * pid_k
    return smc_gain, pid_gain, smc_gain / pid_gain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("parameters", type=Path, help="PX4 parameter JSON backup")
    parser.add_argument("--min-pid-ratio", type=float, default=0.5)
    parser.add_argument("--max-pid-ratio", type=float, default=1.5)
    parser.add_argument("--allow-unacknowledged", action="store_true")
    args = parser.parse_args()

    if args.min_pid_ratio <= 0.0 or args.max_pid_ratio < args.min_pid_ratio:
        raise ValueError("invalid PID gain-ratio envelope")

    parameters = load_parameters(args.parameters)
    acknowledged = int(finite_parameter(parameters, "MC_MSMC_CFG")) == 1
    rejected = []

    print(f"parameters={args.parameters}")
    print(f"model_card_acknowledged={str(acknowledged).lower()}")

    if not acknowledged and not args.allow_unacknowledged:
        rejected.append("model-card-unacknowledged")

    for axis, suffix, pid_axis in AXES:
        smc_gain, pid_gain, ratio = axis_gain(parameters, suffix, pid_axis)
        accepted = args.min_pid_ratio <= ratio <= args.max_pid_ratio

        if not accepted:
            rejected.append(f"{axis}-gain-ratio")

        print(
            f"{axis:5s} smc_local_gain={smc_gain:.6f} pid_p_gain={pid_gain:.6f} "
            f"ratio={ratio:.3f} envelope=[{args.min_pid_ratio:.3f},{args.max_pid_ratio:.3f}] "
            f"sanity={'ACCEPTED' if accepted else 'REJECTED'}"
        )

    print(
        "scope: small-error gain sanity only; physical J/EFF identification "
        "and bounded flight validation remain required"
    )

    if rejected:
        print(f"SMC MODEL CARD REJECTED: {','.join(rejected)}")
        return 2

    print("SMC MODEL CARD SANITY PASSED")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"SMC MODEL CARD VALIDATION FAILED: {exc}", file=sys.stderr)
        sys.exit(1)
