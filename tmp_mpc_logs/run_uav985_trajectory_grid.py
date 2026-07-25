#!/usr/bin/env python3
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROOTFS = ROOT / "build/px4_sitl_default/rootfs"
LOG_ROOT = ROOTFS / "log"
OUTPUT_DIR = ROOT / "tmp_mpc_logs"
ACCEPTANCE_LOG_PROFILE = 25
UAV985_ASTSMC_CARD = {
    "MC_AST_CFG": 1,
    "MC_AST_J_R": 0.040461,
    "MC_AST_J_P": 0.035366,
    "MC_AST_J_Y": 0.050951,
    "MC_AST_EFF_R": 6.978,
    "MC_AST_EFF_P": 6.978,
    "MC_AST_EFF_Y": 1.163,
    "MC_AST_K1_R": 3.0,
    "MC_AST_K1_P": 3.0,
    "MC_AST_K1_Y": 1.5,
    "MC_AST_K2_R": 4.5,
    "MC_AST_K2_P": 4.5,
    "MC_AST_K2_Y": 1.5,
    "MC_AST_TMAX_R": 0.20,
    "MC_AST_TMAX_P": 0.20,
    "MC_AST_TMAX_Y": 0.15,
    "MC_AST_RACC_R": 20.0,
    "MC_AST_RACC_P": 20.0,
    "MC_AST_RACC_Y": 5.0,
    "MC_AST_RJERK_R": 100.0,
    "MC_AST_RJERK_P": 100.0,
    "MC_AST_RJERK_Y": 40.0,
    "MC_AST_TRES_R": 0.10,
    "MC_AST_TRES_P": 0.10,
    "MC_AST_TRES_Y": 0.05,
    "MC_AST_TAU_R": 0.0,
    "MC_AST_TAU_P": 0.0,
    "MC_AST_TAU_Y": 0.0,
    "MC_AST_SLEW_R": 0.0,
    "MC_AST_SLEW_P": 0.0,
    "MC_AST_SLEW_Y": 0.0,
    "MC_AST_DU_R": 0.0,
    "MC_AST_DU_P": 0.0,
    "MC_AST_DU_Y": 0.0,
    "MC_AST_SBD_R": 0.0,
    "MC_AST_SBD_P": 0.0,
    "MC_AST_SBD_Y": 0.0,
    "MC_AST_TRK_B": 0.0,
    "MC_AST_RFF": 1.0,
    "MC_AST_RFF_RP": 1.0,
    "MC_AST_GYRO": 1.0,
    "MC_AST_DT_MIN": 0.0005,
    "MC_AST_DT_MAX": 0.005,
    "MC_AST_REC_ERR": 1.0,
    "MC_BAT_SCALE_EN": 0,
    "MIS_TKO_ALT_MAX": 1.0,
}


@dataclass(frozen=True)
class GridConfig:
    move_s: float
    hold_s: float
    accel_scale: float

    @property
    def tag(self):
        return (
            f"m{self.move_s:.1f}_h{self.hold_s:.1f}_aff{self.accel_scale:.2f}"
            .replace(".", "p")
        )


def parse_config(value):
    parts = value.split(",")

    if len(parts) != 3:
        raise argparse.ArgumentTypeError("config must be move,hold,accel_scale")

    return GridConfig(*(float(part) for part in parts))


def latest_ulog(start_time):
    candidates = [path for path in LOG_ROOT.glob("**/*.ulg") if path.stat().st_mtime >= start_time]

    if not candidates:
        raise RuntimeError("no new ULog found")

    return max(candidates, key=lambda path: path.stat().st_mtime)


def default_sitl_parameter_overrides(controller):
    overrides = {"SDLOG_PROFILE": ACCEPTANCE_LOG_PROFILE}

    if controller == 3:
        overrides.update(UAV985_ASTSMC_CARD)

    return overrides


def effective_parameter_overrides(controller, explicit_overrides=None):
    overrides = default_sitl_parameter_overrides(controller)
    overrides.update(explicit_overrides or {})
    return overrides


def remove_persistent_params():
    for name in ("parameters.bson", "parameters_backup.bson"):
        path = ROOTFS / name

        if path.exists():
            path.unlink()


def stop_process_group(process):
    if process.poll() is None:
        try:
            if process.stdin:
                process.stdin.write("shutdown\n")
                process.stdin.flush()
            process.wait(timeout=20)

        except (BrokenPipeError, subprocess.TimeoutExpired):
            pass

    # Gazebo can outlive the make/PX4 process while remaining in its process
    # group, so clean the group even after the foreground process exits.
    for sig, delay in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 0.5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return

        time.sleep(delay)

    if process.poll() is None:
        process.wait(timeout=5)


def run_command(command, **kwargs):
    completed = subprocess.run(command, cwd=ROOT, text=True, **kwargs)

    if completed.returncode != 0:
        raise RuntimeError(f"command failed with {completed.returncode}: {' '.join(command)}")

    return completed


def run_one(config, args):
    remove_persistent_params()
    start_time = time.time()

    env = os.environ.copy()
    env["PX4_GZ_HEADLESS"] = "1"
    env["PX4_PARAM_MC_RATE_CTRL_T"] = str(args.controller)
    parameter_overrides = effective_parameter_overrides(
        args.controller, getattr(args, "parameter_overrides", {})
    )

    for name, value in parameter_overrides.items():
        env[f"PX4_PARAM_{name}"] = str(value)

    print(f"\n== start target={args.sitl_target} controller={args.controller} {config.tag}", flush=True)
    print("parameter_overrides:", flush=True)

    for name, value in sorted(parameter_overrides.items()):
        print(f"  {name}={value}", flush=True)

    sitl = subprocess.Popen(
        ["make", "px4_sitl", args.sitl_target],
        cwd=ROOT,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    flight_error = None

    try:
        time.sleep(args.startup_wait)
        flight_command = [
            sys.executable,
            "tmp_mpc_logs/offboard_square_smooth.py",
            "--connection",
            args.connection,
            "--takeoff-hold",
            str(args.takeoff_hold),
            "--move",
            str(config.move_s),
            "--hold",
            str(config.hold_s),
            "--side",
            str(args.side),
            "--altitude",
            str(args.altitude),
            "--accel-ff",
            "--accel-scale",
            str(config.accel_scale),
            "--yaw-step-deg",
            str(args.yaw_step_deg),
            "--yaw-hold",
            str(args.yaw_hold),
            "--sitl-ok",
            "--expected-controller",
            str(args.controller),
        ]
        run_command(flight_command, check=True)

    except Exception as error:
        flight_error = error

    finally:
        stop_process_group(sitl)

    destination = None

    try:
        source_log = latest_ulog(start_time)
        model_tag = args.sitl_target.removeprefix("gz_")
        destination = OUTPUT_DIR / f"{model_tag}_ctrl{args.controller}_grid_{config.tag}_{source_log.stem}.ulg"
        shutil.copy2(source_log, destination)
        print(f"log={destination}", flush=True)

    except RuntimeError:
        if flight_error is None:
            raise

    if flight_error is not None:
        raise flight_error

    analyze_command = [
        sys.executable,
        "tmp_mpc_logs/analyze_offboard_square.py",
        "--compact",
        "--takeoff-hold",
        str(args.takeoff_hold),
        "--move",
        str(config.move_s),
        "--hold",
        str(config.hold_s),
        "--side",
        str(args.side),
        "--altitude",
        str(args.altitude),
        "--yaw-step-deg",
        str(args.yaw_step_deg),
        "--yaw-hold",
        str(args.yaw_hold),
        str(destination),
    ]
    run_command(analyze_command, check=True)

    return destination


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        action="append",
        type=parse_config,
        help="move,hold,accel_scale. Can be repeated.",
    )
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--takeoff-hold", type=float, default=8.0)
    parser.add_argument("--side", type=float, default=0.6)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--yaw-step-deg", type=float, default=0.0)
    parser.add_argument("--yaw-hold", type=float, default=3.0)
    parser.add_argument("--startup-wait", type=float, default=35.0)
    parser.add_argument("--controller", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--sitl-target", choices=("gz_uav985", "gz_uav985_flow"), default="gz_uav985")
    args = parser.parse_args()

    configs = args.config or [
        GridConfig(3.0, 2.0, 0.70),
        GridConfig(3.0, 2.0, 0.75),
        GridConfig(3.2, 2.0, 0.70),
        GridConfig(3.2, 2.0, 0.75),
    ]

    logs = []

    for config in configs:
        logs.append(run_one(config, args))

    print("\ncreated logs:")

    for log in logs:
        print(log)


if __name__ == "__main__":
    main()
