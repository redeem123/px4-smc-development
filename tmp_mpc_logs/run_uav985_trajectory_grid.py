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

    print(f"\n== start target={args.sitl_target} controller={args.controller} {config.tag}", flush=True)
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

    finally:
        stop_process_group(sitl)

    source_log = latest_ulog(start_time)
    model_tag = args.sitl_target.removeprefix("gz_")
    destination = OUTPUT_DIR / f"{model_tag}_ctrl{args.controller}_grid_{config.tag}_{source_log.stem}.ulg"
    shutil.copy2(source_log, destination)
    print(f"log={destination}", flush=True)

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
    parser.add_argument("--takeoff-hold", type=float, default=7.0)
    parser.add_argument("--side", type=float, default=0.6)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--yaw-step-deg", type=float, default=0.0)
    parser.add_argument("--yaw-hold", type=float, default=3.0)
    parser.add_argument("--startup-wait", type=float, default=35.0)
    parser.add_argument("--controller", type=int, choices=(1, 2), default=1)
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
