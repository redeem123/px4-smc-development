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
    if process.poll() is not None:
        return

    try:
        if process.stdin:
            process.stdin.write("shutdown\n")
            process.stdin.flush()
        process.wait(timeout=20)
        return

    except (BrokenPipeError, subprocess.TimeoutExpired):
        pass

    for sig in (signal.SIGINT, signal.SIGTERM):
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return

        try:
            process.wait(timeout=8)
            return
        except subprocess.TimeoutExpired:
            continue

    if process.poll() is None:
        os.killpg(process.pid, signal.SIGKILL)
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

    print(f"\n== start {config.tag}", flush=True)
    sitl = subprocess.Popen(
        ["make", "px4_sitl", "gz_uav985"],
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
        ]
        run_command(flight_command, check=True)

    finally:
        stop_process_group(sitl)

    source_log = latest_ulog(start_time)
    destination = OUTPUT_DIR / f"uav985_mpc_grid_{config.tag}_{source_log.stem}.ulg"
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
    parser.add_argument("--startup-wait", type=float, default=35.0)
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
