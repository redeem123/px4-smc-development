#!/usr/bin/env python3

"""Run a fixed read-only PX4 health profile over the MAVLink NSH tunnel."""

import argparse
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Tools.mavlink_shell import MavlinkSerialPort


COMMANDS = (
    "ver all",
    "commander check",
    "commander status",
    "listener health_report 1",
    "listener failsafe_flags 1",
    "listener failure_detector_status 1",
    "listener vehicle_attitude 1",
    "listener battery_status 1",
    "listener sensor_optical_flow 1",
    "listener vehicle_optical_flow 1",
    "listener rate_ctrl_status 1",
    "listener astsmc_safety_status 1",
    "listener vehicle_land_detected 1",
)


def read_until(serial_port, marker, timeout):
    deadline = time.monotonic() + timeout
    output = ""

    while time.monotonic() < deadline:
        chunk = serial_port.read(4096)
        if chunk:
            output += chunk
            if marker in output:
                return output
        else:
            time.sleep(0.01)

    raise TimeoutError(f"timeout waiting for {marker!r}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    serial_port = MavlinkSerialPort(args.port, args.baud, devnum=10)
    serial_port.mav.port.write_timeout = 2

    try:
        serial_port.write("\n")
        read_until(serial_port, "nsh>", 5.0)

        for command in COMMANDS:
            serial_port.write(command + "\n")
            output = read_until(serial_port, "nsh>", 8.0)
            print(f"\n===== {command} =====")
            print(output.strip())
    finally:
        serial_port.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
