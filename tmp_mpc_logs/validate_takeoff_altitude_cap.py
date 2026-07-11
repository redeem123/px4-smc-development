#!/usr/bin/env python3
import argparse
import math
import time

from pymavlink import mavutil

from offboard_square_smooth import (
    arm,
    command_long,
    current_position,
    current_yaw,
    looks_like_sitl_connection,
    read_parameter,
    set_message_interval,
    set_mode,
    send_trajectory,
    wait_armed,
    wait_landed,
)


def current_global_altitude(master, timeout=5.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)

        if message is not None and math.isfinite(message.alt):
            return message.alt * 1e-3

    raise RuntimeError("timeout waiting for global altitude")


def monitor_takeoff(master, timeout, cap):
    deadline = time.monotonic() + timeout
    minimum_z = 0.0
    last_print = 0.0

    while time.monotonic() < deadline:
        message = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)

        if message is None or not math.isfinite(message.z):
            continue

        minimum_z = min(minimum_z, float(message.z))
        relative_altitude = -float(message.z)

        if time.monotonic() - last_print > 0.5:
            print(f"relative_altitude={relative_altitude:.3f}m", flush=True)
            last_print = time.monotonic()

        if relative_altitude > cap + 0.25:
            raise RuntimeError(
                f"takeoff exceeded cap: observed={relative_altitude:.3f}m cap={cap:.3f}m"
            )

    return -minimum_z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--requested-relative-altitude", type=float, default=2.0)
    parser.add_argument("--monitor", type=float, default=12.0)
    parser.add_argument("--sitl-ok", action="store_true")
    args = parser.parse_args()

    if not args.sitl_ok or not looks_like_sitl_connection(args.connection):
        raise RuntimeError("this destructive arming test requires --sitl-ok and an explicit localhost connection")

    if args.requested_relative_altitude <= 0.0 or args.monitor <= 0.0:
        raise ValueError("requested altitude and monitor duration must be positive")

    master = mavutil.mavlink_connection(args.connection)

    if master.wait_heartbeat(timeout=10) is None:
        raise RuntimeError("timeout waiting for MAVLink heartbeat")

    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
    cap = read_parameter(master, "MIS_TKO_ALT_MAX")

    if cap <= 0.0 or args.requested_relative_altitude <= cap + 0.25:
        raise RuntimeError(
            f"test request must exceed enabled cap by more than 0.25m: request={args.requested_relative_altitude} cap={cap}"
        )

    current_altitude = current_global_altitude(master)
    requested_altitude = current_altitude + args.requested_relative_altitude
    hold_position = current_position(master)
    hold_yaw = current_yaw(master)
    armed = False
    landed = False

    try:
        for _ in range(30):
            send_trajectory(master, hold_position, yaw=hold_yaw)
            time.sleep(0.05)

        set_mode(master, "OFFBOARD")
        arm(master)
        armed = True
        wait_armed(master)
        command_long(
            master,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, math.nan, math.nan, math.nan, requested_altitude],
            "QGC-style takeoff",
        )
        observed_altitude = monitor_takeoff(master, args.monitor, cap)

        if observed_altitude < cap - 0.25:
            raise RuntimeError(
                f"vehicle did not reach capped altitude: observed={observed_altitude:.3f}m cap={cap:.3f}m"
            )

        print(
            f"TAKEOFF CAP PASSED request={args.requested_relative_altitude:.3f}m "
            f"cap={cap:.3f}m observed_max={observed_altitude:.3f}m",
            flush=True,
        )

    finally:
        if armed and not landed:
            try:
                set_mode(master, "LAND")
                wait_landed(master)
                landed = True

            except Exception as error:
                print(f"LAND request failed: {error}", flush=True)


if __name__ == "__main__":
    main()
