#!/usr/bin/env python3
import argparse
import math
import time

from pymavlink import mavutil


POSITION_ONLY_TYPEMASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def send_position(master, target):
    x, y, z = target
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1_000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_TYPEMASK,
        x,
        y,
        z,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def latest_position(master, timeout=0.05):
    deadline = time.monotonic() + timeout
    latest = None

    while time.monotonic() < deadline:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)

        if msg is None:
            time.sleep(0.005)
            continue

        latest = (float(msg.x), float(msg.y), float(msg.z))

    return latest


def command_long(master, command, params, label, timeout=5.0):
    mavlink_params = [float(param) for param in params]
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *mavlink_params,
    )

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)

        if msg is not None and msg.command == command:
            print(f"{label} -> ack command={msg.command} result={msg.result}", flush=True)
            return msg.result

    raise RuntimeError(f"timeout waiting for {label} ack")


def set_message_interval(master, message_id, hz):
    interval_us = int(1_000_000 / hz)
    command_long(
        master,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        [message_id, interval_us, 0, 0, 0, 0, 0],
        f"message_interval_{message_id}",
        timeout=2.0,
    )


def set_mode(master, mode_name):
    master.set_mode(mode_name)
    deadline = time.monotonic() + 5.0

    while time.monotonic() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)

        if msg is not None and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"setting {mode_name} -> ack command={msg.command} result={msg.result}", flush=True)
            return msg.result

    raise RuntimeError(f"timeout waiting for setting {mode_name} ack")


def arm(master):
    return command_long(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [1, 0, 0, 0, 0, 0, 0],
        "arming",
    )


def distance(position, target):
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(position, target)))


def hold_waypoint(master, name, target, hold_s, tolerance):
    print(f"waypoint {name}:", flush=True)
    start = time.monotonic()
    last_print = 0.0
    last_pos = None

    while time.monotonic() - start < hold_s:
        send_position(master, target)
        pos = latest_position(master)

        if pos is not None:
            last_pos = pos

            if time.monotonic() - last_print > 0.5:
                print(
                    f"  pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) d={distance(pos, target):.2f}",
                    flush=True,
                )
                last_print = time.monotonic()

        time.sleep(0.05)

    if last_pos is None:
        print(f"  reached/timeout {name}: no local position", flush=True)

    else:
        print(
            f"  reached/timeout {name}: pos=({last_pos[0]:.2f},{last_pos[1]:.2f},{last_pos[2]:.2f}) "
            f"d={distance(last_pos, target):.3f}",
            flush=True,
        )


def wait_landed(master, timeout=20.0):
    deadline = time.monotonic() + timeout
    latest_pos = None
    latest_landed = None

    while time.monotonic() < deadline:
        msg = master.recv_match(type=["LOCAL_POSITION_NED", "EXTENDED_SYS_STATE", "HEARTBEAT"], blocking=True, timeout=0.5)

        if msg is None:
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            latest_pos = (float(msg.x), float(msg.y), float(msg.z))

        elif msg.get_type() == "EXTENDED_SYS_STATE":
            latest_landed = msg.landed_state

        elif msg.get_type() == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if not armed and latest_landed == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                if latest_pos:
                    print(
                        f"landed/disarmed pos=({latest_pos[0]:.2f},{latest_pos[1]:.2f},{latest_pos[2]:.2f}) landed={latest_landed}",
                        flush=True,
                    )

                else:
                    print(f"landed/disarmed landed={latest_landed}", flush=True)

                return

    raise RuntimeError("timeout waiting for landed/disarmed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--hold", type=float, default=4.0)
    parser.add_argument("--takeoff-hold", type=float, default=8.0)
    parser.add_argument("--tolerance", type=float, default=0.08)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.connection)
    hb = master.wait_heartbeat(timeout=10)
    print(f"heartbeat target_system={master.target_system} target_component={master.target_component}", flush=True)

    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)

    takeoff = (0.0, 0.0, -2.0)

    for _ in range(30):
        send_position(master, takeoff)
        time.sleep(0.05)

    set_mode(master, "OFFBOARD")
    arm(master)

    hold_waypoint(master, "takeoff/settle", takeoff, args.takeoff_hold, args.tolerance)

    for name, target in [
        ("east", (0.6, 0.0, -2.0)),
        ("north", (0.6, 0.6, -2.0)),
        ("west", (0.0, 0.6, -2.0)),
        ("home", (0.0, 0.0, -2.0)),
    ]:
        hold_waypoint(master, name, target, args.hold, args.tolerance)

    print("switching LAND", flush=True)
    set_mode(master, "LAND")
    wait_landed(master)
    print("flight script done", flush=True)


if __name__ == "__main__":
    main()
