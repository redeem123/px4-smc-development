#!/usr/bin/env python3
import argparse
import math
import struct
import time

from pymavlink import mavutil


POSITION_VELOCITY_TYPEMASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

POSITION_VELOCITY_ACCELERATION_TYPEMASK = mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE


def now_ms():
    return int(time.time() * 1_000) & 0xFFFFFFFF


def looks_like_sitl_connection(connection):
    return (
        "127.0.0.1" in connection
        or "localhost" in connection
    )


def send_trajectory(
    master,
    position,
    velocity=(0.0, 0.0, 0.0),
    acceleration=(0.0, 0.0, 0.0),
    yaw=0.0,
    acceleration_feedforward=False,
):
    x, y, z = position
    vx, vy, vz = velocity
    ax, ay, az = acceleration
    master.mav.set_position_target_local_ned_send(
        now_ms(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_VELOCITY_ACCELERATION_TYPEMASK if acceleration_feedforward else POSITION_VELOCITY_TYPEMASK,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        ax,
        ay,
        az,
        yaw,
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


def current_yaw(master, timeout=3.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)

        if msg is not None and math.isfinite(msg.yaw):
            return float(msg.yaw)

    raise RuntimeError("timeout waiting for initial attitude yaw")


def current_position(master, timeout=3.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)

        if msg is not None:
            position = (float(msg.x), float(msg.y), float(msg.z))

            if all(math.isfinite(value) for value in position):
                return position

    raise RuntimeError("timeout waiting for a finite local position")


def read_parameter(master, name, timeout=3.0):
    master.param_fetch_one(name)
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)

        if msg is None:
            continue

        parameter_name = msg.param_id.decode("ascii").rstrip("\x00") if isinstance(msg.param_id, bytes) else msg.param_id

        if parameter_name == name and math.isfinite(msg.param_value):
            if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
                return float(msg.param_value)

            if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                return float(struct.unpack("<i", struct.pack("<f", msg.param_value))[0])

            raise RuntimeError(f"unsupported MAVLink parameter type {msg.param_type} for {name}")

    raise RuntimeError(f"timeout reading parameter {name}")


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

            if msg.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                raise RuntimeError(f"{label} rejected with MAV_RESULT={msg.result}")

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

            if msg.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                raise RuntimeError(f"setting {mode_name} rejected with MAV_RESULT={msg.result}")

            return msg.result

    raise RuntimeError(f"timeout waiting for setting {mode_name} ack")


def arm(master):
    return command_long(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [1, 0, 0, 0, 0, 0, 0],
        "arming",
    )


def wait_armed(master, timeout=5.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)

        if msg is not None and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return

    raise RuntimeError("arming was acknowledged but armed state was not observed")


def distance(position, target):
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(position, target)))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def square_waypoints(side_m, hover_home):
    x, y, z = hover_home
    return [
        ("east", (x + side_m, y, z)),
        ("north", (x + side_m, y + side_m, z)),
        ("west", (x, y + side_m, z)),
        ("home", hover_home),
    ]


def minimum_jerk_profile(elapsed, duration):
    tau = min(max(elapsed / duration, 0.0), 1.0)
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau
    s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    ds_dtau = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
    d2s_dtau2 = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3
    return s, ds_dtau / duration, d2s_dtau2 / (duration * duration)


def blend(start, target, elapsed, duration):
    s, ds, d2s = minimum_jerk_profile(elapsed, duration)
    delta = [target[i] - start[i] for i in range(3)]
    position = tuple(start[i] + delta[i] * s for i in range(3))
    velocity = tuple(delta[i] * ds for i in range(3))
    acceleration = tuple(delta[i] * d2s for i in range(3))
    return position, velocity, acceleration


def hold_waypoint(master, name, target, hold_s, yaw, acceleration_feedforward=False):
    print(f"hold {name}:", flush=True)
    start_time = time.monotonic()
    last_print = 0.0
    last_pos = None

    while time.monotonic() - start_time < hold_s:
        send_trajectory(master, target, yaw=yaw, acceleration_feedforward=acceleration_feedforward)
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
        print(f"  hold done {name}: no local position", flush=True)

    else:
        print(
            f"  hold done {name}: pos=({last_pos[0]:.2f},{last_pos[1]:.2f},{last_pos[2]:.2f}) "
            f"d={distance(last_pos, target):.3f}",
            flush=True,
        )


def move_smooth(
    master,
    name,
    start_target,
    end_target,
    move_s,
    hold_s,
    yaw,
    acceleration_feedforward=False,
    acceleration_scale=1.0,
):
    print(f"smooth move {name}:", flush=True)
    start_time = time.monotonic()
    last_print = 0.0
    last_pos = None

    while time.monotonic() - start_time < move_s:
        elapsed = time.monotonic() - start_time
        position_sp, velocity_sp, acceleration_sp = blend(start_target, end_target, elapsed, move_s)
        acceleration_sp = tuple(axis_acceleration * acceleration_scale for axis_acceleration in acceleration_sp)
        send_trajectory(
            master,
            position_sp,
            velocity_sp,
            acceleration_sp,
            yaw=yaw,
            acceleration_feedforward=acceleration_feedforward,
        )
        pos = latest_position(master)

        if pos is not None:
            last_pos = pos

            if time.monotonic() - last_print > 0.5:
                print(
                    f"  sp=({position_sp[0]:.2f},{position_sp[1]:.2f},{position_sp[2]:.2f}) "
                    f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                    f"d_target={distance(pos, end_target):.2f} d_sp={distance(pos, position_sp):.2f}",
                    flush=True,
                )
                last_print = time.monotonic()

        time.sleep(0.05)

    hold_waypoint(master, name, end_target, hold_s, yaw, acceleration_feedforward=acceleration_feedforward)

    if last_pos is None:
        print(f"  move/hold done {name}: no local position", flush=True)


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
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--takeoff-hold", type=float, default=8.0)
    parser.add_argument("--move", type=float, default=3.2)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--side", type=float, default=0.6)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--accel-ff", dest="accel_ff", action="store_true")
    parser.add_argument("--no-accel-ff", dest="accel_ff", action="store_false")
    parser.set_defaults(accel_ff=True)
    parser.add_argument("--accel-scale", type=float, default=0.75)
    parser.add_argument("--yaw-step-deg", type=float, default=0.0)
    parser.add_argument("--yaw-hold", type=float, default=3.0)
    parser.add_argument("--sitl-ok", action="store_true")
    parser.add_argument("--real-flight-ok", action="store_true")
    parser.add_argument("--expected-controller", type=int, choices=(0, 1, 2))
    args = parser.parse_args()

    if args.sitl_ok == args.real_flight_ok:
        raise RuntimeError("select exactly one authorization: --sitl-ok or --real-flight-ok")

    if args.sitl_ok and not looks_like_sitl_connection(args.connection):
        raise RuntimeError("--sitl-ok is only valid for an explicit localhost connection")

    if args.real_flight_ok and args.expected_controller is None:
        raise RuntimeError("real flight requires --expected-controller 0, 1, or 2")

    if args.altitude <= 0.0 or args.side <= 0.0 or args.move <= 0.0 or args.hold <= 0.0:
        raise ValueError("altitude, side, move, and hold must be positive")

    if not 0.0 <= args.yaw_step_deg <= 90.0 or args.yaw_hold <= 0.0:
        raise ValueError("yaw step must be between 0 and 90 degrees and yaw hold must be positive")

    if args.real_flight_ok and (
        args.altitude > 1.0 or args.side > 0.5 or args.yaw_step_deg > 15.0
    ):
        raise RuntimeError("real-flight authorization is limited to 1m altitude, 0.5m side, and 15deg yaw steps")

    master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    if master.wait_heartbeat(timeout=10) is None:
        raise RuntimeError("timeout waiting for MAVLink heartbeat")

    print(f"heartbeat target_system={master.target_system} target_component={master.target_component}", flush=True)

    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)

    if args.expected_controller is not None:
        controller = read_parameter(master, "MC_RATE_CTRL_T")

        if not math.isclose(controller, args.expected_controller, abs_tol=0.01):
            raise RuntimeError(
                f"MC_RATE_CTRL_T={controller:g}, expected controller {args.expected_controller}"
            )

        print(f"verified MC_RATE_CTRL_T={controller:g}", flush=True)

        if args.expected_controller == 2:
            configuration = read_parameter(master, "MC_MSMC_CFG")

            if not math.isclose(configuration, 1.0, abs_tol=0.01):
                raise RuntimeError(f"MC_MSMC_CFG={configuration:g}, expected acknowledged model card 1")

            print("verified MC_MSMC_CFG=1", flush=True)

    start_position = current_position(master)
    yaw = current_yaw(master)
    print(
        f"relative start=({start_position[0]:.2f},{start_position[1]:.2f},{start_position[2]:.2f})",
        flush=True,
    )
    print(f"holding initial yaw={math.degrees(yaw):.1f}deg", flush=True)

    takeoff = (start_position[0], start_position[1], start_position[2] - args.altitude)

    for _ in range(30):
        send_trajectory(master, takeoff, yaw=yaw, acceleration_feedforward=args.accel_ff)
        time.sleep(0.05)

    armed = False
    landed = False

    try:
        set_mode(master, "OFFBOARD")
        arm(master)
        armed = True
        wait_armed(master)

        hold_waypoint(master, "takeoff/settle", takeoff, args.takeoff_hold, yaw,
                      acceleration_feedforward=args.accel_ff)

        current = takeoff

        for name, target in square_waypoints(args.side, takeoff):
            move_smooth(
                master,
                name,
                current,
                target,
                args.move,
                args.hold,
                yaw,
                acceleration_feedforward=args.accel_ff,
                acceleration_scale=args.accel_scale,
            )
            current = target

        if args.yaw_step_deg > 0.0:
            yaw_step = math.radians(args.yaw_step_deg)
            yaw_targets = (
                ("yaw_positive", wrap_angle(yaw + yaw_step)),
                ("yaw_negative", wrap_angle(yaw - yaw_step)),
                ("yaw_home", yaw),
            )

            for name, yaw_target in yaw_targets:
                print(f"yaw target {name}={math.degrees(yaw_target):.1f}deg", flush=True)
                hold_waypoint(
                    master,
                    name,
                    takeoff,
                    args.yaw_hold,
                    yaw_target,
                    acceleration_feedforward=args.accel_ff,
                )

        print("switching LAND", flush=True)
        set_mode(master, "LAND")
        wait_landed(master)
        landed = True
        print("smooth flight script done", flush=True)

    finally:
        if armed and not landed:
            print("flight interrupted after arming; requesting LAND", flush=True)

            try:
                set_mode(master, "LAND")
                wait_landed(master)
                landed = True

            except Exception as error:
                print(f"LAND request failed: {error}", flush=True)


if __name__ == "__main__":
    main()
