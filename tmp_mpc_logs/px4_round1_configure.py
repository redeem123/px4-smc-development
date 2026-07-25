#!/usr/bin/env python3

"""Read and apply the recovery-safe ASTSMC real-flight gain card."""

import argparse
import json
import math
import os
import struct
import sys
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


PARAMETERS = (
    ("MC_AST_CFG", 0.0),
    ("MC_AST_J_R", 0.01),
    ("MC_AST_J_P", 0.01),
    ("MC_AST_J_Y", 0.050951),
    ("MC_AST_EFF_R", 0.56),
    ("MC_AST_EFF_P", 0.53),
    ("MC_AST_EFF_Y", 0.80),
    ("MC_AST_K1_R", 4.5),
    ("MC_AST_K1_P", 4.5),
    ("MC_AST_K1_Y", 1.5),
    ("MC_AST_K2_R", 1.0),
    ("MC_AST_K2_P", 1.0),
    ("MC_AST_K2_Y", 1.5),
    ("MC_AST_TMAX_R", 0.20),
    ("MC_AST_TMAX_P", 0.20),
    ("MC_AST_TMAX_Y", 0.15),
    ("MC_AST_RACC_R", 10.0),
    ("MC_AST_RACC_P", 10.0),
    ("MC_AST_RACC_Y", 5.0),
    ("MC_AST_RJERK_R", 25.0),
    ("MC_AST_RJERK_P", 25.0),
    ("MC_AST_RJERK_Y", 40.0),
    ("MC_AST_TRES_R", 0.10),
    ("MC_AST_TRES_P", 0.10),
    ("MC_AST_TRES_Y", 0.10),
    ("MC_AST_RP_EXT", 0.10),
    ("MC_AST_RP_K1_B", 1.50),
    ("MC_AST_RP_AIR", 1.0),
    ("MC_AST_YAW_EXT", 0.0),
    ("MC_AST_TAU_R", 0.0),
    ("MC_AST_TAU_P", 0.0),
    ("MC_AST_TAU_Y", 0.0),
    ("MC_AST_SLEW_R", 0.0),
    ("MC_AST_SLEW_P", 0.0),
    ("MC_AST_SLEW_Y", 0.0),
    ("MC_AST_DU_R", 1.0),
    ("MC_AST_DU_P", 1.0),
    ("MC_AST_DU_Y", 0.0),
    ("MC_AST_SBD_R", 0.0),
    ("MC_AST_SBD_P", 0.0),
    ("MC_AST_SBD_Y", 0.0),
    ("MC_AST_TRK_B", 0.75),
    ("MC_AST_RFF", 1.0),
    ("MC_AST_RFF_RP", 0.50),
    ("MC_AST_GYRO", 1.0),
    ("MC_AST_DT_MIN", 0.0005),
    ("MC_AST_DT_MAX", 0.005),
    ("MC_AST_REC_ERR", 1.0),
    ("MC_AST_QK1_ERR", 0.50),
    ("MC_AST_REL_ERR", 0.20),
    ("MC_AST_REL_RATE", 20.0),
    ("MC_AST_TRIM_CMD", 0.15),
    ("MC_AST_TRIM_ERR", 0.20),
    ("MC_AST_TRIM_ACC", 1.0),
    ("MC_AST_TRIM_DWL", 0.50),
    ("MC_AST_TRIM_TC", 20.0),
    ("MC_BAT_SCALE_EN", 0.0),
    ("MC_AIRMODE", 0.0),
    ("MC_RATE_CTRL_T", 3.0),
)

AXES = (("roll", "R"), ("pitch", "P"), ("yaw", "Y"))
POSITIVE_PREFIXES = (
    "MC_AST_J_",
    "MC_AST_EFF_",
    "MC_AST_K1_",
    "MC_AST_K2_",
    "MC_AST_TMAX_",
    "MC_AST_RACC_",
    "MC_AST_RJERK_",
    "MC_AST_TRES_",
)
NONNEGATIVE_PREFIXES = (
    "MC_AST_TAU_",
    "MC_AST_SLEW_",
    "MC_AST_DU_",
    "MC_AST_SBD_",
    "MC_AST_REL_",
    "MC_AST_TRIM_",
    "MC_AST_RP_EXT",
    "MC_AST_RP_K1_B",
)


def validate_card(parameters):
    card = {}

    for name, value in parameters:
        if name in card:
            raise ValueError(f"duplicate card parameter {name}")
        if not math.isfinite(float(value)):
            raise ValueError(f"non-finite card parameter {name}")
        card[name] = float(value)

    if card.get("MC_AST_CFG") != 0.0:
        raise ValueError("target MC_AST_CFG must remain 0 until final acknowledgment")
    if card.get("MC_RATE_CTRL_T") != 3.0:
        raise ValueError("target MC_RATE_CTRL_T must select Type 3")
    if card.get("MC_BAT_SCALE_EN") != 0.0:
        raise ValueError("Type 3 requires MC_BAT_SCALE_EN=0")
    if card.get("MC_AIRMODE") != 0.0:
        raise ValueError("Type-3 headroom card requires global MC_AIRMODE=0")
    if card.get("MC_AST_RP_AIR") not in (0.0, 1.0):
        raise ValueError("MC_AST_RP_AIR must be exactly 0 or 1")

    for name, value in card.items():
        if name.startswith(POSITIVE_PREFIXES) and value <= 0.0:
            raise ValueError(f"{name} must be positive")
        if name.startswith(NONNEGATIVE_PREFIXES) and value < 0.0:
            raise ValueError(f"{name} must be non-negative")

    if not 0.0 <= card["MC_AST_TRK_B"] <= 1.0:
        raise ValueError("MC_AST_TRK_B must be in [0, 1]")
    if not 0.0 <= card["MC_AST_RFF"] <= 1.0:
        raise ValueError("MC_AST_RFF must be in [0, 1]")
    if not 0.0 <= card["MC_AST_RFF_RP"] <= 1.0:
        raise ValueError("MC_AST_RFF_RP must be in [0, 1]")
    if not 0.0 <= card["MC_AST_GYRO"] <= 1.0:
        raise ValueError("MC_AST_GYRO must be in [0, 1]")
    if card["MC_AST_DT_MIN"] <= 0.0 or card["MC_AST_DT_MAX"] < card["MC_AST_DT_MIN"]:
        raise ValueError("invalid Type-3 timing interval")
    if card["MC_AST_REC_ERR"] <= 0.0:
        raise ValueError("MC_AST_REC_ERR must be positive")
    if not 0.05 <= card["MC_AST_QK1_ERR"] <= card["MC_AST_REC_ERR"]:
        raise ValueError("MC_AST_QK1_ERR must be in [0.05, MC_AST_REC_ERR]")
    if not 0.05 <= card["MC_AST_REL_ERR"] <= card["MC_AST_REC_ERR"]:
        raise ValueError("MC_AST_REL_ERR must be in [0.05, MC_AST_REC_ERR]")
    if card["MC_AST_REL_RATE"] <= 0.0:
        raise ValueError("MC_AST_REL_RATE must be positive")
    if not 0.45 <= card["MC_AST_EFF_R"] <= 0.65 or not 0.45 <= card["MC_AST_EFF_P"] <= 0.65:
        raise ValueError("roll/pitch authority model must match the accepted PID identification envelope")
    if not 0.0 <= card["MC_AST_RP_EXT"] <= 0.10:
        raise ValueError("MC_AST_RP_EXT must be in [0.00, 0.10]")
    if not 0.0 <= card["MC_AST_RP_K1_B"] <= 1.50:
        raise ValueError("MC_AST_RP_K1_B must be in [0.00, 1.50]")
    if card["MC_AST_RP_K1_B"] > 0.0 and not card["MC_AST_QK1_ERR"] < card["MC_AST_REC_ERR"]:
        raise ValueError("MC_AST_RP_K1_B requires MC_AST_QK1_ERR below MC_AST_REC_ERR")
    for suffix in ("R", "P"):
        if card[f"MC_AST_TRES_{suffix}"] + card["MC_AST_RP_EXT"] > card[f"MC_AST_TMAX_{suffix}"] + 1e-6:
            raise ValueError(f"roll/pitch recovery reserve exceeds MC_AST_TMAX_{suffix}")
    if card["MC_AST_TRES_Y"] > 0.10 + 1e-6:
        raise ValueError("yaw residual reserve exceeds the log-834 allocation-safe bound")
    if not 0.0 <= card["MC_AST_YAW_EXT"] <= 0.05:
        raise ValueError("MC_AST_YAW_EXT must be in [0.00, 0.05]")
    if card["MC_AST_TRES_Y"] + card["MC_AST_YAW_EXT"] > card["MC_AST_TMAX_Y"] + 1e-6:
        raise ValueError("governed yaw reserve exceeds MC_AST_TMAX_Y")
    if not 4.0 <= card["MC_AST_K1_R"] <= 5.0 or not 4.0 <= card["MC_AST_K1_P"] <= 5.0:
        raise ValueError("roll/pitch K1 must match the log-825 corrective-authority envelope")
    if not 0.02 <= card["MC_AST_TRIM_CMD"] <= 0.5:
        raise ValueError("MC_AST_TRIM_CMD must be in [0.02, 0.5]")
    if not 0.02 <= card["MC_AST_TRIM_ERR"] <= 0.5:
        raise ValueError("MC_AST_TRIM_ERR must be in [0.02, 0.5]")
    if not 0.1 <= card["MC_AST_TRIM_ACC"] <= 10.0:
        raise ValueError("MC_AST_TRIM_ACC must be in [0.1, 10]")
    if not 0.1 <= card["MC_AST_TRIM_DWL"] <= 5.0:
        raise ValueError("MC_AST_TRIM_DWL must be in [0.1, 5]")
    if not 1.0 <= card["MC_AST_TRIM_TC"] <= 100.0:
        raise ValueError("MC_AST_TRIM_TC must be in [1, 100]")

    for axis, suffix in AXES:
        residual_torque = card[f"MC_AST_TRES_{suffix}"]
        total_torque = card[f"MC_AST_TMAX_{suffix}"]

        if residual_torque > total_torque:
            raise ValueError(f"MC_AST_TRES_{suffix} exceeds MC_AST_TMAX_{suffix}")

        reference_feedforward = card["MC_AST_RFF"]

        if axis in ("roll", "pitch"):
            reference_feedforward *= card["MC_AST_RFF_RP"]

        control_gain = card[f"MC_AST_EFF_{suffix}"] / card[f"MC_AST_J_{suffix}"]
        nominal_acceleration_demand = (
            reference_feedforward * card[f"MC_AST_RACC_{suffix}"] / control_gain
        )
        nominal_torque_partition = total_torque - residual_torque

        if axis in ("roll", "pitch") and nominal_acceleration_demand > nominal_torque_partition + 1e-6:
            raise ValueError(
                f"{axis} reference feedforward demand {nominal_acceleration_demand:g} "
                f"exceeds nominal partition {nominal_torque_partition:g}"
            )

    return card


def card_authority(card):
    result = {}

    for axis, suffix in AXES:
        control_gain = card[f"MC_AST_EFF_{suffix}"] / card[f"MC_AST_J_{suffix}"]
        residual_torque = card[f"MC_AST_TRES_{suffix}"]
        total_torque = card[f"MC_AST_TMAX_{suffix}"]
        reference_feedforward = card["MC_AST_RFF"]

        if axis in ("roll", "pitch"):
            reference_feedforward *= card["MC_AST_RFF_RP"]

        governed_residual_torque = residual_torque

        if axis in ("roll", "pitch"):
            governed_residual_torque = min(
                total_torque,
                residual_torque + card["MC_AST_RP_EXT"],
            )
        elif axis == "yaw":
            governed_residual_torque = min(
                total_torque,
                residual_torque + card["MC_AST_YAW_EXT"],
            )

        result[axis] = {
            "control_gain": control_gain,
            "residual_reaching_authority": control_gain * residual_torque,
            "maximum_governed_residual_reaching_authority": (
                control_gain * governed_residual_torque
            ),
            "nominal_torque_partition": total_torque - residual_torque,
            "nominal_acceleration_demand": (
                reference_feedforward * card[f"MC_AST_RACC_{suffix}"] / control_gain
            ),
        }

    return result


def print_card_authority(card):
    for axis, values in card_authority(card).items():
        print(
            f"target {axis}: EFF/J={values['control_gain']:.6f} "
            f"residual_reaching={values['residual_reaching_authority']:.6f}rad/s^2 "
            f"governed_max={values['maximum_governed_residual_reaching_authority']:.6f}rad/s^2 "
            f"nominal_demand={values['nominal_acceleration_demand']:.6f} "
            f"nominal_partition={values['nominal_torque_partition']:.6f}"
        )


def write_snapshot(path, before, target, verified=None):
    final_target = dict(target)
    final_target["MC_AST_CFG"] = 1.0
    document = {
        "before": {name: {"value": value, "type": param_type} for name, (value, param_type) in before.items()},
        "target": final_target,
        "authority": card_authority(target),
        "verified": verified,
    }
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parameter_name(message):
    name = message.param_id
    if isinstance(name, bytes):
        name = name.decode("ascii")
    return name.rstrip("\x00")


def decode_parameter(value, param_type):
    formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
    }
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type not in formats:
        raise ValueError(f"unsupported MAVLink parameter type {param_type}")
    return struct.unpack(formats[param_type], struct.pack(">f", value))[0]


def encode_parameter(value, param_type):
    formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
    }
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type not in formats:
        raise ValueError(f"unsupported MAVLink parameter type {param_type}")
    return struct.unpack(">f", struct.pack(formats[param_type], int(value)))[0]


def read_parameter(master, name, timeout=3.0):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and parameter_name(message) == name:
            param_type = int(message.param_type)
            return decode_parameter(message.param_value, param_type), param_type

    raise TimeoutError(f"timeout reading {name}")


def write_parameter(master, name, value, param_type, timeout=3.0):
    encoded_value = encode_parameter(value, param_type)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        encoded_value,
        param_type,
    )
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None or parameter_name(message) != name:
            continue

        actual = decode_parameter(message.param_value, int(message.param_type))
        if not math.isclose(actual, value, rel_tol=0.0, abs_tol=1e-4):
            raise RuntimeError(f"{name} acknowledged {actual:g}, expected {value:g}")
        return actual

    raise TimeoutError(f"timeout writing {name}")


def reboot(master, timeout=3.0):
    command = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if message is None or message.command != command:
            continue
        if message.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"reboot rejected with MAV_RESULT {message.result}")
        print("reboot accepted")
        return

    raise TimeoutError("timeout waiting for reboot acknowledgement")


def card_with_yaw_extension(yaw_extension):
    return tuple(
        (name, yaw_extension if name == "MC_AST_YAW_EXT" else value)
        for name, value in PARAMETERS
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--reboot", action="store_true")
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--yaw-extension", type=float, default=0.0)
    args = parser.parse_args()

    if args.reboot and not args.apply:
        parser.error("--reboot requires --apply")

    try:
        target_parameters = card_with_yaw_extension(args.yaw_extension)
        target = validate_card(target_parameters)
    except ValueError as error:
        parser.error(str(error))

    print_card_authority(target)
    master = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False)
    heartbeat = master.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise TimeoutError("timeout waiting for heartbeat")

    armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(
        f"heartbeat system={master.target_system} component={master.target_component} "
        f"armed={str(armed).lower()} status={heartbeat.system_status}"
    )
    if armed:
        raise RuntimeError("refusing parameter access while vehicle is armed")

    current = {}
    for name, _ in target_parameters:
        current[name] = read_parameter(master, name)
        print(f"before {name}={current[name][0]:g} type={current[name][1]}")

    if args.snapshot is not None:
        write_snapshot(args.snapshot, current, target)
        print(f"saved pre-change snapshot to {args.snapshot}")

    if not args.apply:
        return

    # Keep Mode 3 inactive and the card unacknowledged until every dependent
    # value has been written. This avoids running a partially updated card.
    write_parameter(master, "MC_RATE_CTRL_T", 0.0, current["MC_RATE_CTRL_T"][1])
    print("wrote MC_RATE_CTRL_T=0")
    write_parameter(master, "MC_AST_CFG", 0.0, current["MC_AST_CFG"][1])
    print("wrote MC_AST_CFG=0")

    for name, value in target_parameters:
        if name not in ("MC_AST_CFG", "MC_RATE_CTRL_T"):
            actual = write_parameter(master, name, value, current[name][1])
            print(f"wrote {name}={actual:g}")

    cfg_type = current["MC_AST_CFG"][1]
    actual = write_parameter(master, "MC_AST_CFG", 1.0, cfg_type)
    print(f"wrote MC_AST_CFG={actual:g} (card acknowledged last)")
    controller_type = current["MC_RATE_CTRL_T"][1]
    actual = write_parameter(master, "MC_RATE_CTRL_T", 3.0, controller_type)
    print(f"wrote MC_RATE_CTRL_T={actual:g} (Mode 3 selected after card acknowledgment)")

    verified = {}
    for name, expected in target_parameters:
        expected = 1.0 if name == "MC_AST_CFG" else expected
        actual, param_type = read_parameter(master, name)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-4):
            raise RuntimeError(f"verification failed: {name}={actual:g}, expected {expected:g}")
        verified[name] = {"value": actual, "type": param_type}
        print(f"verified {name}={actual:g}")

    if args.snapshot is not None:
        write_snapshot(args.snapshot, current, target, verified)
        print(f"saved verified snapshot to {args.snapshot}")

    if args.reboot:
        reboot(master)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
