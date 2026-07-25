#!/usr/bin/env python3

"""Read and apply the identified-model mode-2 (model-based SMC) card.

The card replaces the stale roll/pitch torque effectiveness and the obsolete
yaw inertia with the values identified from the stable PID excitation log, and
compensates the surface/reaching gains so that the flight-validated small-error
torque slope of the installed reduced roll/pitch card is preserved exactly.
"""

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


AXES = (("roll", "R"), ("pitch", "P"), ("yaw", "Y"))

# Installed card that flew logs 11_39_30 and 11_41_04 (reduced roll/pitch plus
# the EFF_Y=0.8 yaw correction). It is the behavioral reference, not a target.
LEGACY_PARAMETERS = {
    "MC_MSMC_J_R": 0.01,
    "MC_MSMC_J_P": 0.01,
    "MC_MSMC_J_Y": 0.02,
    "MC_MSMC_EFF_R": 1.0,
    "MC_MSMC_EFF_P": 1.0,
    "MC_MSMC_EFF_Y": 0.80,
    "MC_MSMC_C_R": 2.0,
    "MC_MSMC_C_P": 2.0,
    "MC_MSMC_C_Y": 2.5,
    "MC_MSMC_ETA_R": 4.5,
    "MC_MSMC_ETA_P": 4.5,
    "MC_MSMC_ETA_Y": 1.5,
    "MC_MSMC_BND_R": 0.5,
    "MC_MSMC_BND_P": 0.5,
    "MC_MSMC_BND_Y": 0.2,
    "MC_MSMC_KS_R": 1.0,
    "MC_MSMC_KS_P": 1.0,
    "MC_MSMC_KS_Y": 1.0,
    "MC_MSMC_ILIM_R": 0.20,
    "MC_MSMC_ILIM_P": 0.30,
    "MC_MSMC_ILIM_Y": 2.0,
    "MC_MSMC_TMAX_R": 0.20,
    "MC_MSMC_TMAX_P": 0.20,
    "MC_MSMC_TMAX_Y": 0.17,
}

# Identified from the stable PID excitation log; already installed as the Type-3
# card in px4_round1_configure.py.
IDENTIFIED_INERTIA = (0.01, 0.01, 0.050951)
IDENTIFIED_EFFECTIVENESS = (0.56, 0.53, 0.80)

# Parameter maximum of MC_MSMC_ILIM_*; the yaw integral clamp is truncated here.
INTEGRAL_LIMIT_MAXIMUM = 5.0

PARAMETERS = (
    ("MC_MSMC_CFG", 0.0),
    ("MC_MSMC_J_R", 0.01),
    ("MC_MSMC_J_P", 0.01),
    ("MC_MSMC_J_Y", 0.050951),
    ("MC_MSMC_EFF_R", 0.56),
    ("MC_MSMC_EFF_P", 0.53),
    ("MC_MSMC_EFF_Y", 0.80),
    # C/ETA/KS are the legacy gains scaled by the inverse J/EFF change:
    # roll 0.56, pitch 0.53, yaw 0.02/0.050951.
    ("MC_MSMC_C_R", 1.12),
    ("MC_MSMC_C_P", 1.06),
    ("MC_MSMC_C_Y", 0.981335),
    ("MC_MSMC_ETA_R", 2.52),
    ("MC_MSMC_ETA_P", 2.385),
    ("MC_MSMC_ETA_Y", 0.588801),
    ("MC_MSMC_BND_R", 0.5),
    ("MC_MSMC_BND_P", 0.5),
    ("MC_MSMC_BND_Y", 0.2),
    ("MC_MSMC_KS_R", 0.56),
    ("MC_MSMC_KS_P", 0.53),
    ("MC_MSMC_KS_Y", 0.392534),
    # Integral limits scaled by the inverse gain change so that the maximum
    # integral surface offset C*ILIM is preserved (yaw truncated at the
    # parameter maximum).
    ("MC_MSMC_ILIM_R", 0.357143),
    ("MC_MSMC_ILIM_P", 0.566038),
    ("MC_MSMC_ILIM_Y", 5.0),
    ("MC_MSMC_TMAX_R", 0.20),
    ("MC_MSMC_TMAX_P", 0.20),
    ("MC_MSMC_TMAX_Y", 0.17),
    ("MC_MSMC_RSPD_L", 0.0),
    ("MC_SMC_LPF", 40.0),
    ("MC_SMC_SLEW", 15.0),
    ("MC_RATE_CTRL_T", 2.0),
)

# Paper augmentation of Bouabdallah & Siegwart, ICRA 2005 (a509.pdf): the
# -alpha1^2*z1 attitude-surface term of eq. 33 and the -a2*x4*Omega / -a4*x2*Omega
# propulsion-group gyroscopic term of eq. 6. Both are additions to the card
# above; with the values below at zero the installed law is bit-identical to the
# flight-validated one, which is why they are written explicitly either way.
PAPER_DISABLED = (
    ("MC_MSMC_A1_R", 0.0),
    ("MC_MSMC_A1_P", 0.0),
    ("MC_MSMC_A1_Y", 0.0),
    ("MC_MSMC_JR", 0.0),
    ("MC_MSMC_ROTOR_K", 0.0),
    ("MC_MSMC_ROTOR_D", 0.0),
)

# alpha1 is the slope of the paper's attitude surface, which in PX4's cascade is
# the attitude-loop proportional gain that produced the rate setpoint. The
# installer reads MC_ROLL_P/MC_PITCH_P/MC_YAW_P and refuses to write a card that
# disagrees with them, so these are defaults rather than an assumption.
PAPER_ATTITUDE_SLOPE = (6.5, 6.5, 2.8)

# Estimated, not measured: propulsion-group inertia of one rotor (propeller plus
# motor bell) and the full-command propeller speed of the sqrt thrust map. The
# term is near zero in hover because the counter-rotating pairs cancel, so the
# estimate only sets the scale of a yaw-transient correction.
PAPER_ROTOR_INERTIA = 1.5e-5
PAPER_ROTOR_SPEED_GAIN = 1000.0

# Bitmask of motors that count negatively in eq. 7. The default is the PX4
# quad-X order (motors 3 and 4 turn against motors 1 and 2) and must be checked
# against the airframe before flight.
PAPER_ROTOR_DIRECTIONS = 0b1100

PAPER_ENABLED = (
    ("MC_MSMC_A1_R", PAPER_ATTITUDE_SLOPE[0]),
    ("MC_MSMC_A1_P", PAPER_ATTITUDE_SLOPE[1]),
    ("MC_MSMC_A1_Y", PAPER_ATTITUDE_SLOPE[2]),
    ("MC_MSMC_JR", PAPER_ROTOR_INERTIA),
    ("MC_MSMC_ROTOR_K", PAPER_ROTOR_SPEED_GAIN),
    ("MC_MSMC_ROTOR_D", float(PAPER_ROTOR_DIRECTIONS)),
)

ATTITUDE_SLOPE_SOURCES = ("MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P")

CARD_IDENTIFIED = "identified"
CARD_PAPER = "paper"
CARD_STRICT = "paper-strict"
CARD_MODES = (CARD_IDENTIFIED, CARD_PAPER, CARD_STRICT)

# Structural differences from eq. 30/32 that the flight card carries and the
# strict card removes. Everything else, including the identified model and the
# k1/k2 gains, is shared with the flight-validated card.
#
#   ILIM=0    clamps the error integral to exactly zero, so s = e is the surface
#             of eq. 30 rather than the augmented e + C*integral(e).
#   BND=0.01  is the parameter minimum and the closest reachable approximation
#             of the discontinuous sign(s2) of eq. 32.
#   LPF/SLEW  are chattering mitigations with no counterpart in the paper.
#   TMAX=1.0  is the parameter maximum, so the only remaining bound is the
#             normalized-torque saturation of the allocator itself. The paper
#             has no software torque limit; this is why --bench-locked is
#             mandatory to apply the card.
STRICT_BOUNDARY_LAYER = 0.01
STRICT_TORQUE_LIMIT = 1.0
STRICT_OVERRIDES = (
    ("MC_MSMC_TMAX_R", STRICT_TORQUE_LIMIT),
    ("MC_MSMC_TMAX_P", STRICT_TORQUE_LIMIT),
    ("MC_MSMC_TMAX_Y", STRICT_TORQUE_LIMIT),
    ("MC_MSMC_BND_R", STRICT_BOUNDARY_LAYER),
    ("MC_MSMC_BND_P", STRICT_BOUNDARY_LAYER),
    ("MC_MSMC_BND_Y", STRICT_BOUNDARY_LAYER),
    ("MC_MSMC_ILIM_R", 0.0),
    ("MC_MSMC_ILIM_P", 0.0),
    ("MC_MSMC_ILIM_Y", 0.0),
    ("MC_SMC_LPF", 0.0),
    ("MC_SMC_SLEW", 0.0),
)

POSITIVE_PREFIXES = (
    "MC_MSMC_J_",
    "MC_MSMC_EFF_",
    "MC_MSMC_C_",
    "MC_MSMC_BND_",
    "MC_MSMC_TMAX_",
)
NONNEGATIVE_PREFIXES = (
    "MC_MSMC_ETA_",
    "MC_MSMC_KS_",
    "MC_MSMC_ILIM_",
    "MC_MSMC_RSPD_L",
    "MC_MSMC_A1_",
    "MC_MSMC_JR",
    "MC_MSMC_ROTOR_",
    "MC_SMC_LPF",
    "MC_SMC_SLEW",
)

# Relative tolerance on the preserved small-error torque slope. The compensated
# gains are exact to about 1e-8; anything looser indicates an edited card.
SLOPE_TOLERANCE = 1e-6


def local_torque_slope(card, suffix):
    """Normalized torque per unit rate error inside the boundary layer."""
    return (
        card[f"MC_MSMC_J_{suffix}"]
        / card[f"MC_MSMC_EFF_{suffix}"]
        * (
            card[f"MC_MSMC_C_{suffix}"]
            + card[f"MC_MSMC_ETA_{suffix}"] / card[f"MC_MSMC_BND_{suffix}"]
            + card[f"MC_MSMC_KS_{suffix}"]
        )
    )


def card_parameters(mode):
    """Full card for one of CARD_MODES."""
    if mode not in CARD_MODES:
        raise ValueError(f"unknown card mode {mode}")

    parameters = PARAMETERS + (PAPER_DISABLED if mode == CARD_IDENTIFIED else PAPER_ENABLED)

    if mode != CARD_STRICT:
        return parameters

    overrides = dict(STRICT_OVERRIDES)
    return tuple((name, overrides.get(name, value)) for name, value in parameters)


def paper_terms_enabled(card):
    return any(card[name] != 0.0 for name, _ in PAPER_DISABLED)


def strict_terms_enabled(card):
    """True for a card carrying the paper's surface and reaching law.

    Detection uses only the two unambiguous markers, the nulled integral and the
    collapsed boundary layer, so that the remaining strict requirements are
    validated by name and report themselves instead of silently demoting the
    card to the flight branch.
    """
    return all(
        card[f"MC_MSMC_ILIM_{suffix}"] == 0.0
        and card[f"MC_MSMC_BND_{suffix}"] == STRICT_BOUNDARY_LAYER
        for _, suffix in AXES
    )


def validate_card(parameters):
    card = {}

    for name, value in parameters:
        if name in card:
            raise ValueError(f"duplicate card parameter {name}")
        if not math.isfinite(float(value)):
            raise ValueError(f"non-finite card parameter {name}")
        card[name] = float(value)

    if card.get("MC_MSMC_CFG") != 0.0:
        raise ValueError("target MC_MSMC_CFG must remain 0 until final acknowledgment")
    if card.get("MC_RATE_CTRL_T") != 2.0:
        raise ValueError("target MC_RATE_CTRL_T must select Type 2")

    for name, value in card.items():
        if name.startswith(POSITIVE_PREFIXES) and value <= 0.0:
            raise ValueError(f"{name} must be positive")
        if name.startswith(NONNEGATIVE_PREFIXES) and value < 0.0:
            raise ValueError(f"{name} must be non-negative")

    if card["MC_MSMC_RSPD_L"] != 0.0:
        raise ValueError(
            "MC_MSMC_RSPD_L must stay disabled until the rate-setpoint "
            "acceleration feedforward is separately validated"
        )

    for name, _ in PAPER_DISABLED:
        if name not in card:
            raise ValueError(f"{name} must be written explicitly, even when disabled")

    if card["MC_MSMC_ROTOR_D"] != float(int(card["MC_MSMC_ROTOR_D"])):
        raise ValueError("MC_MSMC_ROTOR_D must be an integer bitmask")

    if card["MC_MSMC_JR"] > 0.0:
        # Without a speed map and a direction mask the rotor term would either
        # be dead or sum every propeller with the same sign, which is worse than
        # leaving it off.
        if card["MC_MSMC_ROTOR_K"] <= 0.0:
            raise ValueError("MC_MSMC_JR requires a positive MC_MSMC_ROTOR_K")
        if card["MC_MSMC_ROTOR_D"] == 0.0:
            raise ValueError("MC_MSMC_JR requires a non-zero MC_MSMC_ROTOR_D")
    elif card["MC_MSMC_ROTOR_K"] != 0.0 or card["MC_MSMC_ROTOR_D"] != 0.0:
        raise ValueError("rotor speed map set without MC_MSMC_JR")

    strict = strict_terms_enabled(card)

    if strict:
        if not paper_terms_enabled(card):
            raise ValueError("the strict card requires the paper terms to be enabled")

        for name in ("MC_SMC_LPF", "MC_SMC_SLEW"):
            if card[name] != 0.0:
                raise ValueError(f"{name} must be disabled on the strict card")

    for index, (axis, suffix) in enumerate(AXES):
        inertia = card[f"MC_MSMC_J_{suffix}"]
        effectiveness = card[f"MC_MSMC_EFF_{suffix}"]

        if not math.isclose(inertia, IDENTIFIED_INERTIA[index], rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"{axis} inertia must match the identified model")
        if not math.isclose(
            effectiveness, IDENTIFIED_EFFECTIVENESS[index], rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"{axis} effectiveness must match the identified model")

        if strict:
            # The paper has no software torque limit, so the strict card opens
            # TMAX to the parameter maximum and leaves only the allocator's own
            # normalized-torque saturation. Anything lower is a card the paper
            # would not produce; anything else is out of parameter range.
            name = f"MC_MSMC_TMAX_{suffix}"

            if card[name] != STRICT_TORQUE_LIMIT:
                raise ValueError(
                    f"{name} must be {STRICT_TORQUE_LIMIT:g} on the strict card"
                )

        else:
            for unchanged in ("BND", "TMAX"):
                name = f"MC_MSMC_{unchanged}_{suffix}"

                if card[name] != LEGACY_PARAMETERS[name]:
                    raise ValueError(f"{name} must stay at the flight-validated value")

        if strict:
            # Collapsing the boundary layer changes the small-error slope by
            # construction, so the preserved quantity is instead the saturated
            # reaching gain k1 and the linear gain k2 = C + KS, which stay at
            # the identified values and set the authority outside the layer.
            for gain in ("C", "ETA", "KS"):
                name = f"MC_MSMC_{gain}_{suffix}"
                expected = dict(PARAMETERS)[name]

                if card[name] != expected:
                    raise ValueError(
                        f"{name} must stay at the identified value on the strict card"
                    )

        else:
            legacy_slope = local_torque_slope(LEGACY_PARAMETERS, suffix)
            slope = local_torque_slope(card, suffix)

            if abs(slope - legacy_slope) > SLOPE_TOLERANCE * legacy_slope:
                raise ValueError(
                    f"{axis} local torque slope {slope:g} does not preserve the "
                    f"flight-validated {legacy_slope:g}"
                )

        integral_limit = card[f"MC_MSMC_ILIM_{suffix}"]

        if integral_limit > INTEGRAL_LIMIT_MAXIMUM + 1e-9:
            raise ValueError(f"MC_MSMC_ILIM_{suffix} exceeds the parameter maximum")

        legacy_clamp = (
            LEGACY_PARAMETERS[f"MC_MSMC_C_{suffix}"]
            * LEGACY_PARAMETERS[f"MC_MSMC_ILIM_{suffix}"]
        )
        clamp = card[f"MC_MSMC_C_{suffix}"] * integral_limit

        if clamp > legacy_clamp * (1.0 + SLOPE_TOLERANCE):
            raise ValueError(
                f"{axis} integral surface clamp {clamp:g} exceeds the "
                f"flight-validated {legacy_clamp:g}"
            )

    return card


def card_authority(card):
    result = {}

    for axis, suffix in AXES:
        legacy_gain = (
            LEGACY_PARAMETERS[f"MC_MSMC_J_{suffix}"]
            / LEGACY_PARAMETERS[f"MC_MSMC_EFF_{suffix}"]
        )
        model_gain = card[f"MC_MSMC_J_{suffix}"] / card[f"MC_MSMC_EFF_{suffix}"]
        legacy_clamp = (
            LEGACY_PARAMETERS[f"MC_MSMC_C_{suffix}"]
            * LEGACY_PARAMETERS[f"MC_MSMC_ILIM_{suffix}"]
        )
        clamp = card[f"MC_MSMC_C_{suffix}"] * card[f"MC_MSMC_ILIM_{suffix}"]

        result[axis] = {
            # Normalized torque per unit physical torque demand.
            "model_gain": model_gain,
            "model_gain_change": model_gain / legacy_gain,
            "local_torque_slope": local_torque_slope(card, suffix),
            "legacy_local_torque_slope": local_torque_slope(LEGACY_PARAMETERS, suffix),
            # Integral action builds this many times slower than the legacy card.
            "integral_time_constant_change": legacy_gain / model_gain,
            "integral_surface_clamp": clamp,
            "legacy_integral_surface_clamp": legacy_clamp,
            "torque_limit": card[f"MC_MSMC_TMAX_{suffix}"],
        }

    return result


def print_card_authority(card):
    for axis, values in card_authority(card).items():
        print(
            f"target {axis}: J/EFF={values['model_gain']:.6f} "
            f"model_gain_change={values['model_gain_change']:.3f}x "
            f"slope={values['local_torque_slope']:.6f} "
            f"legacy_slope={values['legacy_local_torque_slope']:.6f} "
            f"integral_tau={values['integral_time_constant_change']:.3f}x "
            f"integral_clamp={values['integral_surface_clamp']:.6f} "
            f"legacy_integral_clamp={values['legacy_integral_surface_clamp']:.6f}"
        )


def write_snapshot(path, before, target, verified=None):
    final_target = dict(target)
    final_target["MC_MSMC_CFG"] = 1.0
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--reboot", action="store_true")
    parser.add_argument("--snapshot", type=Path)
    card = parser.add_mutually_exclusive_group()
    card.add_argument(
        "--paper",
        action="store_true",
        help="add the eq. 33 attitude-surface and eq. 6 rotor gyroscopic terms",
    )
    card.add_argument(
        "--paper-strict",
        action="store_true",
        help="also remove the integral, boundary layer, and output filters",
    )
    parser.add_argument(
        "--bench-locked",
        action="store_true",
        help="acknowledge that the airframe is on a rotation-locked test bench",
    )
    args = parser.parse_args()

    if args.reboot and not args.apply:
        parser.error("--reboot requires --apply")

    if args.paper_strict:
        mode = CARD_STRICT
    elif args.paper:
        mode = CARD_PAPER
    else:
        mode = CARD_IDENTIFIED

    if args.bench_locked and mode != CARD_STRICT:
        parser.error("--bench-locked only applies to --paper-strict")

    parameters = card_parameters(mode)

    try:
        target = validate_card(parameters)
    except ValueError as error:
        parser.error(str(error))

    print_card_authority(target)

    if paper_terms_enabled(target):
        print(
            "paper terms enabled: attitude surface "
            f"alpha1=({target['MC_MSMC_A1_R']:g}, {target['MC_MSMC_A1_P']:g}, "
            f"{target['MC_MSMC_A1_Y']:g}) rotor J_R={target['MC_MSMC_JR']:g} "
            f"K={target['MC_MSMC_ROTOR_K']:g} reversed=0b"
            f"{int(target['MC_MSMC_ROTOR_D']):b}"
        )
    else:
        print("paper terms disabled: the installed law matches the flight-validated card")

    if strict_terms_enabled(target):
        print(
            "strict card: surface s = e (ILIM=0), boundary layer "
            f"{STRICT_BOUNDARY_LAYER:g} rad/s, output filters off, "
            f"TMAX={STRICT_TORQUE_LIMIT:g} (allocator saturation only). This is "
            "the chattering law of eq. 32 and is bench-only."
        )

        if args.apply and not args.bench_locked:
            parser.error("--paper-strict --apply requires --bench-locked")
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
    for name, _ in parameters:
        current[name] = read_parameter(master, name)
        print(f"before {name}={current[name][0]:g} type={current[name][1]}")

    if paper_terms_enabled(target):
        # alpha1 is the attitude-loop gain that generated the rate setpoint. If
        # the card and the vehicle disagree, -alpha1^2*z1 is not the derivative
        # of the surface actually being tracked, so refuse rather than guess.
        for (axis, suffix), source in zip(AXES, ATTITUDE_SLOPE_SOURCES):
            installed, _ = read_parameter(master, source)
            slope = target[f"MC_MSMC_A1_{suffix}"]

            if not math.isclose(installed, slope, rel_tol=1e-4, abs_tol=1e-6):
                raise RuntimeError(
                    f"{axis} attitude surface slope {slope:g} does not match "
                    f"{source}={installed:g}"
                )

            print(f"checked {axis} alpha1={slope:g} against {source}")

    if args.snapshot is not None:
        write_snapshot(args.snapshot, current, target)
        print(f"saved pre-change snapshot to {args.snapshot}")

    if not args.apply:
        return

    # Keep Mode 2 inactive and the card unacknowledged until every dependent
    # value has been written. This avoids running a partially updated card.
    write_parameter(master, "MC_RATE_CTRL_T", 0.0, current["MC_RATE_CTRL_T"][1])
    print("wrote MC_RATE_CTRL_T=0")
    write_parameter(master, "MC_MSMC_CFG", 0.0, current["MC_MSMC_CFG"][1])
    print("wrote MC_MSMC_CFG=0")

    for name, value in parameters:
        if name not in ("MC_MSMC_CFG", "MC_RATE_CTRL_T"):
            actual = write_parameter(master, name, value, current[name][1])
            print(f"wrote {name}={actual:g}")

    cfg_type = current["MC_MSMC_CFG"][1]
    actual = write_parameter(master, "MC_MSMC_CFG", 1.0, cfg_type)
    print(f"wrote MC_MSMC_CFG={actual:g} (card acknowledged last)")
    controller_type = current["MC_RATE_CTRL_T"][1]
    actual = write_parameter(master, "MC_RATE_CTRL_T", 2.0, controller_type)
    print(f"wrote MC_RATE_CTRL_T={actual:g} (Mode 2 selected after card acknowledgment)")

    verified = {}
    for name, expected in parameters:
        expected = 1.0 if name == "MC_MSMC_CFG" else expected
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
