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
    # Sized so the law is not clipped before the reference paper's 26 deg release
    # angle. With the k2-scale 2.0 hover card the demand at 26 deg is 0.295, so
    # the old 0.20 limit railed the loop past 17 deg and turned it bang-bang:
    # log 856 sat at 53 deg tilt with the roll demand at -0.517, clipped 100% of
    # the time, limit-cycling at 74 deg/s instead of converging.
    "MC_MSMC_TMAX_R": 0.35,
    "MC_MSMC_TMAX_P": 0.35,
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
    ("MC_MSMC_TMAX_R", 0.35),
    ("MC_MSMC_TMAX_P", 0.35),
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
# the attitude-loop proportional gain that produced the rate setpoint. It is
# therefore READ FROM THE VEHICLE at install time (MC_ROLL_P/MC_PITCH_P/
# MC_YAW_P) and these entries are placeholders only: -alpha1^2*z1 is the
# derivative of the tracked surface only when alpha1 equals the gain that
# actually produced rate_sp, so any hardcoded value is right for one tuning and
# wrong for every other one.
PAPER_ATTITUDE_SLOPE = (0.0, 0.0, 0.0)

# Firmware bound in RateControl::setModelBasedSmcPaperConstants.
ATTITUDE_SLOPE_MAX = 20.0

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
CARD_HOVER = "paper-hover"
CARD_MODES = (CARD_IDENTIFIED, CARD_PAPER, CARD_STRICT, CARD_HOVER)

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

# The paper's law is s2_dot = -k1*sign(s2) - k2*s2. The firmware writes the
# linear part as C*e + KS*s_filtered, and on a card with ILIM=0 the surface is
# s = e, so the C*integral(e) term is identically zero and C stops being an
# integral gain: k2 is exactly C + KS and the split between them is free.
#
# That identity is what the strict card gets wrong. Its k2 = C + KS is inherited
# from a card whose boundary layer was 0.5 rad/s wide, where ETA/BND = 5.04 was
# supplying most of the linear gain as a disguised proportional term. Collapsing
# BND to 0.01 removes that contribution and drops the loop's linear gain by 4x
# (3.1x on yaw) without changing a single gain parameter. The hover card moves
# the same gain back into k2, where the paper puts it.
#
# The increase is loaded onto C rather than KS because C multiplies the raw rate
# error while KS multiplies the low-pass filtered surface: raising C adds gain
# without adding filter phase lag, and KS is capped at 5.0 anyway.
# Measured gyro noise on this airframe is 0.094 rad/s quiet and 0.27 rad/s while
# chattering. A boundary layer below that is not an approximation of sign(s2) at
# all: sat() saturates on noise alone and the term becomes a noise-driven relay.
# Log 850 ran BND=0.02 (1.1 deg/s) against a 5.4 deg/s floor and limit-cycled at
# 5.25 Hz with 31x the 5-20 Hz gyro power of the flight-validated card.
GYRO_NOISE_FLOOR = {"R": 0.094, "P": 0.094, "Y": 0.065}

# Expressed as a fraction of each axis's flight-validated layer so that yaw,
# whose validated layer is 0.2 rather than 0.5, narrows by the same proportion
# instead of being handed a roll-sized number.
HOVER_LAYER_FRACTION = 0.3

# Deviations from the paper that the hover card keeps, and why. The strict card
# exists to test without them; this card is the one that is meant to fly.
#
#   TMAX      stays at the flight-validated limit instead of the paper's absent
#             software limit, because a bench mount is not a free-flying vehicle.
#   LPF/SLEW  stay enabled. The paper has no actuator model at all; a relay-like
#             switching term on real ESCs needs the chattering mitigation.
HOVER_KEEPS_FLIGHT_TORQUE_LIMIT = True


def in_layer_linear_gain(card, suffix):
    """Linear gain seen inside the boundary layer: C + KS + ETA/BND."""
    return (
        card[f"MC_MSMC_C_{suffix}"]
        + card[f"MC_MSMC_KS_{suffix}"]
        + card[f"MC_MSMC_ETA_{suffix}"] / card[f"MC_MSMC_BND_{suffix}"]
    )


def reaching_linear_gain(card, suffix):
    """k2 of eq. 32: the linear gain outside a collapsed boundary layer."""
    return card[f"MC_MSMC_C_{suffix}"] + card[f"MC_MSMC_KS_{suffix}"]


def switching_authority(card, suffix):
    """Normalized torque the saturated k1*sign(s) term can command on its own."""
    return (
        card[f"MC_MSMC_J_{suffix}"]
        / card[f"MC_MSMC_EFF_{suffix}"]
        * card[f"MC_MSMC_ETA_{suffix}"]
    )


# Firmware parameter range of MC_MSMC_C_*.
SURFACE_GAIN_MAXIMUM = 10.0
SURFACE_GAIN_MINIMUM = 0.1

# Destabilizing torque per radian contributed by the test bench itself, measured
# from log 849: pitch settled at -29 deg with 0.136 of commanded torque and no
# residual rate, against ~0.002 at level in log 843, so the load grows with
# angle like a CG offset above the pivot. Level is only an equilibrium while the
# controller's attitude stiffness (J/EFF)*alpha2*MC_*_P exceeds it.
BENCH_PENDULUM_STIFFNESS = 0.28


def switching_slope(card, suffix):
    """ETA/BND: the linear gain the switching term contributes inside the layer."""
    return card[f"MC_MSMC_ETA_{suffix}"] / card[f"MC_MSMC_BND_{suffix}"]


def hover_layer(suffix, fraction):
    """Boundary layer for one axis, as a fraction of its validated width."""
    return fraction * LEGACY_PARAMETERS[f"MC_MSMC_BND_{suffix}"]


def hover_reaching_gain(suffix, boundary_layer):
    """ETA scaled with the boundary layer so ETA/BND stays flight-validated.

    Narrowing the layer without touching ETA multiplies ETA/BND, which is a
    linear gain, not a switching amplitude: going from BND=0.5 to BND=0.02 at
    fixed ETA raises it from 5.04 to 126. Tying ETA to BND makes the layer width
    a pure fidelity knob -- it sets the error at which the term saturates and
    becomes a true relay -- without changing the small-signal gain at all.
    """
    identified = dict(PARAMETERS)
    return boundary_layer * switching_slope(identified, suffix)


def hover_surface_gain(suffix, boundary_layer, scale=1.0):
    """C that makes the total in-layer gain match the flight-validated one.

    The gain the loop actually sees is C + KS + ETA/BND, so C has to absorb
    whatever the switching term contributes at this boundary layer. `scale`
    multiplies the target, which is needed when the eq. 33 attitude-surface term
    is enabled: it subtracts alpha1, and on this bench the remainder was below
    the pendulum stiffness and level was not an equilibrium.

    C is the least desirable place to carry gain -- it multiplies the raw rate
    error while KS multiplies the low-pass filtered surface -- so this keeps it
    as small as the target allows rather than loading it deliberately.
    """
    identified = dict(PARAMETERS)
    target = scale * in_layer_linear_gain(identified, suffix)
    return (
        target
        - identified[f"MC_MSMC_KS_{suffix}"]
        - hover_reaching_gain(suffix, boundary_layer) / boundary_layer
    )


def hover_overrides(fraction, k2_scale=1.0):
    overrides = [
        ("MC_MSMC_ILIM_R", 0.0),
        ("MC_MSMC_ILIM_P", 0.0),
        ("MC_MSMC_ILIM_Y", 0.0),
    ]

    for _, suffix in AXES:
        boundary_layer = hover_layer(suffix, fraction)
        surface_gain = hover_surface_gain(suffix, boundary_layer, k2_scale)

        if surface_gain > SURFACE_GAIN_MAXIMUM:
            raise ValueError(
                f"MC_MSMC_C_{suffix}={surface_gain:g} exceeds the parameter maximum "
                f"{SURFACE_GAIN_MAXIMUM:g}; lower --k2-scale"
            )

        if surface_gain < SURFACE_GAIN_MINIMUM:
            raise ValueError(
                f"MC_MSMC_C_{suffix}={surface_gain:g} is below the parameter minimum "
                f"{SURFACE_GAIN_MINIMUM:g}; raise --k2-scale"
            )

        overrides.append((f"MC_MSMC_BND_{suffix}", boundary_layer))
        overrides.append((f"MC_MSMC_ETA_{suffix}", hover_reaching_gain(suffix, boundary_layer)))
        overrides.append((f"MC_MSMC_C_{suffix}", surface_gain))

    return tuple(overrides)


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


def card_parameters(mode, fraction=HOVER_LAYER_FRACTION, k2_scale=1.0):
    """Full card for one of CARD_MODES."""
    if mode not in CARD_MODES:
        raise ValueError(f"unknown card mode {mode}")

    parameters = PARAMETERS + (PAPER_DISABLED if mode == CARD_IDENTIFIED else PAPER_ENABLED)

    if mode == CARD_STRICT:
        overrides = dict(STRICT_OVERRIDES)
    elif mode == CARD_HOVER:
        overrides = dict(hover_overrides(fraction, k2_scale))
    else:
        return parameters

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


def hover_terms_enabled(card):
    """True for the paper surface and reaching law with the flight safety net.

    Distinguished from the strict card by a boundary layer that is narrow enough
    to saturate at hover-scale rate errors but is deliberately not the parameter
    minimum, so the two cards can never be confused for one another.
    """
    return all(
        card[f"MC_MSMC_ILIM_{suffix}"] == 0.0
        and STRICT_BOUNDARY_LAYER < card[f"MC_MSMC_BND_{suffix}"]
        < LEGACY_PARAMETERS[f"MC_MSMC_BND_{suffix}"]
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
    hover = not strict and hover_terms_enabled(card)

    if strict:
        if not paper_terms_enabled(card):
            raise ValueError("the strict card requires the paper terms to be enabled")

        for name in ("MC_SMC_LPF", "MC_SMC_SLEW"):
            if card[name] != 0.0:
                raise ValueError(f"{name} must be disabled on the strict card")

    if hover:
        # Unlike the strict card the paper terms are optional here, because the
        # hover card carries two independent changes: the restored k2, which is
        # derived from flight data, and the eq. 33 attitude-surface term, which
        # has never flown.
        #
        # The attitude-surface term is not a loss of authority even though it
        # opposes the rate setpoint. The paper's U2 carries both -alpha1^2*z1
        # and -alpha1*z2; with z1 = rate_sp/alpha1 and z2 = -e those are
        # -alpha1*rate_sp and +alpha1*e, which cancel at the moment of release
        # where e = rate_sp. What survives is alpha2 = C + KS - alpha1, so C has
        # to be at least alpha1 for the card to be self-consistent. That is why
        # the term is only safe to enable together with the restored k2: on the
        # old C = 1.12 it would have left alpha2 negative.

        # These are the deviations the hover card exists to keep. Losing them
        # silently would turn it into a strict card with a wider layer.
        for name in ("MC_SMC_LPF", "MC_SMC_SLEW"):
            if card[name] != dict(PARAMETERS)[name]:
                raise ValueError(
                    f"{name} must stay at the flight-validated value on the hover card"
                )

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

        elif hover:
            # The paper has no software torque limit, but a bench mount is not a
            # free-flying vehicle, so this card keeps the flight-validated one.
            name = f"MC_MSMC_TMAX_{suffix}"

            if card[name] != LEGACY_PARAMETERS[name]:
                raise ValueError(
                    f"{name} must stay at the flight-validated value on the hover card"
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

        elif hover:
            # k1 is the disturbance-rejection gain and stays at the identified
            # value; k2 = C + KS must equal the linear gain the flight-validated
            # card actually ran with, which is its in-layer gain including the
            # ETA/BND contribution that the narrow layer removes.
            ratio = switching_slope(card, suffix)
            expected_ratio = switching_slope(dict(PARAMETERS), suffix)

            if abs(ratio - expected_ratio) > SLOPE_TOLERANCE * expected_ratio:
                raise ValueError(
                    f"{axis} ETA/BND={ratio:g} does not preserve the "
                    f"flight-validated switching slope {expected_ratio:g}"
                )

            floor = GYRO_NOISE_FLOOR[suffix]

            if card[f"MC_MSMC_BND_{suffix}"] < floor:
                raise ValueError(
                    f"MC_MSMC_BND_{suffix}={card[f'MC_MSMC_BND_{suffix}']:g} is below "
                    f"the {floor:g} rad/s gyro noise floor; sign(s2) would "
                    f"switch on noise instead of on error"
                )

            if card[f"MC_MSMC_KS_{suffix}"] != dict(PARAMETERS)[f"MC_MSMC_KS_{suffix}"]:
                raise ValueError(
                    f"MC_MSMC_KS_{suffix} must stay at the identified value on the "
                    "hover card; the recovered gain is carried by C"
                )

            target_gain = in_layer_linear_gain(dict(PARAMETERS), suffix)
            gain = in_layer_linear_gain(card, suffix)

            # The gain that matters is the total the loop sees, C + KS + ETA/BND.
            # It may be raised above the flight-validated value to clear the
            # bench pendulum, but never lowered below it: that is the defect
            # this card exists to fix.
            if gain < target_gain * (1.0 - SLOPE_TOLERANCE):
                raise ValueError(
                    f"{axis} in-layer gain {gain:g} is below the "
                    f"flight-validated {target_gain:g}"
                )

            if card[f"MC_MSMC_C_{suffix}"] > SURFACE_GAIN_MAXIMUM:
                raise ValueError(
                    f"MC_MSMC_C_{suffix} exceeds the parameter maximum "
                    f"{SURFACE_GAIN_MAXIMUM:g}"
                )

            # A boundary layer only approximates sign(s) if it saturates well
            # inside the rate noise the loop already lives with.
            if card[f"MC_MSMC_BND_{suffix}"] >= LEGACY_PARAMETERS[f"MC_MSMC_BND_{suffix}"]:
                raise ValueError(
                    f"MC_MSMC_BND_{suffix} is no narrower than the flight-validated "
                    "layer, so the card is not approaching sign(s2) at all"
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
    card.add_argument(
        "--paper-hover",
        action="store_true",
        help="paper surface (ILIM=0) and a sign-like boundary layer, with the "
             "linear gain k2=C+KS restored to the value the flight-validated "
             "card actually ran with, and the torque limit and output filters "
             "kept; this is the paper card that is meant to hover",
    )
    parser.add_argument(
        "--layer-fraction",
        type=float,
        default=HOVER_LAYER_FRACTION,
        help=f"hover-card boundary layer as a fraction of each axis's validated "
             f"width (default {HOVER_LAYER_FRACTION:g}); lower it for a more "
             "sign-like switching term, raise it if the loop buzzes",
    )
    parser.add_argument(
        "--k2-scale",
        type=float,
        default=1.0,
        help="multiply the restored k2 on the hover card (default 1.0). The eq. 33 "
             "attitude-surface term subtracts alpha1 from the effective gain, so a "
             "scale above 1 is needed when alpha2 would otherwise fall below the "
             "bench pendulum stiffness",
    )
    parser.add_argument(
        "--no-paper-terms",
        action="store_true",
        help="on the hover card, leave the eq. 33 attitude-surface and eq. 6 rotor "
             "terms disabled so the restored k2 is the only change under test",
    )
    parser.add_argument(
        "--bench-locked",
        action="store_true",
        help="acknowledge that the airframe is on a rotation-locked test bench",
    )
    parser.add_argument(
        "--no-integral",
        action="store_true",
        help="zero MC_MSMC_ILIM_* so the surface is the s2 of eq. 30, while keeping "
             "the boundary layer, output filters and torque limits of the flight "
             "card; isolates the surface change from the chattering law of eq. 32",
    )
    args = parser.parse_args()

    if args.reboot and not args.apply:
        parser.error("--reboot requires --apply")

    if args.paper_strict:
        mode = CARD_STRICT
    elif args.paper_hover:
        mode = CARD_HOVER
    elif args.paper:
        mode = CARD_PAPER
    else:
        mode = CARD_IDENTIFIED

    if args.bench_locked and mode not in (CARD_STRICT, CARD_HOVER):
        parser.error("--bench-locked only applies to --paper-strict or --paper-hover")

    if mode != CARD_HOVER and args.layer_fraction != HOVER_LAYER_FRACTION:
        parser.error("--layer-fraction only applies to --paper-hover")

    if mode == CARD_HOVER and not 0.0 < args.layer_fraction < 1.0:
        parser.error("--layer-fraction must be between 0 and 1")

    if args.k2_scale != 1.0 and mode != CARD_HOVER:
        parser.error("--k2-scale only applies to --paper-hover")

    if not 1.0 <= args.k2_scale <= 3.0:
        parser.error("--k2-scale must be between 1.0 and 3.0")

    if args.no_paper_terms and mode != CARD_HOVER:
        parser.error("--no-paper-terms only applies to --paper-hover")

    parameters = card_parameters(mode, args.layer_fraction, args.k2_scale)

    if args.no_paper_terms:
        disabled = dict(PAPER_DISABLED)
        parameters = tuple(
            (name, disabled.get(name, value)) for name, value in parameters
        )

    if args.no_integral:
        if mode in (CARD_STRICT, CARD_HOVER):
            parser.error(f"--{mode} already zeroes the integral")

        # Only the integral clamp moves. The boundary layer stays at the flight
        # value, so the small-error slope C + KS + ETA/BND is unchanged and the
        # loop keeps the gain it was validated with, unlike the strict card
        # where collapsing BND multiplies that slope by ~38x.
        zeroed = {f"MC_MSMC_ILIM_{suffix}": 0.0 for _, suffix in AXES}
        parameters = tuple((name, zeroed.get(name, value)) for name, value in parameters)

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

    if hover_terms_enabled(target):
        print(
            "hover card: surface s = e (ILIM=0), boundary layer at "
            f"{args.layer_fraction:g} of the flight-validated width, torque limit "
            "and output filters kept."
        )

        for axis, suffix in AXES:
            identified = dict(PARAMETERS)
            print(
                f"  {axis}: k1={target[f'MC_MSMC_ETA_{suffix}']:.4f} "
                f"(saturated authority {switching_authority(target, suffix):.4f} "
                f"normalized torque) k2={reaching_linear_gain(target, suffix):.4f} "
                f"restored from {reaching_linear_gain(identified, suffix):.4f} "
                f"({reaching_linear_gain(target, suffix) / reaching_linear_gain(identified, suffix):.1f}x), "
                f"C {identified[f'MC_MSMC_C_{suffix}']:.4f}"
                f"->{target[f'MC_MSMC_C_{suffix}']:.4f}, "
                f"sign(s2) saturates above "
                f"{math.degrees(target[f'MC_MSMC_BND_{suffix}']):.1f} deg/s of rate error"
            )

        if args.apply and not args.bench_locked:
            parser.error("--paper-hover --apply requires --bench-locked")
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
        # alpha1 is the attitude-loop gain that generated the rate setpoint, so
        # derive it from the vehicle instead of asserting a constant: that way
        # -alpha1^2*z1 is the derivative of the surface actually being tracked
        # on THIS airframe, by construction rather than by coincidence.
        for (axis, suffix), source in zip(AXES, ATTITUDE_SLOPE_SOURCES):
            installed, _ = read_parameter(master, source)

            if not 0.0 < installed <= ATTITUDE_SLOPE_MAX:
                raise RuntimeError(
                    f"{source}={installed:g} is outside (0, {ATTITUDE_SLOPE_MAX:g}]; "
                    f"the firmware would reject alpha1 and silently drop the "
                    f"{axis} attitude-surface term"
                )

            # The attitude-surface term contributes -alpha1*rate_sp while the
            # linear reaching term contributes +(C + KS)*e, and at release
            # e = rate_sp, so the convergence gain that actually survives is
            # alpha2 = C + KS - alpha1. alpha1 can only be known here, after it
            # is read from the vehicle, which is why this is not in
            # validate_card. A non-positive alpha2 is a divergent card: the
            # paper-strict card reaches C + KS = 1.68 against alpha1 = 4.
            reaching = reaching_linear_gain(target, suffix)
            alpha2 = reaching - installed

            if alpha2 <= 0.0:
                raise RuntimeError(
                    f"{axis} alpha2 = C + KS - alpha1 = {reaching:g} - {installed:g} "
                    f"= {alpha2:g} is not positive: with the eq. 33 attitude-surface "
                    f"term enabled this card opposes its own correction and diverges. "
                    f"Raise MC_MSMC_C_{suffix} above {installed:g} or disable the "
                    f"paper terms."
                )

            # Level is only an equilibrium while the controller's attitude
            # stiffness beats the bench's own destabilizing load. This is what
            # log 849 failed: alpha2 = 2.36 gave 0.178 against a measured 0.28,
            # and pitch ran away to -29 deg while roll, whose offset is smaller,
            # held 1.1 deg rms on the same card.
            stiffness = (
                target[f"MC_MSMC_J_{suffix}"]
                / target[f"MC_MSMC_EFF_{suffix}"]
                * alpha2
                * installed
            )
            # Yaw is exempt: rotating about the vertical axis does not raise or
            # lower the centre of mass, so there is no gravity pendulum to beat.
            pendulum = 0.0 if suffix == "Y" else BENCH_PENDULUM_STIFFNESS
            margin = stiffness / pendulum if pendulum else float("inf")

            target[f"MC_MSMC_A1_{suffix}"] = installed
            print(
                f"derived {axis} alpha1={installed:g} from {source}; "
                f"alpha2 = C + KS - alpha1 = {alpha2:.4f}; attitude stiffness "
                f"{stiffness:.3f}"
                + (
                    " (no gravity pendulum on this axis)"
                    if not pendulum
                    else f" vs bench {pendulum:g} ({margin:.2f}x)"
                )
            )

            if margin < 1.0:
                print(
                    f"  WARNING: {axis} stiffness is below the measured bench load; "
                    f"level is not an equilibrium and this axis will drift away. "
                    f"Raise --k2-scale to at least "
                    f"{math.ceil(100 / margin) / 100:.2f}."
                )

        # The write and verify loops iterate `parameters`, not `target`, so the
        # derived slopes have to be folded back in or they are silently dropped
        # and verification passes against the placeholder.
        parameters = tuple((name, target[name]) for name, _ in parameters)

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
