#!/usr/bin/env python3
"""Validate the UAV985 Gazebo assembly and rate-controller model contract."""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASE_MODEL = ROOT / "Tools/simulation/gz/models/uav985_base/model.sdf"
MOTOR_MODEL = ROOT / "Tools/simulation/gz/models/uav985/model.sdf"
FLOW_MODEL = ROOT / "Tools/simulation/gz/models/uav985_flow/model.sdf"
AIRFRAME = ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_uav985"
TARGET_MASS = 0.985
TARGET_INERTIA = (0.0135, 0.0118, 0.0170)


def numeric_text(parent, path):
    return float(parent.find(path).text)


def pose(link):
    element = link.find("pose")
    values = [0.0] * 6 if element is None else [float(value) for value in element.text.split()]
    return values[:3]


def assembled_model():
    root = ET.parse(BASE_MODEL).getroot()
    links = []

    for link in root.findall("./model/link"):
        inertial = link.find("inertial")

        if inertial is None:
            continue

        links.append(
            {
                "name": link.attrib["name"],
                "mass": numeric_text(inertial, "mass"),
                "position": pose(link),
                "inertia": tuple(
                    numeric_text(inertial, f"inertia/{axis}") for axis in ("ixx", "iyy", "izz")
                ),
            }
        )

    total_mass = sum(link["mass"] for link in links)
    center_of_mass = [
        sum(link["mass"] * link["position"][axis] for link in links) / total_mass
        for axis in range(3)
    ]
    total_inertia = [0.0, 0.0, 0.0]

    for link in links:
        dx, dy, dz = [link["position"][axis] - center_of_mass[axis] for axis in range(3)]
        offsets = (dy * dy + dz * dz, dx * dx + dz * dz, dx * dx + dy * dy)

        for axis in range(3):
            total_inertia[axis] += link["inertia"][axis] + link["mass"] * offsets[axis]

    return total_mass, center_of_mass, total_inertia


def motor_parameters():
    root = ET.parse(MOTOR_MODEL).getroot()
    plugins = root.findall("./model/plugin")
    motor_plugins = [plugin for plugin in plugins if plugin.find("motorConstant") is not None]

    if len(motor_plugins) != 4:
        raise RuntimeError(f"expected four motor plugins, found {len(motor_plugins)}")

    names = ("motorConstant", "momentConstant", "timeConstantUp", "timeConstantDown", "maxRotVelocity")
    values = {name: [numeric_text(plugin, name) for plugin in motor_plugins] for name in names}

    for name, entries in values.items():
        if max(entries) - min(entries) > 1e-12:
            raise RuntimeError(f"motor plugins disagree on {name}: {entries}")

    return {name: entries[0] for name, entries in values.items()}


def flow_accessory_mass():
    # ElementTree requires namespace declarations for prefixed attributes, while
    # sdformat accepts the Gazebo custom sensor attribute without one.
    xml = FLOW_MODEL.read_text().replace("gz:type=", "gz_type=")
    root = ET.fromstring(xml)
    masses = []

    for link in root.findall("./model/link"):
        inertial = link.find("inertial")

        if inertial is not None:
            masses.append(numeric_text(inertial, "mass"))

    return sum(masses)


def airframe_parameters():
    pattern = re.compile(r"^param set-default ([A-Z0-9_]+)\s+([-+0-9.eE]+)\s*$")
    parameters = {}

    for line in AIRFRAME.read_text().splitlines():
        match = pattern.match(line)

        if match:
            parameters[match.group(1)] = float(match.group(2))

    return parameters


def assert_close(label, actual, expected, relative_tolerance=1e-4, absolute_tolerance=1e-8):
    if not math.isclose(actual, expected, rel_tol=relative_tolerance, abs_tol=absolute_tolerance):
        raise RuntimeError(f"{label}: actual={actual:.10g} expected={expected:.10g}")


def torque_effectiveness(command, speed_min, speed_max, arm_component, motors):
    rotor_speed = speed_min + command * (speed_max - speed_min)
    thrust_slope = 2.0 * motors["motorConstant"] * rotor_speed * (speed_max - speed_min)
    return (
        4.0 * arm_component * thrust_slope / math.sqrt(2.0),
        4.0 * arm_component * thrust_slope / math.sqrt(2.0),
        4.0 * motors["momentConstant"] * thrust_slope,
    )


def main():
    mass, center_of_mass, inertia = assembled_model()
    motors = motor_parameters()
    accessory_mass = flow_accessory_mass()
    parameters = airframe_parameters()

    assert_close("assembled mass", mass, TARGET_MASS)
    assert_close(
        "flow assembled mass",
        mass + accessory_mass,
        TARGET_MASS,
        relative_tolerance=1e-5,
        absolute_tolerance=5e-6,
    )

    if accessory_mass > 1e-5:
        raise RuntimeError(f"flow sensor links double-count CAD mass: {accessory_mass:.9f} kg")

    for axis, actual, expected in zip("xyz", inertia, TARGET_INERTIA):
        assert_close(f"assembled J{axis}{axis}", actual, expected)

    arm_component = parameters["CA_ROTOR0_PX"]
    speed_min = parameters["SIM_GZ_EC_MIN1"]
    speed_max = parameters["SIM_GZ_EC_MAX1"]
    hover_speed = math.sqrt(TARGET_MASS * 9.80665 / (4.0 * motors["motorConstant"]))
    hover_command = (hover_speed - speed_min) / (speed_max - speed_min)
    effectiveness = torque_effectiveness(hover_command, speed_min, speed_max, arm_component, motors)

    if parameters.get("MC_MSMC_CFG") != 1.0:
        raise RuntimeError("UAV985 SMC physical model card is not acknowledged")

    assert_close("MIS_TKO_ALT_MAX", parameters["MIS_TKO_ALT_MAX"], 1.0)

    for prefix in ("MC_MPC_J", "MC_MSMC_J"):
        for suffix, expected in zip("RPY", TARGET_INERTIA):
            assert_close(f"{prefix}_{suffix}", parameters[f"{prefix}_{suffix}"], expected)

    for prefix in ("MC_MPC_EFF", "MC_MSMC_EFF"):
        for suffix, expected in zip("RPY", effectiveness):
            assert_close(f"{prefix}_{suffix}", parameters[f"{prefix}_{suffix}"], expected, relative_tolerance=0.02)

    lag = 0.5 * (motors["timeConstantUp"] + motors["timeConstantDown"])
    assert_close("MC_MPC_TAU", parameters["MC_MPC_TAU"], lag, relative_tolerance=0.35)

    print(f"mass={mass:.6f}kg cg=({center_of_mass[0]:.7f},{center_of_mass[1]:.7f},{center_of_mass[2]:.7f})m")
    print(f"flow_accessory_dynamic_mass={accessory_mass:.9f}kg (configured all-up mass assumption)")
    print(f"inertia=({inertia[0]:.8f},{inertia[1]:.8f},{inertia[2]:.8f})kg*m^2")
    print(f"hover_speed={hover_speed:.3f}rad/s hover_command={hover_command:.4f}")
    print(
        "hover-local-effectiveness="
        f"({effectiveness[0]:.4f},{effectiveness[1]:.4f},{effectiveness[2]:.4f})Nm/unit"
    )

    for command in (0.30, 0.50):
        local_effectiveness = torque_effectiveness(command, speed_min, speed_max, arm_component, motors)
        ratio = local_effectiveness[0] / effectiveness[0]
        print(
            f"effectiveness@collective={command:.2f}: "
            f"({local_effectiveness[0]:.4f},{local_effectiveness[1]:.4f},"
            f"{local_effectiveness[2]:.4f})Nm/unit ({ratio:.3f}x hover)"
        )

    print("contract scope: local hover model; identify the real vehicle in the intended thrust window")
    print("UAV985 model contract: PASS")


if __name__ == "__main__":
    main()
