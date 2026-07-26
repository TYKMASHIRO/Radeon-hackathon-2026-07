"""Build and validate the passive Phase 4 cockpit mechanisms.

Author: OpenAI Codex
Date: 2026-07-16
Purpose: Generate seat, stick, throttle, coupled rudder, brakes, and test rig.
Why: Mechanisms must be independently loadable and physically testable before
the collision-aware seated reachability search fixes their final layout.
Inputs: YAML layout, mechanism, contact, and simulation parameters.
Outputs: MJCF component files, a combined mechanism scene, validation metrics.
Exceptions: Raises source, compile, numerical, or behavior validation errors.
Coordinate Frame: X aircraft forward, Y aircraft left, Z aircraft up.
Units: SI and radians.
Safety Notes: No actuators are present; all return/hold behavior is passive.
Source: Project engineering parameters, explicitly marked as pre-calibration.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


def _values(values: list[float] | tuple[float, ...]) -> str:
    return " ".join(str(value) for value in values)


def _new_model(name: str, simulation: dict[str, Any], contact: dict[str, Any]) -> ET.Element:
    root = ET.Element("mujoco", {"model": name})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    ET.SubElement(
        root,
        "option",
        {
            "timestep": str(simulation["timestep_s"]),
            "gravity": _values(simulation["gravity_m_s2"]),
        },
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", {"friction": _values(contact["default_friction"])})
    return root


def _add_environment(root: ET.Element, layout: dict[str, Any]) -> ET.Element:
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"name": "cockpit_key", "pos": "-0.5 -0.5 2.0"})
    platform = layout["platform"]
    ET.SubElement(
        world,
        "geom",
        {
            "name": "cockpit_platform",
            "type": "box",
            "pos": _values(platform["center_m"]),
            "size": _values(platform["half_size_m"]),
            "rgba": "0.22 0.25 0.28 1",
        },
    )
    camera = layout["cameras"]["cockpit_overview"]
    ET.SubElement(
        world,
        "camera",
        {
            "name": "cockpit_overview",
            "pos": _values(camera["position_m"]),
            "xyaxes": _values(camera["xyaxes"]),
            "fovy": str(camera["fovy_deg"]),
        },
    )
    return world


def _seat_body(layout: dict[str, Any], contact: dict[str, Any]) -> ET.Element:
    seat = layout["seat"]
    body = ET.Element("body", {"name": "adjustable_seat", "pos": _values(seat["reference_m"])})
    ET.SubElement(
        body,
        "geom",
        {
            "name": "seat_cushion",
            "type": "box",
            "size": _values(seat["cushion_half_size_m"]),
            "friction": _values(contact["seat_friction"]),
            "rgba": "0.18 0.24 0.32 1",
        },
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": "seat_back",
            "type": "box",
            "pos": _values(seat["back_center_offset_m"]),
            "size": _values(seat["back_half_size_m"]),
            "friction": _values(contact["seat_friction"]),
            "rgba": "0.18 0.24 0.32 1",
        },
    )
    support = seat["pelvis_support_half_size_m"]
    cushion_y = seat["cushion_half_size_m"][1]
    for side, sign in (("left", 1.0), ("right", -1.0)):
        ET.SubElement(
            body,
            "geom",
            {
                "name": f"{side}_pelvis_support",
                "type": "box",
                "pos": _values((0.0, sign * (cushion_y - support[1]), support[2])),
                "size": _values(support),
                "friction": _values(contact["seat_friction"]),
                "rgba": "0.20 0.27 0.35 1",
            },
        )
    return body


def _stick_body(layout: dict[str, Any], mechanics: dict[str, Any], contact: dict[str, Any]) -> tuple[ET.Element, ET.Element]:
    config = mechanics["stick"]
    body = ET.Element("body", {"name": "control_stick_base", "pos": _values(layout["controls"]["stick_base_m"])})
    ET.SubElement(body, "geom", {"name": "stick_base_geom", "type": "cylinder", "size": "0.06 0.035", "contype": "0", "conaffinity": "0", "rgba": "0.25 0.28 0.30 1"})
    roll = ET.SubElement(body, "body", {"name": "stick_roll_frame"})
    ET.SubElement(
        roll,
        "joint",
        {
            "name": "stick_roll_joint",
            "type": "hinge",
            "axis": "1 0 0",
            "range": _values(config["roll_range_rad"]),
            "stiffness": str(config["stiffness_nm_rad"]),
            "damping": str(config["damping_nms_rad"]),
            "frictionloss": str(config["frictionloss_nm"]),
        },
    )
    ET.SubElement(roll, "geom", {"name": "stick_roll_hub", "type": "sphere", "size": "0.025", "mass": "0.05", "contype": "0", "conaffinity": "0", "rgba": "0.3 0.32 0.34 1"})
    pitch = ET.SubElement(roll, "body", {"name": "stick_pitch_frame"})
    ET.SubElement(
        pitch,
        "joint",
        {
            "name": "stick_pitch_joint",
            "type": "hinge",
            "axis": "0 1 0",
            "range": _values(config["pitch_range_rad"]),
            "stiffness": str(config["stiffness_nm_rad"]),
            "damping": str(config["damping_nms_rad"]),
            "frictionloss": str(config["frictionloss_nm"]),
        },
    )
    ET.SubElement(
        pitch,
        "geom",
        {
            "name": "stick_handle",
            "type": "capsule",
            "fromto": f"0 0 0 0 0 {config['handle_length_m']}",
            "size": "0.025",
            "mass": str(config["moving_mass_kg"] - 0.05),
            "friction": _values(contact["control_surface_friction"]),
            "rgba": "0.08 0.09 0.10 1",
        },
    )
    ET.SubElement(pitch, "site", {"name": "stick_force_site", "pos": f"0 0 {config['handle_length_m']}", "size": "0.01"})
    sensor = ET.Element("sensor")
    ET.SubElement(sensor, "jointpos", {"name": "stick_roll_position", "joint": "stick_roll_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "stick_roll_velocity", "joint": "stick_roll_joint"})
    ET.SubElement(sensor, "jointpos", {"name": "stick_pitch_position", "joint": "stick_pitch_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "stick_pitch_velocity", "joint": "stick_pitch_joint"})
    ET.SubElement(sensor, "force", {"name": "stick_force", "site": "stick_force_site"})
    ET.SubElement(sensor, "torque", {"name": "stick_torque", "site": "stick_force_site"})
    return body, sensor


def _throttle_body(layout: dict[str, Any], mechanics: dict[str, Any], contact: dict[str, Any]) -> tuple[ET.Element, ET.Element]:
    config = mechanics["throttle"]
    body = ET.Element("body", {"name": "throttle_base", "pos": _values(layout["controls"]["throttle_base_m"])})
    ET.SubElement(body, "geom", {"name": "throttle_rail", "type": "box", "pos": f"{config['travel_m'] / 2} 0 0", "size": f"{config['travel_m'] / 2 + 0.025} 0.045 0.025", "rgba": "0.25 0.28 0.30 1"})
    slider = ET.SubElement(body, "body", {"name": "throttle_handle_body"})
    ET.SubElement(
        slider,
        "joint",
        {
            "name": "throttle_joint",
            "type": "slide",
            "axis": "1 0 0",
            "range": f"0 {config['travel_m']}",
            "damping": str(config["damping_ns_m"]),
            "frictionloss": str(config["frictionloss_n"]),
        },
    )
    ET.SubElement(slider, "geom", {"name": "throttle_handle", "type": "box", "pos": "0 0 0.07", "size": "0.035 0.045 0.07", "mass": str(config["moving_mass_kg"]), "friction": _values(contact["control_surface_friction"]), "rgba": "0.12 0.13 0.14 1"})
    ET.SubElement(slider, "site", {"name": "throttle_force_site", "pos": "0 0 0.14", "size": "0.01"})
    sensor = ET.Element("sensor")
    ET.SubElement(sensor, "jointpos", {"name": "throttle_position", "joint": "throttle_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "throttle_velocity", "joint": "throttle_joint"})
    ET.SubElement(sensor, "force", {"name": "throttle_force", "site": "throttle_force_site"})
    return body, sensor


def _pedal_system(layout: dict[str, Any], mechanics: dict[str, Any], contact: dict[str, Any]) -> tuple[list[ET.Element], ET.Element, ET.Element]:
    controls = layout["controls"]
    rudder = mechanics["rudder"]
    brakes = mechanics["brakes"]
    bodies: list[ET.Element] = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        position = (controls["pedal_center_x_m"], sign * controls["pedal_lateral_offset_m"], controls["pedal_height_m"])
        pedal = ET.Element("body", {"name": f"{side}_rudder_pedal", "pos": _values(position)})
        ET.SubElement(
            pedal,
            "joint",
            {
                "name": f"{side}_rudder_joint",
                "type": "slide",
                "axis": "1 0 0",
                "range": f"{-rudder['pedal_travel_m']} {rudder['pedal_travel_m']}",
                "stiffness": str(rudder["stiffness_n_m"]),
                "damping": str(rudder["damping_ns_m"]),
                "frictionloss": str(rudder["frictionloss_n"]),
            },
        )
        ET.SubElement(pedal, "geom", {"name": f"{side}_rudder_plate", "type": "box", "size": "0.025 0.075 0.09", "mass": str(rudder["pedal_mass_kg_each"]), "friction": _values(contact["control_surface_friction"]), "rgba": "0.22 0.24 0.25 1"})
        brake = ET.SubElement(pedal, "body", {"name": f"{side}_brake_plate", "pos": "0 0 0.07"})
        ET.SubElement(
            brake,
            "joint",
            {
                "name": f"{side}_brake_joint",
                "type": "hinge",
                "axis": f"0 {sign} 0",
                "range": _values(brakes["range_rad"]),
                "stiffness": str(brakes["stiffness_nm_rad"]),
                "damping": str(brakes["damping_nms_rad"]),
                "frictionloss": str(brakes["frictionloss_nm"]),
            },
        )
        ET.SubElement(brake, "geom", {"name": f"{side}_brake_surface", "type": "box", "pos": "0.03 0 0.04", "size": "0.045 0.07 0.015", "mass": str(brakes["plate_mass_kg_each"]), "friction": _values(contact["control_surface_friction"]), "rgba": "0.35 0.12 0.10 1"})
        heel = ET.Element("body", {"name": f"{side}_heel_support", "pos": _values((controls["pedal_center_x_m"] - 0.13, sign * controls["pedal_lateral_offset_m"], controls["heel_support_height_m"]))})
        ET.SubElement(heel, "geom", {"name": f"{side}_heel_support_geom", "type": "box", "size": "0.08 0.08 0.02", "friction": _values(contact["control_surface_friction"]), "rgba": "0.20 0.22 0.24 1"})
        bodies.extend((pedal, heel))
    equality = ET.Element("equality")
    ET.SubElement(equality, "joint", {"name": "rudder_opposed_coupling", "joint1": "right_rudder_joint", "joint2": "left_rudder_joint", "polycoef": "0 -1 0 0 0"})
    sensor = ET.Element("sensor")
    for side in ("left", "right"):
        ET.SubElement(sensor, "jointpos", {"name": f"{side}_rudder_position", "joint": f"{side}_rudder_joint"})
        ET.SubElement(sensor, "jointvel", {"name": f"{side}_rudder_velocity", "joint": f"{side}_rudder_joint"})
        ET.SubElement(sensor, "jointpos", {"name": f"{side}_brake_position", "joint": f"{side}_brake_joint"})
        ET.SubElement(sensor, "jointvel", {"name": f"{side}_brake_velocity", "joint": f"{side}_brake_joint"})
    return bodies, equality, sensor


def _write(root: ET.Element, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _component_model(name: str, body_items: list[ET.Element], sections: list[ET.Element], simulation: dict[str, Any], contact: dict[str, Any]) -> ET.Element:
    root = _new_model(name, simulation, contact)
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"name": f"{name}_light", "pos": "-0.5 -0.5 2"})
    for body in body_items:
        world.append(deepcopy(body))
    for section in sections:
        root.append(deepcopy(section))
    return root


def _qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _set_qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    data.qpos[int(model.jnt_qposadr[joint_id])] = value


def build_cockpit(project_root: Path) -> dict[str, Any]:
    """Generate all Phase 4 cockpit files and run passive behavior tests."""
    project_root = project_root.resolve()
    layout = yaml.safe_load((project_root / "configs/cockpit_layout.yaml").read_text(encoding="utf-8"))
    mechanics = yaml.safe_load((project_root / "configs/pedal_parameters.yaml").read_text(encoding="utf-8"))
    contact = yaml.safe_load((project_root / "configs/contact_parameters.yaml").read_text(encoding="utf-8"))
    simulation = yaml.safe_load((project_root / "configs/simulation.yaml").read_text(encoding="utf-8"))
    derived = project_root / "models/derived/cockpit"
    scenes = project_root / "models/scenes"

    seat = _seat_body(layout, contact)
    stick, stick_sensor = _stick_body(layout, mechanics, contact)
    throttle, throttle_sensor = _throttle_body(layout, mechanics, contact)
    pedals, pedal_equality, pedal_sensor = _pedal_system(layout, mechanics, contact)
    _write(_component_model("seat", [seat], [], simulation, contact), derived / "seat.xml")
    _write(_component_model("control_stick", [stick], [stick_sensor], simulation, contact), derived / "control_stick.xml")
    _write(_component_model("throttle", [throttle], [throttle_sensor], simulation, contact), derived / "throttle.xml")
    _write(_component_model("rudder_pedals", pedals, [pedal_equality, pedal_sensor], simulation, contact), derived / "rudder_pedals.xml")

    combined = _new_model("adjustable_bionic_pilot_cockpit", simulation, contact)
    world = _add_environment(combined, layout)
    for body in [seat, stick, throttle, *pedals]:
        world.append(deepcopy(body))
    combined.append(deepcopy(pedal_equality))
    combined_sensor = ET.SubElement(combined, "sensor")
    for section in (stick_sensor, throttle_sensor, pedal_sensor):
        for sensor in section:
            combined_sensor.append(deepcopy(sensor))
    combined_path = derived / "cockpit_frame.xml"
    _write(combined, combined_path)

    expected_component_joints = {
        "seat.xml": 0,
        "control_stick.xml": 2,
        "throttle.xml": 1,
        "rudder_pedals.xml": 4,
    }
    component_loads = {
        filename: mujoco.MjModel.from_xml_path(str(derived / filename)).njnt == expected_joints
        for filename, expected_joints in expected_component_joints.items()
    }

    scene_path = scenes / "cockpit_mechanisms.xml"
    scene_path.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<mujoco model=\"cockpit_mechanism_test\">\n"
        "  <include file=\"../derived/cockpit/cockpit_frame.xml\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )
    pedal_scene = scenes / "pedal_test.xml"
    pedal_scene.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<mujoco model=\"pedal_test\">\n"
        "  <include file=\"../derived/cockpit/rudder_pedals.xml\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )

    model = mujoco.MjModel.from_xml_path(str(combined_path))
    data = mujoco.MjData(model)
    stick_config = mechanics["stick"]
    throttle_config = mechanics["throttle"]
    rudder_config = mechanics["rudder"]
    brake_config = mechanics["brakes"]
    initial = {
        "stick_roll_joint": 0.7 * stick_config["roll_range_rad"][1],
        "stick_pitch_joint": 0.7 * stick_config["pitch_range_rad"][0],
        "throttle_joint": 0.6 * throttle_config["travel_m"],
        "left_rudder_joint": 0.5 * rudder_config["pedal_travel_m"],
        "right_rudder_joint": -0.5 * rudder_config["pedal_travel_m"],
        "left_brake_joint": 0.8 * brake_config["range_rad"][1],
        "right_brake_joint": 0.8 * brake_config["range_rad"][1],
    }
    for name, value in initial.items():
        _set_qpos(model, data, name, value)
    mujoco.mj_forward(model, data)
    throttle_initial = _qpos(model, data, "throttle_joint")
    duration = float(simulation["mechanism_test_duration_s"])
    while data.time < duration:
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError(f"Cockpit mechanism numerical failure at {data.time:.6f} s")

    final = {name: _qpos(model, data, name) for name in initial}
    stick_tolerance = 0.05 * stick_config["roll_range_rad"][1]
    rudder_tolerance = 0.05 * rudder_config["pedal_travel_m"]
    brake_tolerance = 0.05 * brake_config["range_rad"][1]
    behavior = {
        "stick_returns_to_center": abs(final["stick_roll_joint"]) < stick_tolerance and abs(final["stick_pitch_joint"]) < stick_tolerance,
        "throttle_holds_position": abs(final["throttle_joint"] - throttle_initial) < 1e-6,
        "rudder_returns_to_center": abs(final["left_rudder_joint"]) < rudder_tolerance and abs(final["right_rudder_joint"]) < rudder_tolerance,
        "rudder_remains_opposed": abs(final["left_rudder_joint"] + final["right_rudder_joint"]) < 1e-8,
        "brakes_release": abs(final["left_brake_joint"]) < brake_tolerance and abs(final["right_brake_joint"]) < brake_tolerance,
        "finite_state": bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
    }
    result = {
        "load_success": True,
        "scene_load_success": mujoco.MjModel.from_xml_path(str(scene_path)).njnt == model.njnt,
        "pedal_scene_load_success": mujoco.MjModel.from_xml_path(str(pedal_scene)).njnt == 4,
        "component_loads": component_loads,
        "counts": {"joints": model.njnt, "dofs": model.nv, "actuators": model.nu, "sensors": model.nsensor, "equalities": model.neq},
        "test_duration_s": float(data.time),
        "initial_qpos": initial,
        "final_qpos": final,
        "behavior": behavior,
        "layout_status": layout["status"],
        "parameter_status": mechanics["design_status"],
        "output": str(combined_path),
        "scene": str(scene_path),
    }
    result["passed"] = bool(
        result["scene_load_success"]
        and result["pedal_scene_load_success"]
        and all(component_loads.values())
        and result["counts"]["joints"] == 7
        and result["counts"]["actuators"] == 0
        and result["counts"]["equalities"] == 1
        and all(behavior.values())
    )
    if not result["passed"]:
        raise RuntimeError(f"Cockpit mechanism validation failed: {result}")
    return result
