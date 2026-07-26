"""Build and control a hand-following MuJoCo cockpit scene.

The right-hand world position drives the two-axis center stick. The left-hand
world position is projected onto the aircraft-forward axis and drives the
single-axis throttle. Coordinates use X forward, Y left, Z up.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from aeropilot_humanoid.pilot_action_builder import build_pilot_control_action


STICK_ROLL_ACTUATOR = "stick_roll_hand_follow"
STICK_PITCH_ACTUATOR = "stick_pitch_hand_follow"
THROTTLE_ACTUATOR = "throttle_hand_follow"


@dataclass(frozen=True)
class HandFollowCommand:
    """Joint targets derived from right- and left-hand world positions."""

    stick_roll_rad: float
    stick_pitch_rad: float
    throttle_m: float


def _section(root: ET.Element, tag: str) -> ET.Element:
    section = root.find(tag)
    if section is None:
        section = ET.SubElement(root, tag)
    return section


def _joint_range(root: ET.Element, name: str) -> str:
    joint = root.find(f".//joint[@name='{name}']")
    if joint is None or not joint.get("range"):
        raise RuntimeError(f"Limited scene joint is missing: {name}")
    return str(joint.get("range"))


def _write_scene(scene: Path, output: Path) -> None:
    scene.parent.mkdir(parents=True, exist_ok=True)
    relative = Path(os.path.relpath(output, scene.parent)).as_posix()
    scene.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<mujoco model=\"bionic_pilot_hand_follow_scene\">\n"
        f"  <include file=\"{relative}\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )


def build_hand_follow_scene(project_root: Path) -> dict[str, Any]:
    """Generate a loadable scene with position servos for stick and throttle."""
    project_root = project_root.resolve()
    action = build_pilot_control_action(project_root)
    source = Path(action["output"])
    output = project_root / "models/derived/pilot/g1_o6_hand_follow_controls.xml"
    scene = project_root / "models/scenes/hand_follow_controls.xml"

    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "g1_o6_bionic_pilot_hand_follow")
    actuator = _section(root, "actuator")
    for name in (STICK_ROLL_ACTUATOR, STICK_PITCH_ACTUATOR, THROTTLE_ACTUATOR):
        existing = actuator.find(f"./*[@name='{name}']")
        if existing is not None:
            actuator.remove(existing)

    ET.SubElement(
        actuator,
        "position",
        {
            "name": STICK_ROLL_ACTUATOR,
            "joint": "stick_roll_joint",
            "kp": "120",
            "kv": "0.3",
            "ctrlrange": _joint_range(root, "stick_roll_joint"),
            "forcerange": "-25 25",
        },
    )
    ET.SubElement(
        actuator,
        "position",
        {
            "name": STICK_PITCH_ACTUATOR,
            "joint": "stick_pitch_joint",
            "kp": "120",
            "kv": "0.3",
            "ctrlrange": _joint_range(root, "stick_pitch_joint"),
            "forcerange": "-25 25",
        },
    )
    ET.SubElement(
        actuator,
        "position",
        {
            "name": THROTTLE_ACTUATOR,
            "joint": "throttle_joint",
            "kp": "250",
            "kv": "25",
            "ctrlrange": _joint_range(root, "throttle_joint"),
            "forcerange": "-80 80",
        },
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    _write_scene(scene, output)

    model = mujoco.MjModel.from_xml_path(str(scene))
    actuator_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        for actuator_id in range(model.nu)
    }
    required = {
        STICK_ROLL_ACTUATOR,
        STICK_PITCH_ACTUATOR,
        THROTTLE_ACTUATOR,
    }
    result = {
        "output": str(output),
        "scene": str(scene),
        "load_success": True,
        "actuators": model.nu,
        "added_actuators": sorted(required),
        "joint_axes": {
            "stick": ["roll", "pitch"],
            "throttle": ["aircraft_forward_x"],
        },
        "passed": required.issubset(actuator_names),
    }
    if not result["passed"]:
        raise RuntimeError(f"Hand-follow scene validation failed: {result}")
    return result


def _object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise RuntimeError(f"Scene object is missing: {name}")
    return int(object_id)


class HandFollowController:
    """Map hand world positions to the cockpit position servos."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self._control_root_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "imported_3d66_joystick_throttle",
        )
        self._stick_pivot_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "fbx_stick_roll_frame",
        )
        self._stick_site_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "stick_force_site",
        )
        self._throttle_site_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "throttle_force_site",
        )
        self._right_hand_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "rh_hand_base_link",
        )
        self._left_hand_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "lh_hand_base_link",
        )
        self._stick_roll_joint_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "stick_roll_joint",
        )
        self._stick_pitch_joint_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "stick_pitch_joint",
        )
        self._throttle_joint_id = _object_id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "throttle_joint",
        )
        self._actuator_ids = (
            _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, STICK_ROLL_ACTUATOR),
            _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, STICK_PITCH_ACTUATOR),
            _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, THROTTLE_ACTUATOR),
        )

        mujoco.mj_forward(model, data)
        self._stick_site_local = model.site_pos[self._stick_site_id].copy()
        self._stick_tip_from_hand_world = (
            data.site_xpos[self._stick_site_id] - data.xpos[self._right_hand_id]
        ).copy()
        self._left_hand_reference_world = data.xpos[self._left_hand_id].copy()
        self._throttle_reference_m = self._joint_position(
            self._throttle_joint_id
        )

    def _joint_position(self, joint_id: int) -> float:
        address = int(self.model.jnt_qposadr[joint_id])
        return float(self.data.qpos[address])

    def _clip_joint(self, joint_id: int, value: float) -> float:
        if not bool(self.model.jnt_limited[joint_id]):
            return value
        lower, upper = self.model.jnt_range[joint_id]
        return float(np.clip(value, lower, upper))

    def command(
        self,
        right_hand_world: np.ndarray | None = None,
        left_hand_world: np.ndarray | None = None,
    ) -> HandFollowCommand:
        """Convert hand positions into limited stick and throttle targets."""
        mujoco.mj_forward(self.model, self.data)
        if right_hand_world is None:
            right_hand_world = self.data.xpos[self._right_hand_id]
        if left_hand_world is None:
            left_hand_world = self.data.xpos[self._left_hand_id]
        right_hand_world = np.asarray(right_hand_world, dtype=float)
        left_hand_world = np.asarray(left_hand_world, dtype=float)
        if right_hand_world.shape != (3,) or left_hand_world.shape != (3,):
            raise ValueError("Hand positions must each contain exactly three values")

        target_tip_world = right_hand_world + self._stick_tip_from_hand_world
        pivot_world = self.data.xpos[self._stick_pivot_id]
        root_rotation = self.data.xmat[self._control_root_id].reshape(3, 3)
        target_local = root_rotation.T @ (target_tip_world - pivot_world)
        target_norm = float(np.linalg.norm(target_local))
        site_norm = float(np.linalg.norm(self._stick_site_local))
        if target_norm <= 1e-9 or site_norm <= 1e-9:
            raise RuntimeError("Stick grip vector has zero length")
        x, y, z = target_local * (site_norm / target_norm)
        x0, y0, z0 = self._stick_site_local
        xz_radius = float(np.hypot(x0, z0))
        if xz_radius <= 1e-9:
            raise RuntimeError("Stick grip vector cannot define pitch")
        pitch = np.arcsin(np.clip(x / xz_radius, -1.0, 1.0)) - np.arctan2(
            x0,
            z0,
        )
        pitch_plane_z = -x0 * np.sin(pitch) + z0 * np.cos(pitch)
        roll = np.arctan2(y0, pitch_plane_z) - np.arctan2(y, z)

        throttle_axis_world = (
            root_rotation @ self.model.jnt_axis[self._throttle_joint_id]
        )
        throttle = self._throttle_reference_m + float(
            np.dot(
                left_hand_world - self._left_hand_reference_world,
                throttle_axis_world,
            )
        )
        return HandFollowCommand(
            stick_roll_rad=self._clip_joint(
                self._stick_roll_joint_id,
                float(roll),
            ),
            stick_pitch_rad=self._clip_joint(
                self._stick_pitch_joint_id,
                float(pitch),
            ),
            throttle_m=self._clip_joint(
                self._throttle_joint_id,
                throttle,
            ),
        )

    def apply(self, command: HandFollowCommand) -> None:
        """Write the three position targets to mjData.ctrl."""
        values = (
            command.stick_roll_rad,
            command.stick_pitch_rad,
            command.throttle_m,
        )
        for actuator_id, value in zip(self._actuator_ids, values, strict=True):
            self.data.ctrl[actuator_id] = value

    def update(
        self,
        right_hand_world: np.ndarray | None = None,
        left_hand_world: np.ndarray | None = None,
    ) -> HandFollowCommand:
        """Compute and apply one control update."""
        command = self.command(right_hand_world, left_hand_world)
        self.apply(command)
        return command


def simulate_hand_follow_sequence(
    project_root: Path,
    duration_s: float = 3.0,
) -> dict[str, Any]:
    """Validate that the control servos follow the existing hand motion."""
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")

    from aeropilot_humanoid.pilot_control_sequence import (
        KEYFRAME_NAMES,
        _keyframe_id,
        _sample_keyframes,
    )

    build = build_hand_follow_scene(project_root)
    model = mujoco.MjModel.from_xml_path(build["scene"])
    data = mujoco.MjData(model)
    key_qpos = [
        model.key_qpos[_keyframe_id(model, name)].copy()
        for name in KEYFRAME_NAMES
    ]
    data.qpos[:] = key_qpos[0]
    mujoco.mj_forward(model, data)
    controller = HandFollowController(model, data)

    control_joint_names = (
        "stick_roll_joint",
        "stick_pitch_joint",
        "throttle_joint",
    )
    qpos_addresses = np.array(
        [
            int(model.jnt_qposadr[_object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in control_joint_names
        ]
    )
    dof_addresses = np.array(
        [
            int(model.jnt_dofadr[_object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in control_joint_names
        ]
    )
    step_count = max(3, int(round(duration_s / model.opt.timestep)) + 1)
    initial_controls = data.qpos[qpos_addresses].copy()
    final_command = controller.command()

    for step_index in range(step_count):
        robot_target = _sample_keyframes(key_qpos, step_index, step_count)
        control_qpos = data.qpos[qpos_addresses].copy()
        control_qvel = data.qvel[dof_addresses].copy()
        data.qpos[:] = robot_target
        data.qpos[qpos_addresses] = control_qpos
        data.qvel[:] = 0.0
        data.qvel[dof_addresses] = control_qvel
        mujoco.mj_forward(model, data)
        final_command = controller.update()
        mujoco.mj_step(model, data)

    final_controls = data.qpos[qpos_addresses].copy()
    final_targets = np.array(
        [
            final_command.stick_roll_rad,
            final_command.stick_pitch_rad,
            final_command.throttle_m,
        ]
    )
    final_error = np.abs(final_controls - final_targets)

    left_hand = data.xpos[controller._left_hand_id].copy()
    right_hand = data.xpos[controller._right_hand_id].copy()
    axial = controller.command(right_hand, left_hand + np.array([0.04, 0.0, 0.0]))
    disturbed = controller.command(
        right_hand,
        left_hand + np.array([0.04, 0.08, -0.06]),
    )
    throttle_cross_axis_error = abs(axial.throttle_m - disturbed.throttle_m)

    result = {
        **build,
        "status": "passed_hand_position_follow",
        "duration_s": duration_s,
        "step_count": step_count,
        "initial_controls": {
            name: float(value)
            for name, value in zip(control_joint_names, initial_controls, strict=True)
        },
        "final_targets": {
            name: float(value)
            for name, value in zip(control_joint_names, final_targets, strict=True)
        },
        "final_controls": {
            name: float(value)
            for name, value in zip(control_joint_names, final_controls, strict=True)
        },
        "final_abs_error": {
            name: float(value)
            for name, value in zip(control_joint_names, final_error, strict=True)
        },
        "control_motion": {
            name: float(final - initial)
            for name, final, initial in zip(
                control_joint_names,
                final_controls,
                initial_controls,
                strict=True,
            )
        },
        "throttle_cross_axis_error_m": float(throttle_cross_axis_error),
        "coordinate_frame": {
            "x": "aircraft_forward",
            "y": "aircraft_left",
            "z": "aircraft_up",
        },
    }
    result["passed"] = bool(
        abs(result["control_motion"]["stick_roll_joint"]) > 0.08
        and abs(result["control_motion"]["stick_pitch_joint"]) > 0.15
        and result["control_motion"]["throttle_joint"] > 0.10
        and final_error[0] < 0.02
        and final_error[1] < 0.02
        and final_error[2] < 0.005
        and throttle_cross_axis_error < 1e-12
    )
    if not result["passed"]:
        raise RuntimeError(f"Hand-follow sequence validation failed: {result}")
    return result
