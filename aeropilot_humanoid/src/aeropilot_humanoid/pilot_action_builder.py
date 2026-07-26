"""Build a humanoid cockpit-control action scene.

Author: OpenAI Codex
Date: 2026-07-25
Purpose: Combine the sourced G1+O6 humanoid with cockpit stick/throttle action
keyframes.
Why: The pilot-control scene must be reproducible and traceable to the audited
robot sources and local cockpit-control asset.
Inputs: Derived G1+O6 MJCF, derived cockpit MJCF, local FBX reference file.
Outputs: A combined MJCF scene, include wrapper, and validation metrics.
Exceptions: Raises source, compile, or reachability validation errors.
Coordinate Frame: X aircraft forward, Y aircraft left, Z aircraft up.
Units: SI and radians.
Safety Notes: Keyframes are geometric action poses, not trained contact policy.
Source: Unitree G1, LinkerHand O6, and local 3d66 cockpit-control FBX.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from aeropilot_humanoid.cockpit_builder import build_cockpit
from aeropilot_humanoid.g1_o6_builder import build_g1_o6


LEFT_ARM = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_ARM = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def _values(values: np.ndarray | list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{float(value):.10g}" for value in values)


def _section(root: ET.Element, tag: str) -> ET.Element:
    section = root.find(tag)
    if section is None:
        section = ET.SubElement(root, tag)
    return section


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Scene joint is missing: {name}")
    return int(joint_id)


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise RuntimeError(f"Scene body is missing: {name}")
    return int(body_id)


def _site_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise RuntimeError(f"Scene site is missing: {name}")
    return data.site_xpos[site_id].copy()


def _set_joint(model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float) -> None:
    joint_id = _joint_id(model, name)
    data.qpos[int(model.jnt_qposadr[joint_id])] = value


def _clamp_joint(model: mujoco.MjModel, data: mujoco.MjData, joint_id: int) -> None:
    if not bool(model.jnt_limited[joint_id]):
        return
    address = int(model.jnt_qposadr[joint_id])
    lower, upper = model.jnt_range[joint_id]
    data.qpos[address] = np.clip(data.qpos[address], lower, upper)


def _merge_visual_include(root: ET.Element, source_path: Path, output: Path) -> bool:
    if not source_path.is_file():
        return False
    source = ET.parse(source_path).getroot()
    target_asset = _section(root, "asset")
    source_asset = source.find("asset")
    if source_asset is not None:
        for child in source_asset:
            copied = deepcopy(child)
            filename = copied.get("file")
            if filename:
                resolved = (source_path.parent / filename).resolve()
                copied.set("file", Path(os.path.relpath(resolved, output.parent)).as_posix())
            target_asset.append(copied)

    target_world = _section(root, "worldbody")
    source_world = source.find("worldbody")
    if source_world is not None:
        for child in source_world:
            target_world.append(deepcopy(child))
    return True


def _merge_cockpit(
    humanoid_path: Path,
    cockpit_path: Path,
    output: Path,
    visual_controls_path: Path,
) -> bool:
    humanoid_tree = ET.parse(humanoid_path)
    humanoid = humanoid_tree.getroot()
    humanoid.set("model", "g1_o6_bionic_pilot_controls")
    cockpit = ET.parse(cockpit_path).getroot()

    world = _section(humanoid, "worldbody")
    cockpit_world = cockpit.find("worldbody")
    if cockpit_world is None:
        raise RuntimeError(f"Cockpit MJCF has no worldbody: {cockpit_path}")
    skipped_cockpit_bodies = {"control_stick_base", "throttle_base"}
    for child in cockpit_world:
        if child.tag == "body" and child.get("name") in skipped_cockpit_bodies:
            continue
        world.append(deepcopy(child))

    for tag in ("equality", "sensor"):
        source = cockpit.find(tag)
        if source is None:
            continue
        target = _section(humanoid, tag)
        for child in source:
            joint = child.get("joint", "")
            site = child.get("site", "")
            if joint.startswith("stick_") or joint.startswith("throttle_"):
                continue
            if site.startswith("stick_") or site.startswith("throttle_"):
                continue
            target.append(deepcopy(child))

    ET.SubElement(
        world,
        "camera",
        {
            "name": "pilot_controls_view",
            "pos": "-0.35 -1.15 1.10",
            "xyaxes": "0.92 -0.39 0 0.20 0.47 0.86",
            "fovy": "50",
        },
    )
    ET.SubElement(
        world,
        "camera",
        {
            "name": "pilot_side_view",
            "pos": "-0.85 -0.70 0.95",
            "xyaxes": "0.64 -0.77 0 0.32 0.27 0.91",
            "fovy": "55",
        },
    )
    imported_visual_controls = _merge_visual_include(humanoid, visual_controls_path, output)

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(humanoid_tree, space="  ")
    humanoid_tree.write(output, encoding="utf-8", xml_declaration=True)
    return imported_visual_controls


def _apply_pilot_seed(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    data.qpos[:] = model.qpos0
    data.qpos[:7] = [0.08, 0.0, 0.74, 1.0, 0.0, 0.0, 0.0]
    for side in ("left", "right"):
        _set_joint(model, data, f"{side}_hip_pitch_joint", -1.15)
        _set_joint(model, data, f"{side}_knee_joint", 1.75)
        _set_joint(model, data, f"{side}_ankle_pitch_joint", -0.55)
    _set_joint(model, data, "waist_pitch_joint", 0.18)

    arm_seed = {
        "left_shoulder_pitch_joint": 0.30,
        "left_shoulder_roll_joint": 0.65,
        "left_shoulder_yaw_joint": 0.20,
        "left_elbow_joint": 1.10,
        "right_shoulder_pitch_joint": 0.30,
        "right_shoulder_roll_joint": -0.65,
        "right_shoulder_yaw_joint": -0.20,
        "right_elbow_joint": 1.10,
    }
    for name, value in arm_seed.items():
        _set_joint(model, data, name, value)

    hand_inputs = {
        "thumb_cmc_yaw": 0.45,
        "thumb_cmc_pitch": 0.35,
        "index_mcp_pitch": 0.75,
        "middle_mcp_pitch": 0.75,
        "ring_mcp_pitch": 0.55,
        "pinky_mcp_pitch": 0.55,
    }
    for prefix, thumb_ip_gain in (("lh", 2.29), ("rh", 1.86)):
        for suffix, value in hand_inputs.items():
            _set_joint(model, data, f"{prefix}_{suffix}", value)
        _set_joint(model, data, f"{prefix}_thumb_ip", thumb_ip_gain * hand_inputs["thumb_cmc_pitch"])
        for suffix in ("index", "middle", "ring", "pinky"):
            _set_joint(model, data, f"{prefix}_{suffix}_dip", 0.89 * hand_inputs[f"{suffix}_mcp_pitch"])


def _solve_arm(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
    hand_body: str,
    target: np.ndarray,
) -> float:
    joint_names = LEFT_ARM if side == "left" else RIGHT_ARM
    joint_ids = [_joint_id(model, name) for name in joint_names]
    body_id = _body_id(model, hand_body)
    columns = [int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids]

    for _ in range(350):
        mujoco.mj_forward(model, data)
        error = target - data.xpos[body_id]
        if np.linalg.norm(error) < 0.012:
            break
        jac_pos = np.zeros((3, model.nv))
        jac_rot = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jac_pos, jac_rot, body_id)
        jac = jac_pos[:, columns]
        damp = 0.01
        delta = jac.T @ np.linalg.solve(jac @ jac.T + damp * np.eye(3), error)
        delta = np.clip(delta, -0.04, 0.04)
        for index, joint_id in enumerate(joint_ids):
            data.qpos[int(model.jnt_qposadr[joint_id])] += delta[index]
            _clamp_joint(model, data, joint_id)

    mujoco.mj_forward(model, data)
    return float(np.linalg.norm(target - data.xpos[body_id]))


def _keyframe_qpos(
    model: mujoco.MjModel,
    action: dict[str, Any],
) -> tuple[np.ndarray, dict[str, float]]:
    data = mujoco.MjData(model)
    _apply_pilot_seed(model, data)
    _set_joint(model, data, "throttle_joint", float(action["throttle_m"]))
    _set_joint(model, data, "stick_pitch_joint", float(action["stick_pitch_rad"]))
    _set_joint(model, data, "stick_roll_joint", float(action["stick_roll_rad"]))
    for joint_name, value in action.get("joint_overrides", {}).items():
        _set_joint(model, data, joint_name, float(value))

    left_target = _site_position(model, data, "throttle_force_site") + np.array([-0.02, 0.0, 0.0])
    right_target = _site_position(model, data, "stick_force_site") + np.array([0.0, 0.0, -0.02])
    left_error = _solve_arm(model, data, "left", "lh_hand_base_link", left_target)
    right_error = _solve_arm(model, data, "right", "rh_hand_base_link", right_target)
    return data.qpos.copy(), {
        "left_hand_to_throttle_m": left_error,
        "right_hand_to_stick_m": right_error,
    }


def _add_keyframes(
    output: Path,
    model: mujoco.MjModel,
    actions: list[dict[str, Any]],
) -> dict[str, dict[str, float]]:
    tree = ET.parse(output)
    root = tree.getroot()
    existing = root.find("keyframe")
    if existing is not None:
        root.remove(existing)
    keyframe = ET.SubElement(root, "keyframe")

    errors: dict[str, dict[str, float]] = {}
    for index, action in enumerate(actions):
        qpos, action_errors = _keyframe_qpos(model, action)
        name = str(action["name"])
        errors[name] = action_errors
        ET.SubElement(
            keyframe,
            "key",
            {
                "name": name,
                "time": f"{index * 1.25:.2f}",
                "qpos": _values(qpos),
            },
        )

    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return errors


def _write_scene(scene: Path, output: Path) -> None:
    scene.parent.mkdir(parents=True, exist_ok=True)
    relative = Path(os.path.relpath(output, scene.parent)).as_posix()
    scene.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<mujoco model=\"bionic_pilot_control_action_scene\">\n"
        f"  <include file=\"{relative}\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )


def build_pilot_control_action(project_root: Path) -> dict[str, Any]:
    """Generate the G1+O6 pilot-control action scene and validate keyframes."""
    project_root = project_root.resolve()
    humanoid = project_root / "models/derived/humanoid/g1_o6_full.xml"
    humanoid_scene = project_root / "models/scenes/g1_o6_test.xml"
    cockpit = project_root / "models/derived/cockpit/cockpit_frame.xml"
    if not humanoid.is_file():
        build_g1_o6(project_root, humanoid, humanoid_scene)
    if not cockpit.is_file():
        build_cockpit(project_root)

    output = project_root / "models/derived/pilot/g1_o6_cockpit_action.xml"
    scene = project_root / "models/scenes/pilot_control_action.xml"
    fbx = project_root.parent / "3d66.com_JDH5455235936.fbx"
    converted_controls = project_root / "models/derived/cockpit/imported_3d66/fbx_controls.xml"
    imported_visual_controls = _merge_cockpit(humanoid, cockpit, output, converted_controls)
    model = mujoco.MjModel.from_xml_path(str(output))

    actions: list[dict[str, Any]] = [
        {
            "name": "pilot_ready_on_controls",
            "throttle_m": 0.02,
            "stick_pitch_rad": 0.0,
            "stick_roll_rad": 0.0,
        },
        {
            "name": "left_hand_pushes_throttle_forward",
            "throttle_m": 0.14,
            "stick_pitch_rad": 0.0,
            "stick_roll_rad": 0.0,
        },
        {
            "name": "left_thumb_toggles_throttle_switch",
            "throttle_m": 0.14,
            "stick_pitch_rad": 0.0,
            "stick_roll_rad": 0.0,
            "joint_overrides": {
                "lh_thumb_cmc_yaw": 0.72,
                "lh_thumb_cmc_pitch": 0.48,
                "lh_thumb_ip": 1.0992,
                "lh_index_mcp_pitch": 0.92,
                "lh_index_dip": 0.8188,
            },
        },
        {
            "name": "right_hand_deflects_control_stick",
            "throttle_m": 0.14,
            "stick_pitch_rad": -0.22,
            "stick_roll_rad": -0.12,
        },
    ]
    reach_errors = _add_keyframes(output, model, actions)
    _write_scene(scene, output)

    final_model = mujoco.MjModel.from_xml_path(str(output))
    scene_model = mujoco.MjModel.from_xml_path(str(scene))
    max_reach_error = max(
        max(errors.values()) for errors in reach_errors.values()
    )
    result = {
        "load_success": True,
        "scene_load_success": scene_model.nq == final_model.nq,
        "output": str(output),
        "scene": str(scene),
        "local_control_fbx": {
            "path": str(fbx),
            "exists": fbx.is_file(),
            "size_bytes": fbx.stat().st_size if fbx.is_file() else None,
            "status": (
                "Converted through Blender to split MuJoCo OBJ meshes. The "
                "pilot scene drives the converted model's throttle_joint, "
                "stick_roll_joint, and stick_pitch_joint directly."
            ),
        },
        "converted_control_xml": {
            "path": str(converted_controls),
            "exists": converted_controls.is_file(),
            "imported_into_action_scene": imported_visual_controls,
        },
        "online_humanoid_sources": {
            "unitree_g1": "https://github.com/unitreerobotics/unitree_mujoco",
            "unitree_g1_description": "https://github.com/unitreerobotics/unitree_ros",
            "linkerhand_o6": "https://github.com/linker-bot/linkerhand-urdf",
        },
        "keyframes": [str(action["name"]) for action in actions],
        "counts": {
            "joints": final_model.njnt,
            "qpos": final_model.nq,
            "actuators": final_model.nu,
            "equalities": final_model.neq,
            "keyframes": final_model.nkey,
        },
        "reach_errors_m": reach_errors,
        "max_reach_error_m": max_reach_error,
        "validation_threshold_m": 0.03,
        "action_scope": (
            "Geometric keyframe storyboard for humanoid pilot manipulation; "
            "contact-rich grasping and policy control remain unclaimed."
        ),
    }
    result["passed"] = bool(
        result["scene_load_success"]
        and result["local_control_fbx"]["exists"]
        and result["converted_control_xml"]["imported_into_action_scene"]
        and result["counts"]["keyframes"] == len(actions)
        and max_reach_error < result["validation_threshold_m"]
    )
    if not result["passed"]:
        raise RuntimeError(f"Pilot control action validation failed: {result}")
    return result
