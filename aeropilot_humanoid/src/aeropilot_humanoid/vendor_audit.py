"""Inspect official Unitree G1 and LinkerHand O6 model sources.

Author: OpenAI Codex
Date: 2026-07-16
Purpose: Produce reproducible Phase 0/1 structural and load evidence.
Why: Model identity, kinematic topology, coupling, and provenance must be
verified before any cockpit or wrist adapter is designed.
Inputs: Official MJCF/URDF paths beneath ``third_party``.
Outputs: JSON-compatible dictionaries used by reports and tests.
Exceptions: Raises ``FileNotFoundError`` for absent sources and preserves
MuJoCo/XML parse errors with the source path in the message.
Coordinate Frame: Vendor-native frames; no transform is applied here.
Units: SI and radians, as declared by the vendor models.
Safety Notes: This module is read-only with respect to vendor trees.
Source: Unitree and LinkerHand repositories in ``source_manifest.yaml``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


FREE_JOINT = int(mujoco.mjtJoint.mjJNT_FREE)
JOINT_TYPE_NAMES = {
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
    int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
    int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
}


@dataclass(frozen=True)
class G1Joint:
    """One G1 joint mapped to its physical branch and actuator, if any."""

    index: int
    name: str
    joint_type: str
    body: str
    parent_body: str
    branch: str
    axis: list[float]
    limited: bool
    range_rad: list[float] | None
    qpos0: list[float]
    actuator: str | None


@dataclass(frozen=True)
class O6Joint:
    """One O6 URDF joint, including its official mimic relationship."""

    name: str
    joint_type: str
    parent: str
    child: str
    axis: list[float]
    limit_lower_rad: float | None
    limit_upper_rad: float | None
    effort: float | None
    velocity: float | None
    mimic_joint: str | None
    mimic_multiplier: float | None
    mimic_offset_rad: float | None
    role: str


def _name(model: mujoco.MjModel, object_type: mujoco.mjtObj, object_id: int) -> str:
    value = mujoco.mj_id2name(model, object_type, object_id)
    return value or f"unnamed_{object_id}"


def _body_is_descendant(model: mujoco.MjModel, body_id: int, ancestor_id: int) -> bool:
    current = body_id
    while current > 0:
        if current == ancestor_id:
            return True
        current = int(model.body_parentid[current])
    return False


def _qpos_width(joint_type: int) -> int:
    if joint_type == int(mujoco.mjtJoint.mjJNT_FREE):
        return 7
    if joint_type == int(mujoco.mjtJoint.mjJNT_BALL):
        return 4
    return 1


def inspect_g1(path: Path) -> dict[str, Any]:
    """Load and structurally classify the official G1 29DoF MJCF.

    Author: OpenAI Codex
    Date: 2026-07-16
    Purpose: Verify counts, mass, limits, parent-child topology, and actuators.
    Why: A filename or joint-name substring count cannot prove a 29DoF tree.
    Inputs: ``path`` to the official ``g1_29dof.xml``.
    Outputs: JSON-compatible model summary and per-joint mapping.
    Exceptions: ``FileNotFoundError`` or contextual ``RuntimeError``.
    Coordinate Frame: Official G1 MJCF frame.
    Units: metres, kilograms, seconds, radians, newtons, newton-metres.
    Safety Notes: The vendor file is only read.
    Source: Unitree ``unitree_robots/g1/g1_29dof.xml``.
    """
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Official G1 MJCF not found: {path}")
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:  # MuJoCo exposes several compiler exception types.
        raise RuntimeError(f"Failed to load official G1 MJCF {path}: {exc}") from exc

    branch_anchors = {
        "left_leg": "left_hip_pitch_link",
        "right_leg": "right_hip_pitch_link",
        "left_arm": "left_shoulder_pitch_link",
        "right_arm": "right_shoulder_pitch_link",
        # Waist is an ancestor of both arms, so it must be considered after
        # the more specific arm subtrees.
        "waist": "waist_yaw_link",
    }
    anchor_ids: dict[str, int] = {}
    for branch, body_name in branch_anchors.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise RuntimeError(f"G1 branch anchor is missing: {body_name}")
        anchor_ids[branch] = body_id

    actuator_by_joint: dict[int, str] = {}
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        actuator_by_joint[joint_id] = _name(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
        )

    joints: list[G1Joint] = []
    for joint_id in range(model.njnt):
        body_id = int(model.jnt_bodyid[joint_id])
        parent_id = int(model.body_parentid[body_id])
        branch = "floating_base"
        for candidate, anchor_id in anchor_ids.items():
            if _body_is_descendant(model, body_id, anchor_id):
                branch = candidate
                break
        joint_type = int(model.jnt_type[joint_id])
        qpos_address = int(model.jnt_qposadr[joint_id])
        qpos_width = _qpos_width(joint_type)
        limited = bool(model.jnt_limited[joint_id])
        joints.append(
            G1Joint(
                index=joint_id,
                name=_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
                joint_type=JOINT_TYPE_NAMES[joint_type],
                body=_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
                parent_body=_name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id),
                branch=branch,
                axis=[float(value) for value in model.jnt_axis[joint_id]],
                limited=limited,
                range_rad=(
                    [float(value) for value in model.jnt_range[joint_id]]
                    if limited
                    else None
                ),
                qpos0=[
                    float(value)
                    for value in model.qpos0[qpos_address : qpos_address + qpos_width]
                ],
                actuator=actuator_by_joint.get(joint_id),
            )
        )

    branch_counts = {
        branch: sum(
            joint.branch == branch and joint.joint_type != "free" for joint in joints
        )
        for branch in branch_anchors
    }
    leaves: dict[str, str] = {}
    for branch in ("left_arm", "right_arm"):
        anchor_id = anchor_ids[branch]
        candidates = [
            body_id
            for body_id in range(1, model.nbody)
            if _body_is_descendant(model, body_id, anchor_id)
            and not any(
                int(model.body_parentid[other_id]) == body_id
                for other_id in range(1, model.nbody)
            )
        ]
        if len(candidates) != 1:
            raise RuntimeError(
                f"Expected one terminal body for {branch}, found {len(candidates)}"
            )
        leaves[branch] = _name(model, mujoco.mjtObj.mjOBJ_BODY, candidates[0])

    finite = all(
        np.isfinite(values).all()
        for values in (model.body_mass, model.body_inertia, model.qpos0)
    )
    validations = {
        "29_actuated_joints": model.nu == 29 and len(actuator_by_joint) == 29,
        "left_arm_7dof": branch_counts["left_arm"] == 7,
        "right_arm_7dof": branch_counts["right_arm"] == 7,
        "left_leg_6dof_chain": branch_counts["left_leg"] == 6,
        "right_leg_6dof_chain": branch_counts["right_leg"] == 6,
        "waist_3dof": branch_counts["waist"] == 3,
        "all_arrays_finite": finite,
        "all_hinges_actuated": all(
            joint.actuator is not None
            for joint in joints
            if joint.joint_type == "hinge"
        ),
    }
    return {
        "source": str(path),
        "load_success": True,
        "model_counts": {
            "bodies_including_world": model.nbody,
            "physical_bodies": model.nbody - 1,
            "joints_including_floating_base": model.njnt,
            "actuated_hinge_joints": model.nu,
            "qpos": model.nq,
            "dofs": model.nv,
            "actuators": model.nu,
            "sensors": model.nsensor,
            "keyframes": model.nkey,
        },
        "total_mass_kg": float(np.sum(model.body_mass)),
        "branch_counts": branch_counts,
        "terminal_links": {
            "left_wrist": leaves["left_arm"],
            "right_wrist": leaves["right_arm"],
        },
        "initial_qpos": [float(value) for value in model.qpos0],
        "joints": [asdict(joint) for joint in joints],
        "validations": validations,
        "pedal_capability_status": (
            "Both six-joint leg chains are structurally complete; actual pedal "
            "reachability and contact capability remain unverified until Phase 5."
        ),
    }


def _float_attribute(element: ET.Element | None, name: str) -> float | None:
    if element is None or element.get(name) is None:
        return None
    return float(element.get(name, ""))


def _vector(text: str | None, expected: int = 3) -> list[float]:
    values = [float(value) for value in (text or "").split()]
    if len(values) != expected:
        raise ValueError(f"Expected {expected} values, got {values!r}")
    return values


def _urdf_inertia_is_positive(link: ET.Element) -> bool:
    inertial = link.find("inertial")
    if inertial is None:
        return False
    inertia = inertial.find("inertia")
    mass = inertial.find("mass")
    if inertia is None or mass is None or float(mass.get("value", "0")) <= 0.0:
        return False
    ixx = float(inertia.get("ixx", "nan"))
    ixy = float(inertia.get("ixy", "nan"))
    ixz = float(inertia.get("ixz", "nan"))
    iyy = float(inertia.get("iyy", "nan"))
    iyz = float(inertia.get("iyz", "nan"))
    izz = float(inertia.get("izz", "nan"))
    matrix = np.array(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
    )
    return bool(np.isfinite(matrix).all() and np.linalg.eigvalsh(matrix).min() > 0.0)


def inspect_o6(path: Path) -> dict[str, Any]:
    """Parse and independently compile one official O6 URDF.

    Author: OpenAI Codex
    Date: 2026-07-16
    Purpose: Preserve exact left/right topology, limits, axes, and mimic data.
    Why: O6 has six hardware inputs but eleven URDF joints; five are coupled.
    Inputs: Official left or right O6 URDF path.
    Outputs: JSON-compatible URDF and raw MuJoCo compiler summary.
    Exceptions: Contextual XML, path, or MuJoCo errors.
    Coordinate Frame: Vendor O6 root-link frame.
    Units: metres, kilograms, seconds, radians.
    Safety Notes: Does not add actuators or change passive relationships.
    Source: LinkerHand O6 URDF repository.
    """
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Official O6 URDF not found: {path}")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise RuntimeError(f"Failed to parse O6 URDF {path}: {exc}") from exc

    links = root.findall("link")
    joints_xml = root.findall("joint")
    child_links = {
        child.get("link", "")
        for joint in joints_xml
        if (child := joint.find("child")) is not None
    }
    root_links = [
        link.get("name", "") for link in links if link.get("name", "") not in child_links
    ]

    joints: list[O6Joint] = []
    for joint in joints_xml:
        parent = joint.find("parent")
        child = joint.find("child")
        axis = joint.find("axis")
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        if parent is None or child is None or axis is None:
            raise RuntimeError(f"Incomplete O6 joint in {path}: {joint.get('name')}")
        joints.append(
            O6Joint(
                name=joint.get("name", ""),
                joint_type=joint.get("type", ""),
                parent=parent.get("link", ""),
                child=child.get("link", ""),
                axis=_vector(axis.get("xyz")),
                limit_lower_rad=_float_attribute(limit, "lower"),
                limit_upper_rad=_float_attribute(limit, "upper"),
                effort=_float_attribute(limit, "effort"),
                velocity=_float_attribute(limit, "velocity"),
                mimic_joint=mimic.get("joint") if mimic is not None else None,
                mimic_multiplier=_float_attribute(mimic, "multiplier"),
                mimic_offset_rad=_float_attribute(mimic, "offset"),
                role="passive_mimic" if mimic is not None else "independent_input",
            )
        )

    mesh_paths: list[str] = []
    missing_meshes: list[str] = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename not in mesh_paths:
            mesh_paths.append(filename)
        resolved = path.parent / Path(filename.replace("/", str(Path('/'))))
        if not resolved.is_file() and filename not in missing_meshes:
            missing_meshes.append(filename)

    urdf_mass = sum(
        float(mass.get("value", "0"))
        for link in links
        if (mass := link.find("inertial/mass")) is not None
    )
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:
        raise RuntimeError(f"MuJoCo failed to compile O6 URDF {path}: {exc}") from exc
    compiled_mass = float(np.sum(model.body_mass))
    active = [joint.name for joint in joints if joint.role == "independent_input"]
    passive = [joint.name for joint in joints if joint.role == "passive_mimic"]
    finite = all(
        np.isfinite(values).all()
        for values in (model.body_mass, model.body_inertia, model.qpos0)
    )
    return {
        "source": str(path),
        "robot_name": root.get("name", ""),
        "load_success": True,
        "root_links": root_links,
        "link_count": len(links),
        "joint_count": len(joints),
        "independent_joint_count": len(active),
        "passive_mimic_joint_count": len(passive),
        "independent_joint_names": active,
        "passive_mimic_joint_names": passive,
        "joints": [asdict(joint) for joint in joints],
        "meshes": mesh_paths,
        "missing_meshes": missing_meshes,
        "urdf_total_mass_kg": urdf_mass,
        "all_link_inertias_positive_definite": all(
            _urdf_inertia_is_positive(link) for link in links
        ),
        "raw_mujoco_compile": {
            "bodies_including_world": model.nbody,
            "joints": model.njnt,
            "qpos": model.nq,
            "dofs": model.nv,
            "actuators": model.nu,
            "sensors": model.nsensor,
            "dynamic_mass_kg": compiled_mass,
            "arrays_finite": finite,
        },
        "root_mass_preserved_by_raw_compile": bool(
            np.isclose(compiled_mass, urdf_mass, rtol=1e-8, atol=1e-10)
        ),
        "conversion_note": (
            "The raw URDF compiles, but MuJoCo fuses the fixed root link into "
            "the world. The compiled dynamic mass therefore omits the root-link "
            "mass. Phase 2 must preserve the root as an attachable body and "
            "recreate all mimic constraints."
        ),
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    """Write deterministic UTF-8 JSON for machine review."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
