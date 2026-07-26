"""Convert official O6 URDFs into root-preserving MuJoCo MJCF models.

Author: OpenAI Codex
Date: 2026-07-16
Purpose: Produce separate left/right derived hand models without altering vendor files.
Why: Direct fixed-root URDF compilation fuses the palm root into the world and
does not retain URDF mimic relationships.
Inputs: One official O6 URDF and an output path under ``models/derived``.
Outputs: Loadable MJCF with full root inertia and five equality constraints.
Exceptions: Raises contextual conversion or validation errors.
Coordinate Frame: The official O6 root-link frame is preserved.
Units: SI and radians.
Safety Notes: No actuators are invented; Phase 6 must add calibrated control.
Source: Official LinkerHand O6 URDF plus MuJoCo's official URDF compiler.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .vendor_audit import inspect_o6


@dataclass(frozen=True)
class ConversionPaths:
    """Source, derived model, and standalone scene paths for one hand."""

    urdf: Path
    mjcf: Path
    scene: Path


def _set_absolute_mesh_paths(root: ET.Element, urdf: Path) -> None:
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            raise RuntimeError(f"Mesh without filename in {urdf}")
        source = (urdf.parent / filename).resolve()
        if not source.is_file():
            raise FileNotFoundError(f"O6 mesh is missing: {source}")
        mesh.set("filename", source.as_posix())


def _remove_mount_freejoint(root: ET.Element, name: str) -> None:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"joint", "freejoint"} and child.get("name") == name:
                parent.remove(child)
                removed += 1
    if removed != 1:
        raise RuntimeError(f"Expected one temporary mount joint {name}, removed {removed}")


def _rewrite_mesh_paths(root: ET.Element, urdf: Path, output: Path) -> None:
    source_mesh_dir = urdf.parent / "meshes"
    for mesh in root.findall("./asset/mesh"):
        filename = mesh.get("file")
        if not filename:
            continue
        source = source_mesh_dir / Path(filename).name
        if not source.is_file():
            raise FileNotFoundError(f"Converted mesh source is missing: {source}")
        relative = Path(os.path.relpath(source, output.parent)).as_posix()
        mesh.set("file", relative)


def _rotation_from_rpy(rpy: str) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float
    )
    rotation_y = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float
    )
    rotation_z = np.array(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float
    )
    return rotation_z @ rotation_y @ rotation_x


def _restore_exact_inertials(root: ET.Element, urdf_root: ET.Element) -> None:
    bodies = {
        body.get("name", ""): body for body in root.findall(".//body")
    }
    for link in urdf_root.findall("link"):
        link_name = link.get("name", "")
        inertial_source = link.find("inertial")
        if inertial_source is None:
            continue
        body = bodies.get(link_name)
        if body is None:
            raise RuntimeError(f"Converted MJCF body is missing for URDF link: {link_name}")
        origin = inertial_source.find("origin")
        mass = inertial_source.find("mass")
        inertia = inertial_source.find("inertia")
        if origin is None or mass is None or inertia is None:
            raise RuntimeError(f"Incomplete official inertial for O6 link: {link_name}")
        ixx = float(inertia.get("ixx", "nan"))
        ixy = float(inertia.get("ixy", "nan"))
        ixz = float(inertia.get("ixz", "nan"))
        iyy = float(inertia.get("iyy", "nan"))
        iyz = float(inertia.get("iyz", "nan"))
        izz = float(inertia.get("izz", "nan"))
        inertia_inertial = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
        )
        rotation = _rotation_from_rpy(origin.get("rpy", "0 0 0"))
        inertia_body = rotation @ inertia_inertial @ rotation.T
        inertial = body.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(body, "inertial")
        inertial.set("pos", origin.get("xyz", "0 0 0"))
        inertial.set("mass", mass.get("value", ""))
        inertial.set(
            "fullinertia",
            " ".join(
                f"{value:.17g}"
                for value in (
                    inertia_body[0, 0],
                    inertia_body[1, 1],
                    inertia_body[2, 2],
                    inertia_body[0, 1],
                    inertia_body[0, 2],
                    inertia_body[1, 2],
                )
            ),
        )
        inertial.attrib.pop("quat", None)
        inertial.attrib.pop("diaginertia", None)


def _append_mimic_equalities(root: ET.Element, audit: dict[str, Any]) -> None:
    existing = root.find("equality")
    if existing is not None:
        root.remove(existing)
    equality = ET.SubElement(root, "equality")
    for joint in audit["joints"]:
        if joint["role"] != "passive_mimic":
            continue
        offset = joint["mimic_offset_rad"] or 0.0
        multiplier = joint["mimic_multiplier"]
        ET.SubElement(
            equality,
            "joint",
            {
                "name": f"{joint['name']}_coupling",
                "joint1": joint["name"],
                "joint2": joint["mimic_joint"],
                "polycoef": f"{offset:.12g} {multiplier:.12g} 0 0 0",
            },
        )


def _write_scene(scene: Path, model: Path) -> None:
    relative = Path(os.path.relpath(model, scene.parent)).as_posix()
    scene.parent.mkdir(parents=True, exist_ok=True)
    side = "left" if "left" in model.stem else "right"
    scene.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        f"<mujoco model=\"o6_{side}_standalone_test\">\n"
        f"  <include file=\"{relative}\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )


def _joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Converted joint is missing: {name}")
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _set_joint_qpos(
    model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float
) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Converted joint is missing: {name}")
    data.qpos[int(model.jnt_qposadr[joint_id])] = value


def validate_converted_o6(
    model_path: Path, audit: dict[str, Any]
) -> dict[str, Any]:
    """Load a derived hand and perform a coupling-consistent kinematic motion test.

    Author: OpenAI Codex
    Date: 2026-07-16
    Purpose: Prove mass, topology, mesh portability, and visible joint motion.
    Why: XML generation alone does not demonstrate a valid MuJoCo model.
    Inputs: Derived MJCF and the source URDF audit dictionary.
    Outputs: Machine-readable Phase 2 validation metrics.
    Exceptions: MuJoCo load or structural validation errors.
    Coordinate Frame: Official O6 palm-root frame.
    Units: metres, kilograms, radians.
    Safety Notes: Test sets qpos directly; it is not a calibrated controller.
    Source: Derived model generated from the official O6 URDF.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    open_positions = np.array(data.xpos, copy=True)

    joint_by_name = {joint["name"]: joint for joint in audit["joints"]}
    passive_by_active: dict[str, list[dict[str, Any]]] = {}
    for joint in audit["joints"]:
        if joint["role"] == "passive_mimic":
            passive_by_active.setdefault(joint["mimic_joint"], []).append(joint)

    commanded: dict[str, float] = {}
    for active_name in audit["independent_joint_names"]:
        active = joint_by_name[active_name]
        upper = float(active["limit_upper_rad"])
        for passive in passive_by_active.get(active_name, []):
            multiplier = float(passive["mimic_multiplier"])
            offset = float(passive["mimic_offset_rad"] or 0.0)
            passive_upper = float(passive["limit_upper_rad"])
            upper = min(upper, (passive_upper - offset) / multiplier)
        value = 0.25 * upper
        commanded[active_name] = value
        _set_joint_qpos(model, data, active_name, value)
        for passive in passive_by_active.get(active_name, []):
            passive_value = (
                float(passive["mimic_offset_rad"] or 0.0)
                + float(passive["mimic_multiplier"]) * value
            )
            _set_joint_qpos(model, data, passive["name"], passive_value)

    mujoco.mj_forward(model, data)
    displacements = np.linalg.norm(data.xpos - open_positions, axis=1)
    residuals = []
    for passive_name in audit["passive_mimic_joint_names"]:
        passive = joint_by_name[passive_name]
        expected = (
            float(passive["mimic_offset_rad"] or 0.0)
            + float(passive["mimic_multiplier"])
            * _joint_qpos(model, data, passive["mimic_joint"])
        )
        residuals.append(abs(_joint_qpos(model, data, passive_name) - expected))

    output_tree = ET.parse(model_path)
    mesh_files = [
        mesh.get("file", "") for mesh in output_tree.getroot().findall("./asset/mesh")
    ]
    return {
        "load_success": True,
        "bodies_including_world": model.nbody,
        "joints": model.njnt,
        "qpos": model.nq,
        "dofs": model.nv,
        "actuators": model.nu,
        "equalities": model.neq,
        "total_mass_kg": float(np.sum(model.body_mass)),
        "source_urdf_mass_kg": float(audit["urdf_total_mass_kg"]),
        "mass_preserved": bool(
            np.isclose(
                np.sum(model.body_mass),
                audit["urdf_total_mass_kg"],
                rtol=1e-12,
                atol=1e-14,
            )
        ),
        "mass_absolute_error_kg": abs(
            float(np.sum(model.body_mass)) - float(audit["urdf_total_mass_kg"])
        ),
        "root_body_present": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, audit["root_links"][0]
        )
        > 0,
        "all_mesh_paths_relative": all(
            filename and not Path(filename).is_absolute() for filename in mesh_files
        ),
        "kinematic_motion_test": {
            "command_fraction_of_safe_range": 0.25,
            "commanded_independent_joints_rad": commanded,
            "max_body_displacement_m": float(np.max(displacements)),
            "max_mimic_residual_rad": max(residuals, default=0.0),
            "passed": bool(
                np.max(displacements) > 0.001 and max(residuals, default=0.0) < 1e-12
            ),
        },
    }


def convert_o6(paths: ConversionPaths) -> dict[str, Any]:
    """Convert one official hand through MuJoCo and restore omitted semantics.

    Author: OpenAI Codex
    Date: 2026-07-16
    Purpose: Build one canonical, attachable, root-preserving O6 MJCF.
    Why: Direct URDF compilation fuses the root into world and drops mimic tags.
    Inputs: Official URDF, derived MJCF path, and standalone scene path.
    Outputs: Phase 2 conversion validation dictionary.
    Exceptions: Contextual conversion/validation errors.
    Coordinate Frame: Vendor root-link frame, unchanged.
    Units: SI and radians.
    Safety Notes: Adds equality constraints only, no guessed actuation strength.
    Source: Official LinkerHand URDF compiled with official MuJoCo APIs.
    """
    urdf = paths.urdf.resolve()
    output = paths.mjcf.resolve()
    audit = inspect_o6(urdf)
    if len(audit["root_links"]) != 1:
        raise RuntimeError(f"Expected one O6 root link in {urdf}: {audit['root_links']}")
    root_link = audit["root_links"][0]
    prefix = root_link.split("_", maxsplit=1)[0]
    mount_joint = f"{prefix}_temporary_mount_free"
    dummy_root = f"{prefix}_conversion_world"

    urdf_tree = ET.parse(urdf)
    urdf_root = urdf_tree.getroot()
    _set_absolute_mesh_paths(urdf_root, urdf)
    urdf_root.insert(0, ET.Element("link", {"name": dummy_root}))
    floating = ET.SubElement(
        urdf_root, "joint", {"name": mount_joint, "type": "floating"}
    )
    ET.SubElement(floating, "parent", {"link": dummy_root})
    ET.SubElement(floating, "child", {"link": root_link})

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="o6_convert_") as directory:
        temporary = Path(directory)
        adapted_urdf = temporary / f"{prefix}_adapted.urdf"
        compiled_mjcf = temporary / f"{prefix}_compiled.xml"
        ET.indent(urdf_tree, space="  ")
        urdf_tree.write(adapted_urdf, encoding="utf-8", xml_declaration=True)
        try:
            model = mujoco.MjModel.from_xml_path(str(adapted_urdf))
            mujoco.mj_saveLastXML(str(compiled_mjcf), model)
        except Exception as exc:
            raise RuntimeError(f"MuJoCo O6 conversion failed for {urdf}: {exc}") from exc
        tree = ET.parse(compiled_mjcf)

    root = tree.getroot()
    root.set("model", audit["robot_name"] + "_derived")
    _remove_mount_freejoint(root, mount_joint)
    _rewrite_mesh_paths(root, urdf, output)
    _restore_exact_inertials(root, urdf_root)
    _append_mimic_equalities(root, audit)
    root.insert(
        0,
        ET.Comment(
            " Derived from the official LinkerHand URDF; root inertia and mimic "
            "constraints restored. No vendor file was modified. "
        ),
    )
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    _write_scene(paths.scene.resolve(), output)

    validation = validate_converted_o6(output, audit)
    scene_model = mujoco.MjModel.from_xml_path(str(paths.scene.resolve()))
    validation["standalone_scene_load_success"] = scene_model.njnt == 11
    validation["source"] = str(urdf)
    validation["output"] = str(output)
    validation["scene"] = str(paths.scene.resolve())
    validation["passive_constraints"] = [
        {
            "joint": joint["name"],
            "source_joint": joint["mimic_joint"],
            "multiplier": joint["mimic_multiplier"],
            "offset_rad": joint["mimic_offset_rad"],
        }
        for joint in audit["joints"]
        if joint["role"] == "passive_mimic"
    ]
    required = (
        validation["mass_preserved"]
        and validation["root_body_present"]
        and validation["all_mesh_paths_relative"]
        and validation["equalities"] == 5
        and validation["actuators"] == 0
        and validation["kinematic_motion_test"]["passed"]
        and validation["standalone_scene_load_success"]
    )
    validation["passed"] = bool(required)
    if not required:
        raise RuntimeError(f"Derived O6 validation failed: {validation}")
    return validation
