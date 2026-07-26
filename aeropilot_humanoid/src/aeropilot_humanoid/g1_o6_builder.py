"""Build a mass-correct G1 29DoF model with independent left/right O6 hands.

Author: OpenAI Codex
Date: 2026-07-16
Purpose: Generate the Phase 3 combined humanoid model from audited sources.
Why: The official G1 MJCF combines a removable 0.170 kg rubber hand into each
wrist-yaw inertia; simply adding O6 would double-count mass.
Inputs: Official G1 MJCF/URDF reference, derived O6 MJCFs, adapter YAML.
Outputs: A loadable combined MJCF and validation metrics.
Exceptions: Raises contextual source, topology, mass, or frame errors.
Coordinate Frame: G1 vendor frames; adapter mappings are in wrist local frames.
Units: SI and radians.
Safety Notes: No O6 actuators are invented; adapters are fixed mounting frames,
not weld constraints used to fake grasping.
Source: Unitree and LinkerHand sources locked in ``source_manifest.yaml``.
"""

from __future__ import annotations

from copy import deepcopy
import os
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


def _body(root: ET.Element, name: str) -> ET.Element:
    result = root.find(f".//body[@name='{name}']")
    if result is None:
        raise RuntimeError(f"MJCF body is missing: {name}")
    return result


def _urdf_link(root: ET.Element, name: str) -> ET.Element:
    for link in root.findall("link"):
        if link.get("name") == name:
            return link
    raise RuntimeError(f"Unitree URDF link is missing: {name}")


def _urdf_joint(root: ET.Element, name: str) -> ET.Element:
    for joint in root.findall("joint"):
        if joint.get("name") == name:
            return joint
    raise RuntimeError(f"Unitree URDF joint is missing: {name}")


def _replace_wrist_inertial(body: ET.Element, reference_link: ET.Element) -> float:
    source = reference_link.find("inertial")
    target = body.find("inertial")
    if source is None or target is None:
        raise RuntimeError(f"Missing wrist inertial for {body.get('name')}")
    origin = source.find("origin")
    mass = source.find("mass")
    inertia = source.find("inertia")
    if origin is None or mass is None or inertia is None:
        raise RuntimeError(f"Incomplete wrist inertial for {body.get('name')}")
    if origin.get("rpy", "0 0 0") != "0 0 0":
        raise RuntimeError("The audited Unitree wrist inertial is no longer axis-aligned")
    target.set("pos", origin.get("xyz", ""))
    target.set("mass", mass.get("value", ""))
    target.set(
        "fullinertia",
        " ".join(
            inertia.get(name, "")
            for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        ),
    )
    target.attrib.pop("quat", None)
    target.attrib.pop("diaginertia", None)
    return float(mass.get("value", "0"))


def _remove_rubber_hand(root: ET.Element, mesh_name: str) -> int:
    removed_geoms = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("mesh") == mesh_name:
                parent.remove(child)
                removed_geoms += 1
    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("G1 MJCF has no asset section")
    meshes = [mesh for mesh in asset.findall("mesh") if mesh.get("name") == mesh_name]
    for mesh in meshes:
        asset.remove(mesh)
    if removed_geoms != 1 or len(meshes) != 1:
        raise RuntimeError(
            f"Expected one G1 rubber-hand geom/asset for {mesh_name}, got "
            f"{removed_geoms}/{len(meshes)}"
        )
    return removed_geoms


def _merge_hand(
    combined_root: ET.Element,
    hand_path: Path,
    output: Path,
    side: str,
    adapter: dict[str, Any],
) -> None:
    hand_root = ET.parse(hand_path).getroot()
    combined_asset = combined_root.find("asset")
    hand_asset = hand_root.find("asset")
    hand_world = hand_root.find("worldbody")
    if combined_asset is None or hand_asset is None or hand_world is None:
        raise RuntimeError(f"Incomplete derived O6 MJCF: {hand_path}")
    hand_bodies = hand_world.findall("body")
    if len(hand_bodies) != 1:
        raise RuntimeError(f"Expected one O6 root body in {hand_path}")

    mesh_names: dict[str, str] = {}
    prefix = f"o6_{side}_"
    for source_mesh in hand_asset.findall("mesh"):
        mesh = deepcopy(source_mesh)
        old_name = mesh.get("name", "")
        new_name = prefix + old_name
        mesh_names[old_name] = new_name
        mesh.set("name", new_name)
        source_file = (hand_path.parent / mesh.get("file", "")).resolve()
        if not source_file.is_file():
            raise FileNotFoundError(f"O6 mesh is missing during merge: {source_file}")
        mesh.set("file", Path(os.path.relpath(source_file, output.parent)).as_posix())
        combined_asset.append(mesh)

    hand_body = deepcopy(hand_bodies[0])
    for geom in hand_body.findall(".//geom"):
        old_name = geom.get("mesh")
        if old_name:
            geom.set("mesh", mesh_names[old_name])
    adapter_body = ET.Element(
        "body",
        {
            "name": adapter["adapter_body"],
            "pos": " ".join(str(value) for value in adapter["position_m"]),
            "quat": " ".join(str(value) for value in adapter["quaternion_wxyz"]),
        },
    )
    adapter_body.append(hand_body)
    _body(combined_root, adapter["parent_body"]).append(adapter_body)

    combined_equality = combined_root.find("equality")
    if combined_equality is None:
        combined_equality = ET.SubElement(combined_root, "equality")
    hand_equality = hand_root.find("equality")
    if hand_equality is None:
        raise RuntimeError(f"O6 mimic equalities are missing: {hand_path}")
    for equality in hand_equality:
        combined_equality.append(deepcopy(equality))


def _extend_keyframes(root: ET.Element, added_qpos: int) -> None:
    for key in root.findall("./keyframe/key"):
        qpos = key.get("qpos")
        if qpos:
            key.set("qpos", qpos + " " + " ".join("0" for _ in range(added_qpos)))


def _relative_frame(
    model: mujoco.MjModel, data: mujoco.MjData, parent: str, child: str
) -> tuple[np.ndarray, np.ndarray]:
    parent_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent)
    child_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, child)
    if parent_id < 0 or child_id < 0:
        raise RuntimeError(f"Missing frame in combined model: {parent}, {child}")
    parent_rotation = data.xmat[parent_id].reshape(3, 3)
    child_rotation = data.xmat[child_id].reshape(3, 3)
    relative_position = parent_rotation.T @ (data.xpos[child_id] - data.xpos[parent_id])
    relative_rotation = parent_rotation.T @ child_rotation
    return relative_position, relative_rotation


def build_g1_o6(
    project_root: Path, output: Path, scene: Path
) -> dict[str, Any]:
    """Build and validate the Phase 3 G1 plus dual-O6 model.

    Author: OpenAI Codex
    Date: 2026-07-16
    Purpose: Attach both official O6 models while removing rubber-hand dynamics.
    Why: Preserve the mandated G1 model without duplicate mass or mirrored hands.
    Inputs: Project root, output MJCF path, standalone scene path.
    Outputs: Phase 3 structural, mass, and frame validation metrics.
    Exceptions: Source/compile/validation errors with context.
    Coordinate Frame: G1 wrist-local adapter coordinates.
    Units: metres, kilograms, radians.
    Safety Notes: Kinematic/mass integration only; grasp/contact is unclaimed.
    Source: Locked Unitree and LinkerHand repositories.
    """
    project_root = project_root.resolve()
    g1_path = project_root / "third_party/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
    g1_reference = project_root / "third_party/unitree_ros/robots/g1_description/g1_29dof_with_hand.urdf"
    left_hand = project_root / "models/derived/hands/o6_left.xml"
    right_hand = project_root / "models/derived/hands/o6_right.xml"
    config_path = project_root / "configs/wrist_adapter.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    tree = ET.parse(g1_path)
    root = tree.getroot()
    root.set("model", "g1_29dof_dual_o6")
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("Official G1 MJCF compiler section is missing")
    g1_meshes = g1_path.parent / "meshes"
    compiler.attrib.pop("meshdir", None)
    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("Official G1 MJCF asset section is missing")
    for mesh in asset.findall("mesh"):
        filename = mesh.get("file")
        if not filename:
            raise RuntimeError(f"G1 mesh has no file: {mesh.get('name')}")
        source_mesh = (g1_meshes / filename).resolve()
        if not source_mesh.is_file():
            raise FileNotFoundError(f"Official G1 mesh is missing: {source_mesh}")
        mesh.set("file", Path(os.path.relpath(source_mesh, output.parent)).as_posix())

    urdf_root = ET.parse(g1_reference).getroot()
    replacement_masses: dict[str, float] = {}
    for side in ("left", "right"):
        wrist_name = f"{side}_wrist_yaw_link"
        replacement_masses[side] = _replace_wrist_inertial(
            _body(root, wrist_name), _urdf_link(urdf_root, wrist_name)
        )
        fixed_joint = _urdf_joint(urdf_root, f"{side}_hand_palm_joint")
        origin = fixed_joint.find("origin")
        if origin is None:
            raise RuntimeError(f"Unitree hand mount origin is missing for {side}")
        configured = np.asarray(config[side]["position_m"], dtype=float)
        official = np.asarray(
            [float(value) for value in origin.get("xyz", "").split()], dtype=float
        )
        if not np.allclose(configured, official, atol=1e-12):
            raise RuntimeError(f"Configured {side} adapter position differs from Unitree")
        _remove_rubber_hand(root, f"{side}_rubber_hand")

    output.parent.mkdir(parents=True, exist_ok=True)
    _merge_hand(root, left_hand, output, "left", config["left"])
    _merge_hand(root, right_hand, output, "right", config["right"])
    _extend_keyframes(root, added_qpos=22)
    root.insert(
        0,
        ET.Comment(
            " Derived from official G1 29DoF MJCF and independent official O6 "
            "hands. G1 rubber-hand geometry/inertia removed using Unitree URDF. "
        ),
    )
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)

    scene.parent.mkdir(parents=True, exist_ok=True)
    relative = Path(os.path.relpath(output, scene.parent)).as_posix()
    scene.write_text(
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<mujoco model=\"g1_dual_o6_standalone_test\">\n"
        f"  <include file=\"{relative}\"/>\n"
        "</mujoco>\n",
        encoding="utf-8",
    )

    official_model = mujoco.MjModel.from_xml_path(str(g1_path))
    model = mujoco.MjModel.from_xml_path(str(output))
    scene_model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    expected_mass = float(np.sum(official_model.body_mass))
    for side in ("left", "right"):
        wrist_id = mujoco.mj_name2id(
            official_model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wrist_yaw_link"
        )
        expected_mass -= float(official_model.body_mass[wrist_id])
        expected_mass += replacement_masses[side]
    for hand_path in (left_hand, right_hand):
        hand_model = mujoco.MjModel.from_xml_path(str(hand_path))
        expected_mass += float(np.sum(hand_model.body_mass))

    frame_results: dict[str, Any] = {}
    for side in ("left", "right"):
        adapter = config[side]
        position, rotation = _relative_frame(
            model, data, adapter["parent_body"], adapter["child_body"]
        )
        expected_rotation = np.column_stack(
            [
                adapter["axis_mapping"]["o6_x_in_g1_wrist"],
                adapter["axis_mapping"]["o6_y_in_g1_wrist"],
                adapter["axis_mapping"]["o6_z_in_g1_wrist"],
            ]
        )
        frame_results[side] = {
            "position_m": position.tolist(),
            "position_error_m": float(
                np.linalg.norm(position - np.asarray(adapter["position_m"], dtype=float))
            ),
            "rotation_matrix": rotation.tolist(),
            "rotation_max_abs_error": float(np.max(np.abs(rotation - expected_rotation))),
            "right_handed_determinant": float(np.linalg.det(rotation)),
        }

    contact_pairs = []
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
        geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
        contact_pairs.append([geom1 or str(contact.geom1), geom2 or str(contact.geom2)])

    actual_mass = float(np.sum(model.body_mass))
    result = {
        "load_success": True,
        "scene_load_success": scene_model.njnt == model.njnt,
        "model_counts": {
            "bodies_including_world": model.nbody,
            "joints_including_floating_base": model.njnt,
            "qpos": model.nq,
            "dofs": model.nv,
            "actuators": model.nu,
            "sensors": model.nsensor,
            "equalities": model.neq,
        },
        "mass": {
            "official_g1_with_rubber_hands_kg": float(np.sum(official_model.body_mass)),
            "removed_rubber_hand_mass_reference_kg_each": float(
                config["removed_g1_rubber_hand_mass_kg_each"]
            ),
            "replacement_wrist_yaw_mass_kg_each": replacement_masses,
            "expected_combined_kg": expected_mass,
            "actual_combined_kg": actual_mass,
            "absolute_error_kg": abs(actual_mass - expected_mass),
            "preserved_without_duplication": bool(
                np.isclose(actual_mass, expected_mass, rtol=1e-12, atol=1e-12)
            ),
        },
        "frames": frame_results,
        "contacts_at_initial_pose": contact_pairs,
        "rubber_hand_assets_removed": all(
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_MESH, f"{side}_rubber_hand"
            )
            < 0
            for side in ("left", "right")
        ),
        "hand_actuators_added": 0,
        "physical_adapter_hardware_status": (
            "Mount frames are verified; adapter plate geometry/mass requires measured CAD."
        ),
        "output": str(output),
        "scene": str(scene),
    }
    result["passed"] = bool(
        result["scene_load_success"]
        and result["model_counts"]["actuators"] == 29
        and result["model_counts"]["sensors"] == 95
        and result["model_counts"]["equalities"] == 10
        and result["mass"]["preserved_without_duplication"]
        and result["rubber_hand_assets_removed"]
        and all(
            frame["position_error_m"] < 1e-12
            and frame["rotation_max_abs_error"] < 1e-12
            and abs(frame["right_handed_determinant"] - 1.0) < 1e-12
            for frame in frame_results.values()
        )
    )
    if not result["passed"]:
        raise RuntimeError(f"G1+O6 combined model validation failed: {result}")
    return result
