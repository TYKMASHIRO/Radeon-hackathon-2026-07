"""Regression tests for official vendor identity and structure."""

from __future__ import annotations

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

from aeropilot_humanoid.vendor_audit import inspect_g1, inspect_o6
from aeropilot_humanoid.o6_converter import validate_converted_o6
from aeropilot_humanoid.g1_o6_builder import build_g1_o6
from aeropilot_humanoid.cockpit_builder import build_cockpit
from aeropilot_humanoid.pilot_action_builder import build_pilot_control_action
from aeropilot_humanoid.pilot_control_sequence import simulate_pilot_control_sequence
from aeropilot_humanoid.hand_follow_controller import (
    HandFollowController,
    build_hand_follow_scene,
    simulate_hand_follow_sequence,
)
import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
G1_PATH = PROJECT_ROOT / "third_party/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
LEFT_PATH = PROJECT_ROOT / "third_party/linkerhand-urdf/o6/left/linkerhand_o6_left.urdf"
RIGHT_PATH = PROJECT_ROOT / "third_party/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf"
DERIVED_LEFT = PROJECT_ROOT / "models/derived/hands/o6_left.xml"
DERIVED_RIGHT = PROJECT_ROOT / "models/derived/hands/o6_right.xml"
COMBINED = PROJECT_ROOT / "models/derived/humanoid/g1_o6_full.xml"
COMBINED_SCENE = PROJECT_ROOT / "models/scenes/g1_o6_test.xml"


@pytest.fixture(scope="module")
def g1() -> dict:
    return inspect_g1(G1_PATH)


@pytest.fixture(scope="module")
def hands() -> tuple[dict, dict]:
    return inspect_o6(LEFT_PATH), inspect_o6(RIGHT_PATH)


def test_g1_is_official_29dof_tree(g1: dict) -> None:
    assert all(g1["validations"].values())
    assert g1["branch_counts"] == {
        "left_leg": 6,
        "right_leg": 6,
        "waist": 3,
        "left_arm": 7,
        "right_arm": 7,
    }
    assert g1["terminal_links"] == {
        "left_wrist": "left_wrist_yaw_link",
        "right_wrist": "right_wrist_yaw_link",
    }


def test_g1_has_expected_loaded_counts_and_mass(g1: dict) -> None:
    assert g1["model_counts"]["joints_including_floating_base"] == 30
    assert g1["model_counts"]["actuators"] == 29
    assert g1["model_counts"]["sensors"] == 95
    assert g1["total_mass_kg"] == pytest.approx(35.112142)


def test_o6_hands_are_independent_official_models(hands: tuple[dict, dict]) -> None:
    left, right = hands
    assert left["robot_name"] == "linkerhand_o6_left"
    assert right["robot_name"] == "linkerhand_o6_right"
    assert left["root_links"] == ["lh_hand_base_link"]
    assert right["root_links"] == ["rh_hand_base_link"]
    for hand in hands:
        assert hand["link_count"] == 12
        assert hand["joint_count"] == 11
        assert hand["independent_joint_count"] == 6
        assert hand["passive_mimic_joint_count"] == 5
        assert hand["missing_meshes"] == []
        assert hand["all_link_inertias_positive_definite"]
        assert hand["raw_mujoco_compile"]["arrays_finite"]
        assert hand["raw_mujoco_compile"]["actuators"] == 0


def test_o6_preserves_asymmetric_official_thumb_coupling(hands: tuple[dict, dict]) -> None:
    left, right = hands
    left_thumb = next(joint for joint in left["joints"] if joint["name"] == "lh_thumb_ip")
    right_thumb = next(joint for joint in right["joints"] if joint["name"] == "rh_thumb_ip")
    assert left_thumb["mimic_multiplier"] == pytest.approx(2.29)
    assert right_thumb["mimic_multiplier"] == pytest.approx(1.86)


def test_raw_o6_compile_exposes_root_mass_loss(hands: tuple[dict, dict]) -> None:
    for hand in hands:
        assert not hand["root_mass_preserved_by_raw_compile"]
        assert hand["raw_mujoco_compile"]["dynamic_mass_kg"] < hand["urdf_total_mass_kg"]


def test_derived_o6_models_restore_root_mass_and_mimic_constraints(
    hands: tuple[dict, dict],
) -> None:
    for model_path, audit in zip((DERIVED_LEFT, DERIVED_RIGHT), hands, strict=True):
        result = validate_converted_o6(model_path, audit)
        assert result["mass_preserved"]
        assert result["root_body_present"]
        assert result["all_mesh_paths_relative"]
        assert result["equalities"] == 5
        assert result["actuators"] == 0
        assert result["kinematic_motion_test"]["passed"]


def test_combined_model_preserves_mass_frames_and_g1_actuators() -> None:
    result = build_g1_o6(PROJECT_ROOT, COMBINED, COMBINED_SCENE)
    assert result["passed"]
    assert result["model_counts"]["actuators"] == 29
    assert result["model_counts"]["equalities"] == 10
    assert result["mass"]["preserved_without_duplication"]
    assert result["rubber_hand_assets_removed"]


def test_cockpit_mechanisms_are_passive_and_behave_independently() -> None:
    result = build_cockpit(PROJECT_ROOT)
    assert result["passed"]
    assert result["counts"]["joints"] == 7
    assert result["counts"]["actuators"] == 0
    assert result["counts"]["equalities"] == 1
    assert all(result["behavior"].values())


def test_pilot_control_action_scene_has_valid_keyframes() -> None:
    result = build_pilot_control_action(PROJECT_ROOT)
    assert result["passed"]
    assert result["scene_load_success"]
    assert result["local_control_fbx"]["exists"]
    assert result["converted_control_xml"]["imported_into_action_scene"]
    assert result["counts"]["keyframes"] == 4
    assert result["max_reach_error_m"] < result["validation_threshold_m"]
    root = ET.parse(result["output"]).getroot()
    body_names = {body.get("name") for body in root.findall(".//body")}
    site_names = {site.get("name") for site in root.findall(".//site")}
    assert "fbx_throttle_handle_body" in body_names
    assert "fbx_stick_roll_frame" in body_names
    assert "throttle_thumb_switch_site" in site_names
    assert "throttle_index_switch_site" in site_names
    assert "control_stick_base" not in body_names
    assert "throttle_base" not in body_names


def test_fbx_control_meshes_align_with_their_mujoco_joint_frames() -> None:
    imported = PROJECT_ROOT / "models/derived/cockpit/imported_3d66"
    root = ET.parse(imported / "fbx_controls.xml").getroot()
    control_body = root.find(
        ".//body[@name='imported_3d66_joystick_throttle']"
    )
    roll_body = root.find(".//body[@name='fbx_stick_roll_frame']")
    pitch_body = root.find(".//body[@name='fbx_stick_pitch_frame']")
    throttle_body = root.find(".//body[@name='fbx_throttle_handle_body']")
    assert all(
        body is not None
        for body in (control_body, roll_body, pitch_body, throttle_body)
    )

    root_geom_names = {
        geom.get("name") for geom in control_body.findall("./geom")
    }
    assert "fbx_controls_static_visual" in root_geom_names
    assert "fbx_stick_static_visual" in root_geom_names
    assert roll_body.find("./geom[@name='fbx_stick_static_visual']") is None
    assert pitch_body.find("./geom[@name='fbx_stick_visual']") is not None
    assert throttle_body.find("./geom[@name='fbx_throttle_visual']") is not None

    def obj_bounds(filename: str) -> tuple[np.ndarray, np.ndarray]:
        vertices = [
            [float(value) for value in line.split()[1:4]]
            for line in (imported / "meshes" / filename)
            .read_text(encoding="utf-8")
            .splitlines()
            if line.startswith("v ")
        ]
        points = np.asarray(vertices)
        return points.min(axis=0), points.max(axis=0)

    throttle_min, throttle_max = obj_bounds("3d66_throttle_moving.obj")
    stick_min, stick_max = obj_bounds("3d66_stick_moving.obj")
    throttle_static_min, throttle_static_max = obj_bounds(
        "3d66_controls_static.obj"
    )
    stick_static_min, stick_static_max = obj_bounds("3d66_stick_static.obj")
    throttle_site = np.fromstring(
        throttle_body.find("./site[@name='throttle_force_site']").get("pos"),
        sep=" ",
    )
    stick_site = np.fromstring(
        pitch_body.find("./site[@name='stick_force_site']").get("pos"),
        sep=" ",
    )

    assert np.all(throttle_site >= throttle_min)
    assert np.all(throttle_site <= throttle_max)
    assert np.all(stick_site >= stick_min)
    assert np.all(stick_site <= stick_max)
    assert stick_site[2] > stick_min[2] + 0.8 * (
        stick_max[2] - stick_min[2]
    )
    assert np.mean([throttle_static_min[1], throttle_static_max[1]]) > 0.35
    assert abs(np.mean([stick_static_min[1], stick_static_max[1]])) < 0.01
    assert (
        pitch_body.find("./geom[@name='fbx_stick_visual']").get("contype")
        == "0"
    )
    assert (
        throttle_body.find("./geom[@name='fbx_throttle_visual']").get(
            "contype"
        )
        == "0"
    )
    model = mujoco.MjModel.from_xml_path(str(imported / "fbx_controls.xml"))
    expected_minimum_faces = {
        "fbx_controls_static_mesh": 450,
        "fbx_throttle_moving_mesh": 2800,
        "fbx_stick_static_mesh": 650,
        "fbx_stick_moving_mesh": 2600,
    }
    for mesh_name, minimum_faces in expected_minimum_faces.items():
        assert model.mesh_facenum[model.mesh(mesh_name).id] >= minimum_faces


def test_pilot_control_sequence_moves_throttle_and_stick() -> None:
    result = simulate_pilot_control_sequence(PROJECT_ROOT, frame_count=12, duration_s=1.0)
    assert result["passed"]
    assert result["control_motion"]["throttle_delta_m"] > 0.10
    assert result["control_motion"]["stick_pitch_delta_rad"] < -0.15
    assert result["control_motion"]["stick_roll_delta_rad"] < -0.08
    assert max(result["max_tracking_error_m"].values()) < result["validation_threshold_m"]


def test_hand_follow_scene_maps_hand_space_to_control_axes() -> None:
    result = build_hand_follow_scene(PROJECT_ROOT)
    assert result["passed"]
    assert result["actuators"] == 32

    model = mujoco.MjModel.from_xml_path(result["scene"])
    data = mujoco.MjData(model)
    ready_key = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        "pilot_ready_on_controls",
    )
    data.qpos[:] = model.key_qpos[ready_key]
    mujoco.mj_forward(model, data)
    controller = HandFollowController(model, data)

    right_hand = data.body("rh_hand_base_link").xpos.copy()
    left_hand = data.body("lh_hand_base_link").xpos.copy()
    neutral = controller.command(right_hand, left_hand)
    moved = controller.command(
        right_hand + np.array([0.04, 0.04, 0.0]),
        left_hand + np.array([0.10, 0.05, -0.04]),
    )
    axial_only = controller.command(
        right_hand,
        left_hand + np.array([0.10, 0.0, 0.0]),
    )

    assert neutral.stick_roll_rad == pytest.approx(0.0, abs=1e-9)
    assert neutral.stick_pitch_rad == pytest.approx(0.0, abs=1e-9)
    assert moved.stick_roll_rad < -0.10
    assert moved.stick_pitch_rad > 0.10
    assert moved.throttle_m == pytest.approx(axial_only.throttle_m)
    assert moved.throttle_m - neutral.throttle_m == pytest.approx(0.10)
    controller.apply(moved)
    assert data.actuator("stick_roll_hand_follow").ctrl[0] == pytest.approx(
        moved.stick_roll_rad
    )
    assert data.actuator("stick_pitch_hand_follow").ctrl[0] == pytest.approx(
        moved.stick_pitch_rad
    )
    assert data.actuator("throttle_hand_follow").ctrl[0] == pytest.approx(
        moved.throttle_m
    )


def test_hand_follow_servos_track_the_existing_robot_hand_motion() -> None:
    result = simulate_hand_follow_sequence(PROJECT_ROOT, duration_s=3.0)
    assert result["passed"]
    assert result["throttle_cross_axis_error_m"] < 1e-12
    assert max(result["final_abs_error"].values()) < 0.02


@pytest.mark.parametrize(
    "repository",
    ["unitree_mujoco", "unitree_ros", "linkerhand-urdf", "linkerhand-ros2-sdk"],
)
def test_vendor_worktree_is_unmodified(repository: str) -> None:
    path = PROJECT_ROOT / "third_party" / repository
    if not (path / ".git").exists():
        pytest.skip("Vendor Git metadata is not present in this distribution")
    result = subprocess.run(
        ["git", "-C", str(path), "status", "--short"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    assert result.stdout == ""
