"""Generate a continuous humanoid pilot-control sequence from action keyframes.

The sequence is a deterministic kinematic playback: robot joints and cockpit
control joints follow smooth interpolated keyframes so the resulting MuJoCo
scene can be inspected, rendered, and regression-tested without a trained
contact policy.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from aeropilot_humanoid.pilot_action_builder import build_pilot_control_action
from aeropilot_humanoid.vendor_audit import write_json


KEYFRAME_NAMES = [
    "pilot_ready_on_controls",
    "left_hand_pushes_throttle_forward",
    "left_thumb_toggles_throttle_switch",
    "right_hand_deflects_control_stick",
]


def _joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Scene joint is missing: {name}")
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _site_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise RuntimeError(f"Scene site is missing: {name}")
    return data.site_xpos[site_id].copy()


def _body_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise RuntimeError(f"Scene body is missing: {name}")
    return data.xpos[body_id].copy()


def _keyframe_id(model: mujoco.MjModel, name: str) -> int:
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    if key_id < 0:
        raise RuntimeError(f"Scene keyframe is missing: {name}")
    return int(key_id)


def _smoothstep(value: float) -> float:
    clipped = float(np.clip(value, 0.0, 1.0))
    return clipped * clipped * (3.0 - 2.0 * clipped)


def _sample_keyframes(
    key_qpos: list[np.ndarray],
    frame_index: int,
    frame_count: int,
) -> np.ndarray:
    phase = frame_index / (frame_count - 1)
    segment_position = phase * (len(key_qpos) - 1)
    segment = min(int(segment_position), len(key_qpos) - 2)
    local = _smoothstep(segment_position - segment)
    return (1.0 - local) * key_qpos[segment] + local * key_qpos[segment + 1]


def simulate_pilot_control_sequence(
    project_root: Path,
    frame_count: int = 72,
    duration_s: float = 3.0,
) -> dict[str, Any]:
    """Build and validate a continuous humanoid control sequence."""
    if frame_count < 3:
        raise ValueError("frame_count must be at least 3")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")

    build_result = build_pilot_control_action(project_root)
    model_path = Path(build_result["output"])
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key_qpos = [
        model.key_qpos[_keyframe_id(model, name)].copy()
        for name in KEYFRAME_NAMES
    ]

    samples: list[dict[str, float]] = []
    max_left_error = 0.0
    max_right_error = 0.0
    previous_qpos: np.ndarray | None = None
    for frame_index in range(frame_count):
        qpos = _sample_keyframes(key_qpos, frame_index, frame_count)
        data.qpos[:] = qpos
        if previous_qpos is None:
            data.qvel[:] = 0.0
        else:
            data.qvel[:] = (qpos[: model.nv] - previous_qpos[: model.nv]) / (
                duration_s / (frame_count - 1)
            )
        previous_qpos = qpos
        mujoco.mj_forward(model, data)

        left_target = _site_position(model, data, "throttle_force_site") + np.array([-0.02, 0.0, 0.0])
        right_target = _site_position(model, data, "stick_force_site") + np.array([0.0, 0.0, -0.02])
        left_error = float(np.linalg.norm(_body_position(model, data, "lh_hand_base_link") - left_target))
        right_error = float(np.linalg.norm(_body_position(model, data, "rh_hand_base_link") - right_target))
        max_left_error = max(max_left_error, left_error)
        max_right_error = max(max_right_error, right_error)
        samples.append(
            {
                "time_s": frame_index * duration_s / (frame_count - 1),
                "throttle_m": _joint_qpos(model, data, "throttle_joint"),
                "stick_pitch_rad": _joint_qpos(model, data, "stick_pitch_joint"),
                "stick_roll_rad": _joint_qpos(model, data, "stick_roll_joint"),
                "left_hand_to_throttle_m": left_error,
                "right_hand_to_stick_m": right_error,
            }
        )

    first = samples[0]
    last = samples[-1]
    result = {
        "status": "passed_kinematic_control_sequence",
        "source_scene": build_result["scene"],
        "source_model": build_result["output"],
        "frame_count": frame_count,
        "duration_s": duration_s,
        "keyframes": KEYFRAME_NAMES,
        "start_controls": {
            "throttle_m": first["throttle_m"],
            "stick_pitch_rad": first["stick_pitch_rad"],
            "stick_roll_rad": first["stick_roll_rad"],
        },
        "end_controls": {
            "throttle_m": last["throttle_m"],
            "stick_pitch_rad": last["stick_pitch_rad"],
            "stick_roll_rad": last["stick_roll_rad"],
        },
        "control_motion": {
            "throttle_delta_m": last["throttle_m"] - first["throttle_m"],
            "stick_pitch_delta_rad": last["stick_pitch_rad"] - first["stick_pitch_rad"],
            "stick_roll_delta_rad": last["stick_roll_rad"] - first["stick_roll_rad"],
        },
        "max_tracking_error_m": {
            "left_hand_to_throttle": max_left_error,
            "right_hand_to_stick": max_right_error,
        },
        "validation_threshold_m": 0.04,
        "samples": samples,
        "control_scope": (
            "Kinematic robot-and-control playback generated from IK keyframes; "
            "it demonstrates the manipulation sequence without claiming learned "
            "force-closure grasping."
        ),
    }
    result["passed"] = bool(
        result["control_motion"]["throttle_delta_m"] > 0.10
        and result["control_motion"]["stick_pitch_delta_rad"] < -0.15
        and result["control_motion"]["stick_roll_delta_rad"] < -0.08
        and max(result["max_tracking_error_m"].values()) < result["validation_threshold_m"]
    )
    if not result["passed"]:
        raise RuntimeError(f"Pilot control sequence validation failed: {result}")

    write_json(project_root / "reports/pilot_control_sequence_report.json", result)
    return result
