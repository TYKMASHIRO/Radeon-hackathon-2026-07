"""Render the MuJoCo hand-following control sequence to PNG and GIF."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.hand_follow_controller import (  # noqa: E402
    HandFollowController,
    build_hand_follow_scene,
)
from aeropilot_humanoid.pilot_control_sequence import (  # noqa: E402
    KEYFRAME_NAMES,
    _keyframe_id,
    _sample_keyframes,
)
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402


def main() -> None:
    started = time.perf_counter()
    build = build_hand_follow_scene(PROJECT_ROOT)
    model = mujoco.MjModel.from_xml_path(build["scene"])
    data = mujoco.MjData(model)
    key_qpos = [
        model.key_qpos[_keyframe_id(model, name)].copy()
        for name in KEYFRAME_NAMES
    ]
    data.qpos[:] = key_qpos[0]
    mujoco.mj_forward(model, data)
    controller = HandFollowController(model, data)

    joint_names = (
        "stick_roll_joint",
        "stick_pitch_joint",
        "throttle_joint",
    )
    qpos_addresses = np.array(
        [int(model.jnt_qposadr[model.joint(name).id]) for name in joint_names]
    )
    dof_addresses = np.array(
        [int(model.jnt_dofadr[model.joint(name).id]) for name in joint_names]
    )

    model.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
    model.vis.headlight.diffuse[:] = (0.85, 0.85, 0.85)
    model.vis.headlight.specular[:] = (0.25, 0.25, 0.25)
    control_colors = {
        "fbx_controls_static_visual": (0.32, 0.34, 0.37, 1.0),
        "fbx_stick_static_visual": (0.20, 0.24, 0.30, 1.0),
        "fbx_stick_visual": (0.05, 0.35, 0.75, 1.0),
        "fbx_throttle_visual": (0.85, 0.28, 0.05, 1.0),
    }
    for geom_name, rgba in control_colors.items():
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom_name,
        )
        if geom_id >= 0:
            model.geom_rgba[geom_id] = rgba
    for site_name, rgba in (
        ("stick_force_site", (0.15, 0.65, 1.0, 1.0)),
        ("throttle_force_site", (1.0, 0.45, 0.05, 1.0)),
    ):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id >= 0:
            model.site_rgba[site_id] = rgba
            model.site_size[site_id, 0] = 0.018
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.24, 0.10, 0.78)
    camera.distance = 1.12
    camera.azimuth = 135.0
    camera.elevation = -18.0

    duration_s = 3.0
    rendered_frames = 60
    simulation_steps = int(round(duration_s / model.opt.timestep)) + 1
    render_steps = set(
        np.linspace(0, simulation_steps - 1, rendered_frames, dtype=int)
    )
    snapshot_steps = {
        0: "00_ready.png",
        int(round((simulation_steps - 1) / 3)): "01_throttle_forward.png",
        simulation_steps - 1: "02_stick_deflected.png",
    }
    output_dir = PROJECT_ROOT / "reports/screenshots/hand_follow_controls"
    output_dir.mkdir(parents=True, exist_ok=True)

    images: list[Image.Image] = []
    snapshots: list[str] = []
    final_command = controller.command()
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for step_index in range(simulation_steps):
            robot_target = _sample_keyframes(
                key_qpos,
                step_index,
                simulation_steps,
            )
            control_qpos = data.qpos[qpos_addresses].copy()
            control_qvel = data.qvel[dof_addresses].copy()
            data.qpos[:] = robot_target
            data.qpos[qpos_addresses] = control_qpos
            data.qvel[:] = 0.0
            data.qvel[dof_addresses] = control_qvel
            mujoco.mj_forward(model, data)
            final_command = controller.update()
            mujoco.mj_step(model, data)

            if step_index in render_steps or step_index in snapshot_steps:
                renderer.update_scene(data, camera=camera)
                image = Image.fromarray(renderer.render())
                if step_index in render_steps:
                    images.append(image.copy())
                if step_index in snapshot_steps:
                    path = output_dir / snapshot_steps[step_index]
                    image.save(path)
                    snapshots.append(str(path.relative_to(PROJECT_ROOT)))

    gif_path = output_dir / "hand_follow_controls.gif"
    images[0].save(
        gif_path,
        save_all=True,
        append_images=images[1:],
        duration=50,
        loop=0,
    )

    controls_model_path = (
        PROJECT_ROOT
        / "models/derived/cockpit/imported_3d66/fbx_controls.xml"
    )
    controls_model = mujoco.MjModel.from_xml_path(str(controls_model_path))
    controls_data = mujoco.MjData(controls_model)
    for joint_name, value in (
        ("stick_roll_joint", 0.0),
        ("stick_pitch_joint", 0.0),
        ("throttle_joint", 0.0),
    ):
        joint_id = controls_model.joint(joint_name).id
        controls_data.qpos[int(controls_model.jnt_qposadr[joint_id])] = value
    controls_model.vis.headlight.ambient[:] = (0.45, 0.45, 0.45)
    controls_model.vis.headlight.diffuse[:] = (0.85, 0.85, 0.85)
    for geom_name, rgba in control_colors.items():
        geom_id = mujoco.mj_name2id(
            controls_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom_name,
        )
        if geom_id >= 0:
            controls_model.geom_rgba[geom_id] = rgba
    controls_model.site_rgba[:, 3] = 0.0
    for helper_geom_name in (
        "fbx_stick_roll_hub",
        "throttle_thumb_switch",
        "throttle_index_switch",
    ):
        helper_geom_id = mujoco.mj_name2id(
            controls_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            helper_geom_name,
        )
        if helper_geom_id >= 0:
            controls_model.geom_rgba[helper_geom_id, 3] = 0.0
    controls_camera = mujoco.MjvCamera()
    controls_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    controls_camera.lookat[:] = (0.20, 0.18, 0.72)
    controls_camera.distance = 0.78
    controls_camera.azimuth = 45.0
    controls_camera.elevation = -20.0
    mujoco.mj_forward(controls_model, controls_data)
    overview_path = output_dir / "03_fbx_controls_overview.png"
    with mujoco.Renderer(
        controls_model,
        height=480,
        width=640,
    ) as controls_renderer:
        controls_renderer.update_scene(
            controls_data,
            camera=controls_camera,
        )
        Image.fromarray(controls_renderer.render()).save(overview_path)
    snapshots.append(str(overview_path.relative_to(PROJECT_ROOT)))

    elapsed_s = time.perf_counter() - started
    final_controls = data.qpos[qpos_addresses]
    final_targets = np.array(
        [
            final_command.stick_roll_rad,
            final_command.stick_pitch_rad,
            final_command.throttle_m,
        ]
    )
    report = {
        "status": "passed_mujoco_render",
        "scene": build["scene"],
        "fbx_source_model": str(controls_model_path),
        "output_dir": str(output_dir),
        "gif": str(gif_path.relative_to(PROJECT_ROOT)),
        "snapshots": snapshots,
        "simulation_steps": simulation_steps,
        "rendered_frames": len(images),
        "processed_png": len(snapshots),
        "gif_outputs": 1,
        "width_px": 640,
        "height_px": 480,
        "internal_elapsed_ms": elapsed_s * 1000.0,
        "avg_ms_per_rendered_frame": elapsed_s * 1000.0 / len(images),
        "render_fps": len(images) / elapsed_s,
        "final_targets": {
            name: float(value)
            for name, value in zip(joint_names, final_targets, strict=True)
        },
        "final_controls": {
            name: float(value)
            for name, value in zip(joint_names, final_controls, strict=True)
        },
        "final_abs_error": {
            name: float(abs(actual - target))
            for name, actual, target in zip(
                joint_names,
                final_controls,
                final_targets,
                strict=True,
            )
        },
        "valid": len(images),
        "skipped": 0,
        "failed": 0,
        "warnings": 0,
    }
    write_json(PROJECT_ROOT / "reports/hand_follow_render_report.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
