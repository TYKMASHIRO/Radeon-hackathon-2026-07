"""Render the humanoid pilot manipulating throttle and stick as a GIF."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.pilot_control_sequence import (  # noqa: E402
    KEYFRAME_NAMES,
    _keyframe_id,
    _sample_keyframes,
    simulate_pilot_control_sequence,
)
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402


def _render_sequence(model_path: Path, output_dir: Path, frame_count: int) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    key_qpos = [
        model.key_qpos[_keyframe_id(model, name)].copy()
        for name in KEYFRAME_NAMES
    ]
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.10, 0.0, 0.65)
    camera.distance = 1.80
    camera.azimuth = 115.0
    camera.elevation = -16.0

    output_dir.mkdir(parents=True, exist_ok=True)
    frame_paths: list[str] = []
    frames: list[Image.Image] = []
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for frame_index in range(frame_count):
            data.qpos[:] = _sample_keyframes(key_qpos, frame_index, frame_count)
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render())
            if frame_index % 6 == 0 or frame_index == frame_count - 1:
                path = output_dir / f"pilot_control_{frame_index:03d}.png"
                image.save(path)
                frame_paths.append(str(path.relative_to(PROJECT_ROOT)))
            frames.append(image)

    gif_path = output_dir / "pilot_control_sequence.gif"
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=50,
        loop=0,
    )
    return {
        "gif": str(gif_path.relative_to(PROJECT_ROOT)),
        "sampled_frames": frame_paths,
    }


def main() -> None:
    frame_count = 72
    duration_s = 3.0
    result = simulate_pilot_control_sequence(
        PROJECT_ROOT,
        frame_count=frame_count,
        duration_s=duration_s,
    )
    render_result = _render_sequence(
        Path(result["source_model"]),
        PROJECT_ROOT / "reports/screenshots/pilot_control_sequence",
        frame_count,
    )
    result["render"] = render_result
    write_json(PROJECT_ROOT / "reports/pilot_control_sequence_report.json", result)
    (PROJECT_ROOT / "docs/pilot_control_sequence.md").write_text(
        "# Humanoid pilot control sequence\n\n"
        "This runnable sequence animates the G1+O6 humanoid through three "
        "control states: ready on controls, left hand pushing the throttle "
        "forward, and right hand deflecting the control stick.\n\n"
        f"- Source model: `{Path(result['source_model']).relative_to(PROJECT_ROOT)}`\n"
        f"- Duration: {duration_s:.2f} s\n"
        f"- Frames: {frame_count}\n"
        f"- GIF: `{render_result['gif']}`\n"
        f"- Throttle travel: {result['control_motion']['throttle_delta_m']:.3f} m\n"
        f"- Stick pitch delta: {result['control_motion']['stick_pitch_delta_rad']:.3f} rad\n"
        f"- Stick roll delta: {result['control_motion']['stick_roll_delta_rad']:.3f} rad\n"
        f"- Max hand/control tracking error: "
        f"{max(result['max_tracking_error_m'].values()):.4f} m\n\n"
        "The sequence is deterministic kinematic playback from IK keyframes. "
        "It is suitable for visualization and integration tests; forceful "
        "contact-control policy training remains a separate task.\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
