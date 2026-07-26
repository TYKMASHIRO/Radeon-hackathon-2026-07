"""Build, report, and render the humanoid pilot-control action scene."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.pilot_action_builder import build_pilot_control_action  # noqa: E402
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402


def _render_keyframes(model_path: Path, output_dir: Path) -> list[str]:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.10, 0.0, 0.65)
    camera.distance = 1.80
    camera.azimuth = 115.0
    camera.elevation = -16.0
    output_dir.mkdir(parents=True, exist_ok=True)
    screenshots: list[str] = []
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for key_id in range(model.nkey):
            key_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
            if not key_name:
                key_name = f"key_{key_id}"
            data.qpos[:] = model.key_qpos[key_id]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            path = output_dir / f"{key_id + 1:02d}_{key_name}.png"
            Image.fromarray(renderer.render()).save(path)
            screenshots.append(str(path.relative_to(PROJECT_ROOT)))
    return screenshots


def main() -> None:
    result = build_pilot_control_action(PROJECT_ROOT)
    screenshots = _render_keyframes(
        Path(result["output"]),
        PROJECT_ROOT / "reports/screenshots/pilot_action",
    )
    result["screenshots"] = screenshots
    report = {"phase": 5, "status": "passed_geometric_action_storyboard", **result}
    write_json(PROJECT_ROOT / "reports/pilot_control_action_report.json", report)

    (PROJECT_ROOT / "docs/pilot_control_action.md").write_text(
        "# Humanoid pilot-control action scene\n\n"
        "This scene combines the sourced Unitree G1 humanoid, dual LinkerHand O6 "
        "hands, and the passive cockpit stick/throttle mechanisms into a "
        "keyframed control-action storyboard.\n\n"
        f"- Scene: `{Path(result['scene']).relative_to(PROJECT_ROOT)}`\n"
        f"- Model: `{Path(result['output']).relative_to(PROJECT_ROOT)}`\n"
        f"- Local FBX reference: `{Path(result['local_control_fbx']['path']).name}`\n"
        f"- Keyframes: `{', '.join(result['keyframes'])}`\n"
        f"- Maximum hand-to-control target error: {result['max_reach_error_m']:.4f} m\n"
        f"- Validation threshold: {result['validation_threshold_m']:.4f} m\n"
        f"- Screenshots: `{', '.join(screenshots)}`\n\n"
        "The FBX file is converted into split OBJ meshes and imported as "
        "articulated MuJoCo bodies. The generated keyframes drive the converted "
        "model's throttle and stick joints directly. The result proves a "
        "centimetre-scale geometric action pose, not a trained contact-rich "
        "grasping controller.\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
