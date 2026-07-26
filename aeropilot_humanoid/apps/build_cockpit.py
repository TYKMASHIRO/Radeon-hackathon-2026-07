"""Build, test, report, and render the passive Phase 4 cockpit."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.cockpit_builder import build_cockpit  # noqa: E402
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402


def _render(scene: Path, output: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(scene.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cockpit_overview")
    if camera_id < 0:
        raise RuntimeError("Configured cockpit overview camera is missing")
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.12, 0.0, 0.38)
    camera.distance = 1.65
    camera.azimuth = 135.0
    camera.elevation = -22.0
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(output)


def main() -> None:
    result = build_cockpit(PROJECT_ROOT)
    screenshot = PROJECT_ROOT / "reports/screenshots/cockpit_mechanisms.png"
    _render(Path(result["scene"]), screenshot)
    result["screenshot"] = str(screenshot.relative_to(PROJECT_ROOT))
    report = {"phase": 4, "status": "passed_pre_reachability_layout", **result}
    write_json(PROJECT_ROOT / "reports/cockpit_mechanism_report.json", report)
    (PROJECT_ROOT / "docs/cockpit_layout.md").write_text(
        "# Adjustable cockpit mechanism seed\n\n"
        "Phase 4 creates an independently testable seat, two-axis passive "
        "centering stick, non-centering linear throttle, opposed rudder pedals, "
        "independent toe brakes, heel supports, platform, and overview camera.\n\n"
        f"- Passive simulation: {result['test_duration_s']:.3f} s\n"
        f"- Joints / actuators / equalities: {result['counts']['joints']} / "
        f"{result['counts']['actuators']} / {result['counts']['equalities']}\n"
        f"- Behavior checks: `{json.dumps(result['behavior'], sort_keys=True)}`\n\n"
        "All layout values are search seeds/ranges, not a final reachability "
        "claim. Phase 5 must optimize them against G1+O6 joint margins and collisions.\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
