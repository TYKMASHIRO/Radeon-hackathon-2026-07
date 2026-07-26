"""Build, report, and render the Phase 3 G1 plus dual-O6 model."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.g1_o6_builder import build_g1_o6  # noqa: E402
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402


def _render(
    model_path: Path,
    output: Path,
    lookat: tuple[float, float, float],
    distance: float,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = 135.0
    camera.elevation = -12.0
    output.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output)


def main() -> None:
    output = PROJECT_ROOT / "models/derived/humanoid/g1_o6_full.xml"
    scene = PROJECT_ROOT / "models/scenes/g1_o6_test.xml"
    result = build_g1_o6(PROJECT_ROOT, output, scene)
    screenshot = PROJECT_ROOT / "reports/screenshots/g1_o6_combined.png"
    wrist_screenshot = PROJECT_ROOT / "reports/screenshots/g1_o6_wrists.png"
    _render(output, screenshot, (0.0, 0.0, 0.78), 2.0)
    model = mujoco.MjModel.from_xml_path(str(output.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hand_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("lh_hand_base_link", "rh_hand_base_link")
    ]
    midpoint = tuple(float(value) for value in data.xpos[hand_ids].mean(axis=0))
    _render(output, wrist_screenshot, midpoint, 0.65)
    result["screenshots"] = [
        str(screenshot.relative_to(PROJECT_ROOT)),
        str(wrist_screenshot.relative_to(PROJECT_ROOT)),
    ]
    report = {"phase": 3, "status": "passed_with_adapter_cad_pending", **result}
    write_json(PROJECT_ROOT / "reports/g1_o6_integration_report.json", report)
    (PROJECT_ROOT / "docs/g1_o6_integration.md").write_text(
        "# G1 and dual O6 integration\n\n"
        "The official G1 29DoF MJCF remains the kinematic source. Unitree's "
        "official URDF separates each 0.170 kg removable rubber hand from its "
        "0.08457647 kg wrist-yaw link; the derived model removes the rubber-hand "
        "mesh and restores that bare-wrist inertia before attaching O6.\n\n"
        "The official G1 hand model extends fingers along +X and distributes "
        "finger roots along ±Z. O6 extends fingers along +Z and distributes them "
        "along ±Y. Matching official index/middle roots and applying the "
        "right-hand rule yields the side-specific matrices recorded in "
        "`configs/wrist_adapter.yaml`.\n\n"
        f"- Combined mass: {result['mass']['actual_combined_kg']:.12f} kg\n"
        f"- Joints / actuators / equalities: {result['model_counts']['joints_including_floating_base']} / "
        f"{result['model_counts']['actuators']} / {result['model_counts']['equalities']}\n"
        f"- Initial contacts reported by MuJoCo: {len(result['contacts_at_initial_pose'])}\n"
        "- O6 actuator count added: 0\n"
        "- Remaining hardware item: measured adapter-plate CAD, mass, and inertia.\n\n"
        "This phase proves source-correct kinematics, frames, and mass accounting. "
        "It does not yet prove grasping, cockpit reachability, or adapter hardware stiffness.\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
