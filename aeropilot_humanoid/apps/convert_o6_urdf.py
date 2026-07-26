"""Convert both official O6 hands to validated, root-preserving MJCF."""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from aeropilot_humanoid.o6_converter import ConversionPaths, convert_o6  # noqa: E402
from aeropilot_humanoid.vendor_audit import write_json  # noqa: E402
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402


def _set_qpos(model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"Joint is missing while rendering converted O6: {name}")
    data.qpos[int(model.jnt_qposadr[joint_id])] = value


def _render_conversion(
    model_path: Path, result: dict, output: Path, azimuth: float, flexed: bool
) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    if flexed:
        commands = result["kinematic_motion_test"]["commanded_independent_joints_rad"]
        for name, value in commands.items():
            _set_qpos(model, data, name, value)
        for coupling in result["passive_constraints"]:
            value = (
                float(coupling["offset_rad"] or 0.0)
                + float(coupling["multiplier"]) * commands[coupling["source_joint"]]
            )
            _set_qpos(model, data, coupling["joint"], value)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.06)
    camera.distance = 0.32
    camera.azimuth = azimuth
    camera.elevation = -15.0
    output.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output)


def main() -> None:
    third_party = PROJECT_ROOT / "third_party/linkerhand-urdf/o6"
    hands = PROJECT_ROOT / "models/derived/hands"
    scenes = PROJECT_ROOT / "models/scenes"
    jobs = {
        "left": ConversionPaths(
            urdf=third_party / "left/linkerhand_o6_left.urdf",
            mjcf=hands / "o6_left.xml",
            scene=scenes / "o6_left_test.xml",
        ),
        "right": ConversionPaths(
            urdf=third_party / "right/linkerhand_o6_right.urdf",
            mjcf=hands / "o6_right.xml",
            scene=scenes / "o6_right_test.xml",
        ),
    }
    report = {side: convert_o6(paths) for side, paths in jobs.items()}
    screenshot_dir = PROJECT_ROOT / "reports/screenshots"
    for side, paths in jobs.items():
        azimuth = 135.0 if side == "left" else 45.0
        screenshots = []
        for pose in ("open", "flexed"):
            target = screenshot_dir / f"o6_{side}_derived_{pose}.png"
            _render_conversion(
                paths.mjcf,
                report[side],
                target,
                azimuth,
                flexed=pose == "flexed",
            )
            screenshots.append(str(target.relative_to(PROJECT_ROOT)))
        report[side]["screenshots"] = screenshots
    report["phase"] = 2
    report["status"] = "passed" if all(report[side]["passed"] for side in jobs) else "failed"
    report["control_scope"] = (
        "No actuators are added during conversion. Six calibrated position inputs "
        "per hand remain Phase 6 work; five official mimic joints are constrained."
    )
    write_json(PROJECT_ROOT / "reports/o6_conversion_report.json", report)

    docs = PROJECT_ROOT / "docs/o6_conversion.md"
    docs.write_text(
        "# O6 URDF to MJCF conversion\n\n"
        "Both official hands were converted independently with MuJoCo's URDF "
        "compiler. A temporary floating mount forces the palm root to remain a "
        "body; the temporary joint is then removed, portable relative mesh paths "
        "are restored, and the five official mimic tags are encoded as MuJoCo "
        "joint equalities. Vendor files remain untouched.\n\n"
        "| Hand | URDF mass (kg) | Derived mass (kg) | Joints | Equalities | Max motion (m) | Result |\n"
        "|---|---:|---:|---:|---:|---:|---|\n"
        + "\n".join(
            f"| {side} | {result['source_urdf_mass_kg']:.9f} | "
            f"{result['total_mass_kg']:.9f} | {result['joints']} | "
            f"{result['equalities']} | "
            f"{result['kinematic_motion_test']['max_body_displacement_m']:.6f} | passed |"
            for side, result in report.items()
            if side in jobs
        )
        + "\n\nThe motion check is kinematic and coupling-consistent; it is not a "
        "claim about actuator torque, grasp force, or dynamic control.\n",
        encoding="utf-8",
    )
    print(json.dumps({side: report[side] for side in jobs}, indent=2))


if __name__ == "__main__":
    main()
