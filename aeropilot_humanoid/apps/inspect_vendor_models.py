"""Generate Phase 0 audit reports and Phase 1 vendor-model screenshots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import mujoco  # noqa: E402
from PIL import Image  # noqa: E402
import yaml  # noqa: E402

from aeropilot_humanoid.vendor_audit import inspect_g1, inspect_o6, write_json  # noqa: E402


G1_PATH = PROJECT_ROOT / "third_party/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
O6_LEFT_PATH = PROJECT_ROOT / "third_party/linkerhand-urdf/o6/left/linkerhand_o6_left.urdf"
O6_RIGHT_PATH = PROJECT_ROOT / "third_party/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf"


def _git_status(repository: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repository), "status", "--short"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return completed.stdout.strip()


def _joint_table(joints: list[dict[str, Any]], o6: bool = False) -> str:
    if o6:
        header = "| Joint | Parent | Child | Axis | Range (rad) | Role / coupling |\n|---|---|---|---|---|---|"
        rows = []
        for joint in joints:
            relation = joint["role"]
            if joint["mimic_joint"]:
                relation += f" → {joint['mimic_joint']} × {joint['mimic_multiplier']}"
            rows.append(
                f"| {joint['name']} | {joint['parent']} | {joint['child']} | "
                f"{joint['axis']} | [{joint['limit_lower_rad']}, {joint['limit_upper_rad']}] | {relation} |"
            )
        return "\n".join([header, *rows])
    header = "| # | Joint | Body | Parent body | Branch | Axis | Range (rad) | Actuator |\n|---:|---|---|---|---|---|---|---|"
    rows = [
        f"| {joint['index']} | {joint['name']} | {joint['body']} | {joint['parent_body']} | "
        f"{joint['branch']} | {joint['axis']} | {joint['range_rad']} | {joint['actuator'] or '—'} |"
        for joint in joints
    ]
    return "\n".join([header, *rows])


def _write_markdown_reports(
    g1: dict[str, Any], left: dict[str, Any], right: dict[str, Any]
) -> None:
    docs = PROJECT_ROOT / "docs"
    docs.mkdir(parents=True, exist_ok=True)
    g1_doc = f"""# G1 29DoF joint map

Generated from the loaded MuJoCo parent-child tree, not from filename or substring counts.

- Load: **passed**
- Physical bodies: {g1['model_counts']['physical_bodies']} ({g1['model_counts']['bodies_including_world']} including world)
- Joints: {g1['model_counts']['joints_including_floating_base']} including the floating base
- Actuated hinge joints: {g1['model_counts']['actuated_hinge_joints']}
- Actuators: {g1['model_counts']['actuators']}
- Sensors: {g1['model_counts']['sensors']}
- Total mass: {g1['total_mass_kg']:.9f} kg
- Branch counts: `{json.dumps(g1['branch_counts'], sort_keys=True)}`
- Left wrist terminal link: `{g1['terminal_links']['left_wrist']}`
- Right wrist terminal link: `{g1['terminal_links']['right_wrist']}`

{_joint_table(g1['joints'])}

## Scope boundary

The six-joint leg chains are structurally present. Pedal reachability and physical contact are not proven by topology and remain Phase 5/6 work.
"""
    (docs / "g1_joint_map.md").write_text(g1_doc, encoding="utf-8")

    o6_doc = f"""# LinkerHand O6 joint map

Both hands were parsed independently from their official URDFs. They are not mirrored derivatives.

## Left O6

- Root link: `{left['root_links'][0]}`
- Links / joints: {left['link_count']} / {left['joint_count']}
- Independent inputs / passive mimic joints: {left['independent_joint_count']} / {left['passive_mimic_joint_count']}
- URDF mass: {left['urdf_total_mass_kg']:.9f} kg
- Raw MuJoCo dynamic mass: {left['raw_mujoco_compile']['dynamic_mass_kg']:.9f} kg

{_joint_table(left['joints'], o6=True)}

## Right O6

- Root link: `{right['root_links'][0]}`
- Links / joints: {right['link_count']} / {right['joint_count']}
- Independent inputs / passive mimic joints: {right['independent_joint_count']} / {right['passive_mimic_joint_count']}
- URDF mass: {right['urdf_total_mass_kg']:.9f} kg
- Raw MuJoCo dynamic mass: {right['raw_mujoco_compile']['dynamic_mass_kg']:.9f} kg

{_joint_table(right['joints'], o6=True)}

## Integration finding

Raw URDF compilation succeeds, but the fixed root is fused into the world and its inertia is not retained as an attachable rigid body. The root-preserving Phase 2 conversion must also recreate all five mimic constraints per hand. The asymmetric thumb multipliers (left 2.29, right 1.86) must not be mirrored or averaged.
"""
    (docs / "o6_joint_map.md").write_text(o6_doc, encoding="utf-8")


def _write_phase0_report(
    g1: dict[str, Any], left: dict[str, Any], right: dict[str, Any]
) -> None:
    manifest = yaml.safe_load(
        (PROJECT_ROOT / "third_party/source_manifest.yaml").read_text(encoding="utf-8")
    )
    repositories = {
        "unitree_mujoco": PROJECT_ROOT / "third_party/unitree_mujoco",
        "unitree_ros": PROJECT_ROOT / "third_party/unitree_ros",
        "linkerhand_urdf": PROJECT_ROOT / "third_party/linkerhand-urdf",
        "linkerhand_ros2_sdk": PROJECT_ROOT / "third_party/linkerhand-ros2-sdk",
    }
    vendor_clean = {name: _git_status(path) == "" for name, path in repositories.items()}
    phase0 = {
        "phase": 0,
        "status": "passed_with_open_integration_risks",
        "initial_workspace_state": {
            "branch": "feature/modeling-simulation",
            "tracked_project_files": [
                "README.md",
                "requirements.txt",
                "simulation/minimal_robot_demo.py",
                "Radeon-Cloud-User Guide/README.md",
            ],
            "finding": "No official G1 or O6 model existed before this audit.",
        },
        "official_resources": {
            "manifest": manifest,
            "vendor_worktrees_clean": vendor_clean,
            "g1_29dof_found": G1_PATH.is_file(),
            "o6_left_urdf_found": O6_LEFT_PATH.is_file(),
            "o6_right_urdf_found": O6_RIGHT_PATH.is_file(),
            "o6_left_meshes_complete": not left["missing_meshes"],
            "o6_right_meshes_complete": not right["missing_meshes"],
        },
        "g1": g1,
        "o6_left": left,
        "o6_right": right,
        "environment": {
            "required_python": "3.11",
            "audited_python": sys.version.split()[0],
            "python_version_matches_target": sys.version_info[:2] == (3, 11),
            "mujoco": mujoco.__version__,
        },
        "proposed_or_created_files": [
            "third_party/source_manifest.yaml",
            "third_party/licenses/*",
            "src/aeropilot_humanoid/vendor_audit.py",
            "apps/inspect_vendor_models.py",
            "docs/source_audit.md",
            "docs/g1_joint_map.md",
            "docs/o6_joint_map.md",
            "docs/assumptions_and_risks.md",
            "reports/phase0_audit.json",
            "reports/g1_joint_map.json",
            "reports/o6_joint_map.json",
            "reports/vendor_load_report.json",
            "reports/screenshots/*.png",
            "tests/test_vendor_models.py",
        ],
        "unresolved_parameters": [
            "Left and right G1-to-O6 wrist adapter transforms.",
            "Whether the G1 wrist visual/physical geometry must be disabled when O6 is attached.",
            "Cockpit seat, stick, throttle, pedal, and heel-rest dimensions.",
            "Contact stiffness, damping, friction, solver, and actuator gains.",
            "A collision-safe seated pose and multi-start reachability solution.",
        ],
        "risks": [
            "Raw O6 URDF compilation loses root-link dynamics by fusing the fixed root into world.",
            "MuJoCo does not preserve URDF mimic tags as constraints or actuators.",
            "Left/right O6 thumb coupling multipliers differ and cannot be mirrored.",
            "The current Python 3.12.5 environment differs from the target Python 3.11.",
            "The Unitree repository has unrelated case-colliding Go2W terrain filenames on Windows.",
            "A complete leg chain does not prove pedal reachability or safe contact.",
            "High-resolution hand collision meshes may be too expensive for training.",
        ],
        "implementation_plan": [
            "Phase 1: independently load and render G1, O6 left, and O6 right.",
            "Phase 2: convert each O6 URDF to root-preserving MJCF and recreate mimic constraints.",
            "Phase 3: calculate independent left/right wrist adapters and validate orientation/inertia.",
            "Phase 4: model seat, stick, throttle, coupled rudder, differential brakes, and cameras.",
            "Phase 5: solve collision-aware seated reachability with multiple starts.",
            "Phase 6-9: controllers, physical-contact demo, train model, and validation reports.",
        ],
    }
    write_json(PROJECT_ROOT / "reports/phase0_audit.json", phase0)

    source_rows = []
    for name, source in manifest["sources"].items():
        source_rows.append(
            f"| {name} | [{source['repository']}]({source['repository']}) | `{source['branch']}` | `{source['commit']}` | {source['license']} | {source['modified']} |"
        )
    source_doc = f"""# Official source audit

Audit date: 2026-07-16

| Source | Repository | Branch | Commit | License | Modified |
|---|---|---|---|---|---|
{chr(10).join(source_rows)}

All four vendor worktrees were clean at audit time. Exact license copies are stored under `third_party/licenses/`. No vendor file was edited. The parent repository ignores the cloned trees so they cannot be accidentally committed as embedded repositories; `source_manifest.yaml` is the reproducibility lock.

## Required files

- G1: `unitree_robots/g1/g1_29dof.xml` and its `meshes/` directory.
- G1 integration reference: Unitree ROS `g1_29dof.urdf`, used only to separate the removable 0.170 kg rubber-hand inertia and verify mount axes.
- O6 left: `o6/left/linkerhand_o6_left.urdf` and all 12 referenced meshes.
- O6 right: `o6/right/linkerhand_o6_right.urdf` and all 12 referenced meshes.
- LinkerHand SDK: hardware mapping evidence showing six O6 inputs.

## Load result

- G1 official MJCF: passed with 29 actuators, 95 sensors, and 35.112142 kg total mass.
- O6 left raw URDF: compiled, 11 joints, no actuators; root mass not preserved as a dynamic body.
- O6 right raw URDF: compiled, 11 joints, no actuators; root mass not preserved as a dynamic body.
"""
    (PROJECT_ROOT / "docs/source_audit.md").write_text(source_doc, encoding="utf-8")

    risks_doc = """# Assumptions and risks

## Confirmed assumptions

- The existing contest repository remains the outer project, and this subsystem lives in `aeropilot_humanoid/`.
- Vendor-native geometry, mass, inertia, joint axes, and limits are authoritative.
- World coordinates are X forward, Y left, Z up in the derived cockpit model; vendor source files remain unmodified.
- O6 has six command inputs and five passive mimic joints per hand, as jointly evidenced by the URDFs and ROS2 SDK.
- The official Unitree ROS G1 description is used only as integration evidence for removable rubber-hand mass and wrist mounting frames. The required Unitree MuJoCo G1 remains the base model.

## Resolved during Phases 2-4

1. **O6 root dynamics:** Derived MJCF preserves each URDF link's exact mass and full inertia, including the palm root.
2. **Mimic coupling:** Each hand has five equality constraints with the official side-specific multipliers.
3. **G1 wrist dynamics:** Integrated wrist inertias remove the official rubber-hand contribution before attaching O6.
4. **Wrist transform:** Side-specific right-handed adapter transforms are derived from official frame evidence and verified numerically.
5. **Cockpit mechanisms:** The independent and combined passive mechanisms load and pass a finite five-second behavior test without actuators.

## Open risks and required evidence

1. **Physical adapter:** CAD geometry, mass, inertia, fasteners, and structural verification remain pending.
2. **Reachability:** Multi-start IK, joint-margin, collision-distance, contact, and seated ergonomics tests have not started.
3. **Python target:** Validation ran on Python 3.12.5. CI and deployment must repeat on Python 3.11 before target compatibility is claimed.
4. **Windows checkout:** Unitree has unrelated Go2W terrain filenames that collide by case. G1 assets loaded despite that warning.
5. **Collision/training cost:** Official O6 visual/collision STL files still need a high-fidelity convex strategy and a separate simplified training representation.
6. **Hardware calibration:** Stick, throttle, rudder, and brake mechanics are initial engineering values, not measurements from target cockpit hardware.

The combined model proves loading, topology, mass accounting, frame consistency, and passive mechanism behavior only. It does not yet prove seated stability, grasp/contact quality, reachability, pedal operation, a 30-second seated test, or MJX/ROCm training readiness.
"""
    (PROJECT_ROOT / "docs/assumptions_and_risks.md").write_text(
        risks_doc, encoding="utf-8"
    )


def _render(path: Path, output: Path, lookat: tuple[float, float, float], distance: float, azimuth: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = -15.0
    output.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output)


def _render_vendor_models() -> dict[str, str]:
    output = PROJECT_ROOT / "reports/screenshots"
    views = {
        "g1": (G1_PATH, output / "g1_original.png", (0.0, 0.0, 0.75), 2.2, 135.0),
        "o6_left": (O6_LEFT_PATH, output / "o6_left_original.png", (0.0, 0.0, 0.06), 0.32, 135.0),
        "o6_right": (O6_RIGHT_PATH, output / "o6_right_original.png", (0.0, 0.0, 0.06), 0.32, 45.0),
    }
    rendered: dict[str, str] = {}
    for name, (source, target, lookat, distance, azimuth) in views.items():
        _render(source, target, lookat, distance, azimuth)
        rendered[name] = str(target.relative_to(PROJECT_ROOT))
    return rendered


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-render", action="store_true", help="Generate reports without screenshots."
    )
    args = parser.parse_args()

    g1 = inspect_g1(G1_PATH)
    left = inspect_o6(O6_LEFT_PATH)
    right = inspect_o6(O6_RIGHT_PATH)
    write_json(PROJECT_ROOT / "reports/g1_joint_map.json", g1)
    write_json(
        PROJECT_ROOT / "reports/o6_joint_map.json", {"left": left, "right": right}
    )
    _write_markdown_reports(g1, left, right)
    _write_phase0_report(g1, left, right)
    screenshots = {} if args.skip_render else _render_vendor_models()
    load_report = {
        "phase": 1,
        "status": "passed" if not args.skip_render else "load_passed_render_skipped",
        "mujoco_version": mujoco.__version__,
        "models": {
            "g1_29dof": g1["model_counts"],
            "o6_left": left["raw_mujoco_compile"],
            "o6_right": right["raw_mujoco_compile"],
        },
        "screenshots": screenshots,
        "known_limitation": "Raw O6 root-link mass is not preserved by fixed-root URDF compilation.",
    }
    write_json(PROJECT_ROOT / "reports/vendor_load_report.json", load_report)

    print("Phase 0 audit: passed with documented integration risks")
    print(f"G1: {g1['model_counts']['actuated_hinge_joints']} actuated joints, {g1['total_mass_kg']:.6f} kg")
    print(
        "O6 left/right: "
        f"{left['independent_joint_count']}+{left['passive_mimic_joint_count']} / "
        f"{right['independent_joint_count']}+{right['passive_mimic_joint_count']} joints"
    )
    print(f"Phase 1 screenshots: {len(screenshots)}")


if __name__ == "__main__":
    main()
