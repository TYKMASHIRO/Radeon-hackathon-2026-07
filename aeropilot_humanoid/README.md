# Physical AI Bionic Pilot — MuJoCo Model Foundation

This directory contains the verified Phase 0-4 foundation for the Unitree G1
29DoF and dual LinkerHand O6 bionic-pilot model. It audits the official sources,
converts both O6 hands without losing their passive coupling, integrates them
with G1, and provides independently testable passive cockpit mechanisms.

The vendor repositories are intentionally ignored by the parent Git repository.
Fetch them at the commits recorded in `third_party/source_manifest.yaml`; never
edit them in place. All conversions and adapters belong under `models/derived/`.

## Reproduce the audit

Use Python 3.11 for the target environment. The current development environment
is Python 3.12.5 and is recorded as an open compatibility deviation.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m apps.inspect_vendor_models
.\.venv\Scripts\python.exe -m apps.convert_o6_urdf
.\.venv\Scripts\python.exe -m apps.build_combined_model
.\.venv\Scripts\python.exe -m apps.build_cockpit
blender --background --python apps/convert_fbx_controls.py
.\.venv\Scripts\python.exe -m apps.build_pilot_action
.\.venv\Scripts\python.exe -m apps.run_pilot_control_sequence
.\.venv\Scripts\python.exe -m apps.build_hand_follow_controls
.\.venv\Scripts\python.exe -m apps.render_hand_follow_controls
.\.venv\Scripts\python.exe -m pytest -v
```

The commands write machine-readable reports under `reports/`, design evidence
under `docs/`, derived MJCF under `models/derived/`, and screenshots under
`reports/screenshots/`.

## Verified scope

- Official Unitree G1 29DoF MJCF, including 29 actuators.
- Official, separate left and right O6 URDFs and meshes.
- O6 structure: 6 independent inputs and 5 mimic joints per hand.
- Independent MuJoCo compilation of all three source models.
- Exact O6 mass/inertia restoration and five official mimic constraints per hand.
- G1 integration with the removable rubber-hand mass accounted for and no invented O6 actuators.
- Passive seat, stick, throttle, rudder, and brake mechanism load/behavior tests.
- Humanoid pilot-control keyframes and a rendered sequence where the left hand
  advances the throttle and the right hand deflects the control stick.
- Hand-position following where the right hand drives the two-axis center stick
  and only the left hand's aircraft-forward displacement drives the throttle.

The raw O6 URDF compiler fuses the fixed root link into the world and does not
preserve its root inertia as an attachable body. The derived MJCF corrects that
and restores all mimic constraints without inventing actuators. Reachability,
seated stability, grasp/contact quality, physical adapter CAD, hardware
calibration, and MJX/ROCm training readiness remain outside the verified scope.
