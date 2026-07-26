# Assumptions and risks

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
