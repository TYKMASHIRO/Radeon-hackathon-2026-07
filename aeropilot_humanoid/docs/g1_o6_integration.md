# G1 and dual O6 integration

The official G1 29DoF MJCF remains the kinematic source. Unitree's official URDF separates each 0.170 kg removable rubber hand from its 0.08457647 kg wrist-yaw link; the derived model removes the rubber-hand mesh and restores that bare-wrist inertia before attaching O6.

The official G1 hand model extends fingers along +X and distributes finger roots along ±Z. O6 extends fingers along +Z and distributes them along ±Y. Matching official index/middle roots and applying the right-hand rule yields the side-specific matrices recorded in `configs/wrist_adapter.yaml`.

- Combined mass: 34.999955253958 kg
- Joints / actuators / equalities: 52 / 29 / 10
- Initial contacts reported by MuJoCo: 0
- O6 actuator count added: 0
- Remaining hardware item: measured adapter-plate CAD, mass, and inertia.

This phase proves source-correct kinematics, frames, and mass accounting. It does not yet prove grasping, cockpit reachability, or adapter hardware stiffness.
