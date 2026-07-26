# O6 URDF to MJCF conversion

Both official hands were converted independently with MuJoCo's URDF compiler. A temporary floating mount forces the palm root to remain a body; the temporary joint is then removed, portable relative mesh paths are restored, and the five official mimic tags are encoded as MuJoCo joint equalities. Vendor files remain untouched.

| Hand | URDF mass (kg) | Derived mass (kg) | Joints | Equalities | Max motion (m) | Result |
|---|---:|---:|---:|---:|---:|---|
| left | 0.113856113 | 0.113856113 | 11 | 5 | 0.017296 | passed |
| right | 0.113956201 | 0.113956201 | 11 | 5 | 0.018297 | passed |

The motion check is kinematic and coupling-consistent; it is not a claim about actuator torque, grasp force, or dynamic control.
