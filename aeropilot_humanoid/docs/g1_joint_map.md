# G1 29DoF joint map

Generated from the loaded MuJoCo parent-child tree, not from filename or substring counts.

- Load: **passed**
- Physical bodies: 30 (31 including world)
- Joints: 30 including the floating base
- Actuated hinge joints: 29
- Actuators: 29
- Sensors: 95
- Total mass: 35.112142000 kg
- Branch counts: `{"left_arm": 7, "left_leg": 6, "right_arm": 7, "right_leg": 6, "waist": 3}`
- Left wrist terminal link: `left_wrist_yaw_link`
- Right wrist terminal link: `right_wrist_yaw_link`

| # | Joint | Body | Parent body | Branch | Axis | Range (rad) | Actuator |
|---:|---|---|---|---|---|---|---|
| 0 | floating_base_joint | pelvis | world | floating_base | [0.0, 0.0, 1.0] | None | — |
| 1 | left_hip_pitch_joint | left_hip_pitch_link | pelvis | left_leg | [0.0, 1.0, 0.0] | [-2.5307, 2.8798] | left_hip_pitch |
| 2 | left_hip_roll_joint | left_hip_roll_link | left_hip_pitch_link | left_leg | [1.0, 0.0, 0.0] | [-0.5236, 2.9671] | left_hip_roll |
| 3 | left_hip_yaw_joint | left_hip_yaw_link | left_hip_roll_link | left_leg | [0.0, 0.0, 1.0] | [-2.7576, 2.7576] | left_hip_yaw |
| 4 | left_knee_joint | left_knee_link | left_hip_yaw_link | left_leg | [0.0, 1.0, 0.0] | [-0.087267, 2.8798] | left_knee |
| 5 | left_ankle_pitch_joint | left_ankle_pitch_link | left_knee_link | left_leg | [0.0, 1.0, 0.0] | [-0.87267, 0.5236] | left_ankle_pitch |
| 6 | left_ankle_roll_joint | left_ankle_roll_link | left_ankle_pitch_link | left_leg | [1.0, 0.0, 0.0] | [-0.2618, 0.2618] | left_ankle_roll |
| 7 | right_hip_pitch_joint | right_hip_pitch_link | pelvis | right_leg | [0.0, 1.0, 0.0] | [-2.5307, 2.8798] | right_hip_pitch |
| 8 | right_hip_roll_joint | right_hip_roll_link | right_hip_pitch_link | right_leg | [1.0, 0.0, 0.0] | [-2.9671, 0.5236] | right_hip_roll |
| 9 | right_hip_yaw_joint | right_hip_yaw_link | right_hip_roll_link | right_leg | [0.0, 0.0, 1.0] | [-2.7576, 2.7576] | right_hip_yaw |
| 10 | right_knee_joint | right_knee_link | right_hip_yaw_link | right_leg | [0.0, 1.0, 0.0] | [-0.087267, 2.8798] | right_knee |
| 11 | right_ankle_pitch_joint | right_ankle_pitch_link | right_knee_link | right_leg | [0.0, 1.0, 0.0] | [-0.87267, 0.5236] | right_ankle_pitch |
| 12 | right_ankle_roll_joint | right_ankle_roll_link | right_ankle_pitch_link | right_leg | [1.0, 0.0, 0.0] | [-0.2618, 0.2618] | right_ankle_roll |
| 13 | waist_yaw_joint | waist_yaw_link | pelvis | waist | [0.0, 0.0, 1.0] | [-2.618, 2.618] | waist_yaw |
| 14 | waist_roll_joint | waist_roll_link | waist_yaw_link | waist | [1.0, 0.0, 0.0] | [-0.52, 0.52] | waist_roll |
| 15 | waist_pitch_joint | torso_link | waist_roll_link | waist | [0.0, 1.0, 0.0] | [-0.52, 0.52] | waist_pitch |
| 16 | left_shoulder_pitch_joint | left_shoulder_pitch_link | torso_link | left_arm | [0.0, 1.0, 0.0] | [-3.0892, 2.6704] | left_shoulder_pitch |
| 17 | left_shoulder_roll_joint | left_shoulder_roll_link | left_shoulder_pitch_link | left_arm | [1.0, 0.0, 0.0] | [-1.5882, 2.2515] | left_shoulder_roll |
| 18 | left_shoulder_yaw_joint | left_shoulder_yaw_link | left_shoulder_roll_link | left_arm | [0.0, 0.0, 1.0] | [-2.618, 2.618] | left_shoulder_yaw |
| 19 | left_elbow_joint | left_elbow_link | left_shoulder_yaw_link | left_arm | [0.0, 1.0, 0.0] | [-1.0472, 2.0944] | left_elbow |
| 20 | left_wrist_roll_joint | left_wrist_roll_link | left_elbow_link | left_arm | [1.0, 0.0, 0.0] | [-1.97222, 1.97222] | left_wrist_roll |
| 21 | left_wrist_pitch_joint | left_wrist_pitch_link | left_wrist_roll_link | left_arm | [0.0, 1.0, 0.0] | [-1.61443, 1.61443] | left_wrist_pitch |
| 22 | left_wrist_yaw_joint | left_wrist_yaw_link | left_wrist_pitch_link | left_arm | [0.0, 0.0, 1.0] | [-1.61443, 1.61443] | left_wrist_yaw |
| 23 | right_shoulder_pitch_joint | right_shoulder_pitch_link | torso_link | right_arm | [0.0, 1.0, 0.0] | [-3.0892, 2.6704] | right_shoulder_pitch |
| 24 | right_shoulder_roll_joint | right_shoulder_roll_link | right_shoulder_pitch_link | right_arm | [1.0, 0.0, 0.0] | [-2.2515, 1.5882] | right_shoulder_roll |
| 25 | right_shoulder_yaw_joint | right_shoulder_yaw_link | right_shoulder_roll_link | right_arm | [0.0, 0.0, 1.0] | [-2.618, 2.618] | right_shoulder_yaw |
| 26 | right_elbow_joint | right_elbow_link | right_shoulder_yaw_link | right_arm | [0.0, 1.0, 0.0] | [-1.0472, 2.0944] | right_elbow |
| 27 | right_wrist_roll_joint | right_wrist_roll_link | right_elbow_link | right_arm | [1.0, 0.0, 0.0] | [-1.97222, 1.97222] | right_wrist_roll |
| 28 | right_wrist_pitch_joint | right_wrist_pitch_link | right_wrist_roll_link | right_arm | [0.0, 1.0, 0.0] | [-1.61443, 1.61443] | right_wrist_pitch |
| 29 | right_wrist_yaw_joint | right_wrist_yaw_link | right_wrist_pitch_link | right_arm | [0.0, 0.0, 1.0] | [-1.61443, 1.61443] | right_wrist_yaw |

## Scope boundary

The six-joint leg chains are structurally present. Pedal reachability and physical contact are not proven by topology and remain Phase 5/6 work.
