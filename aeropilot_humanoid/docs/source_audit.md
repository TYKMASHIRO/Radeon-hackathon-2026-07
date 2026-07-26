# Official source audit

Audit date: 2026-07-16

| Source | Repository | Branch | Commit | License | Modified |
|---|---|---|---|---|---|
| unitree_mujoco | [https://github.com/unitreerobotics/unitree_mujoco.git](https://github.com/unitreerobotics/unitree_mujoco.git) | `main` | `ae6a8403e272733e9996ef59990880330496177f` | BSD-3-Clause | False |
| unitree_ros | [https://github.com/unitreerobotics/unitree_ros.git](https://github.com/unitreerobotics/unitree_ros.git) | `master` | `d96d8f63ae17a7108d4f7229c00ef875ba7129c9` | BSD-3-Clause | False |
| linkerhand_urdf | [https://github.com/linker-bot/linkerhand-urdf.git](https://github.com/linker-bot/linkerhand-urdf.git) | `main` | `735145e8843f44d85c1464725e6971ba97a6258e` | Apache-2.0 | False |
| linkerhand_ros2_sdk | [https://github.com/linker-bot/linkerhand-ros2-sdk.git](https://github.com/linker-bot/linkerhand-ros2-sdk.git) | `main` | `5f4f36c660843f7fc26990348d49b402ed9fac48` | Apache-2.0 | False |

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
