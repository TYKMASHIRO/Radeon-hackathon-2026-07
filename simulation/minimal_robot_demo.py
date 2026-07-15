"""Minimal MuJoCo robot body simulation.

Run:
    python simulation/minimal_robot_demo.py
"""

from __future__ import annotations

import math

import mujoco
import numpy as np


ROBOT_XML = """
<mujoco model="minimal_robot">
  <compiler angle="degree" coordinate="local"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <default>
    <geom friction="1.0 0.005 0.0001"/>
    <joint damping="0.04" armature="0.01"/>
    <motor ctrlrange="-1 1"/>
  </default>

  <worldbody>
    <light name="top" pos="0 0 2"/>
    <geom name="floor" type="plane" size="3 3 0.05" rgba="0.75 0.8 0.85 1"/>

    <body name="base" pos="0 0 0.16">
      <freejoint name="base_free"/>
      <geom name="body" type="box" size="0.18 0.12 0.05" mass="1.0" rgba="0.2 0.4 0.8 1"/>

      <body name="left_leg" pos="0 0.09 -0.04">
        <joint name="left_hip" type="hinge" axis="0 1 0" range="-45 45"/>
        <geom name="left_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.18" size="0.025" mass="0.15" rgba="0.9 0.45 0.2 1"/>
      </body>

      <body name="right_leg" pos="0 -0.09 -0.04">
        <joint name="right_hip" type="hinge" axis="0 1 0" range="-45 45"/>
        <geom name="right_leg_geom" type="capsule" fromto="0 0 0 0 0 -0.18" size="0.025" mass="0.15" rgba="0.9 0.45 0.2 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="left_hip_motor" joint="left_hip" gear="0.35"/>
    <motor name="right_hip_motor" joint="right_hip" gear="0.35"/>
  </actuator>
</mujoco>
"""


def run_simulation(duration: float = 2.0) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_string(ROBOT_XML)
    data = mujoco.MjData(model)

    while data.time < duration:
        data.ctrl[0] = 0.7 * math.sin(2.0 * math.pi * data.time)
        data.ctrl[1] = 0.7 * math.sin(2.0 * math.pi * data.time + math.pi)
        mujoco.mj_step(model, data)

    base_position = np.array(data.qpos[:3], dtype=float)
    left_hip_angle = float(data.qpos[7])
    right_hip_angle = float(data.qpos[8])

    return {
        "time_s": float(data.time),
        "base_x_m": float(base_position[0]),
        "base_y_m": float(base_position[1]),
        "base_z_m": float(base_position[2]),
        "left_hip_rad": left_hip_angle,
        "right_hip_rad": right_hip_angle,
    }


def main() -> None:
    result = run_simulation()
    print("MuJoCo minimal robot demo finished.")
    for key, value in result.items():
        print(f"{key}: {value:.6f}")


if __name__ == "__main__":
    main()
