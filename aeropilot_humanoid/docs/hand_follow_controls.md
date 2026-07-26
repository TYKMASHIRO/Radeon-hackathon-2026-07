# 手部空间位置随动控制

生成后的 MuJoCo 场景：

- `models/scenes/hand_follow_controls.xml`
- `models/derived/pilot/g1_o6_hand_follow_controls.xml`

从原始 FBX 重新生成拆分网格：

```powershell
blender --background --python apps/convert_fbx_controls.py
```

转换器会把 Blender 的 OBJ 导出轴变换 `(x,y,z) → (y,-x,z)` 纳入
支点计算，并在导出前合并每个机构分组，确保 MuJoCo 载入完整的
摇杆和油门网格，而不是只载入分组中的第一个对象。FBX 三角网格仅
用于显示，移动体质量和惯量独立定义，避免高精度视觉网格造成错误
碰撞力。

坐标系为 X 向飞机前方、Y 向左、Z 向上。右手世界坐标控制中置
摇杆的横滚和俯仰；左手位移只投影到 X 轴，因此油门只有前进和后退。

运行生成和验证：

```powershell
python apps/build_hand_follow_controls.py
python apps/render_hand_follow_controls.py
```

在控制循环中使用：

```python
from pathlib import Path

import mujoco
import numpy as np

from aeropilot_humanoid.hand_follow_controller import HandFollowController

model = mujoco.MjModel.from_xml_path(
    "models/scenes/hand_follow_controls.xml"
)
data = mujoco.MjData(model)

# 校准时双手应位于中立握持位置。
ready = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "pilot_ready_on_controls",
)
data.qpos[:] = model.key_qpos[ready]
mujoco.mj_forward(model, data)
controller = HandFollowController(model, data)

while data.time < 3.0:
    # 不传参数时直接读取模型中的左右手位置。
    controller.update()

    # 外部手部跟踪器也可直接传入世界坐标：
    # controller.update(
    #     right_hand_world=np.array([x_r, y_r, z_r]),
    #     left_hand_world=np.array([x_l, y_l, z_l]),
    # )
    mujoco.mj_step(model, data)
```

三个位置执行器分别是：

- `stick_roll_hand_follow`
- `stick_pitch_hand_follow`
- `throttle_hand_follow`

控制器会把摇杆角度限制在 XML 关节范围 `[-0.3, 0.3] rad`，把油门
限制在 `[0, 0.16] m`。左手 Y/Z 方向的移动不会改变油门指令。
