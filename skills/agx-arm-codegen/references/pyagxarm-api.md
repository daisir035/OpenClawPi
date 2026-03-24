# pyAgxArm API 速查与最小可运行模板

供 OpenClaw 根据用户自然语言生成机械臂控制代码时参考。SDK 来源：pyAgxArm（[GitHub](https://github.com/agilexrobotics/pyAgxArm)）；示例参考：`pyAgxArm/demos/nero/test1.py`。

## 1. 连接与配置

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

# 配置：robot 可选 nero / piper / piper_h / piper_l / piper_x；channel 如 can0
robot_cfg = create_agx_arm_config(
    robot="nero",
    comm="can",
    channel="can0",
    interface="socketcan",
)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()
```

- `create_agx_arm_config(robot, comm="can", channel="can0", interface="socketcan", **kwargs)`：创建配置字典；CAN 相关参数通过 kwargs 传入（如 channel、interface）。
- `AgxArmFactory.create_arm(config)`：返回机械臂驱动实例。
- `robot.connect()`：建立 CAN 连接并启动读取线程。

## 2. 使能与模式

```python
robot.set_normal_mode()   # 普通模式（单臂控制）
# 使能：轮询直到成功
while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)   # 运动速度百分比 0–100
# 失能
while not robot.disable():
    time.sleep(0.01)
```

- 主从模式（Nero/Piper 等）：`robot.set_master_mode()`（零力拖拽）、`robot.set_slave_mode()`（跟随主臂）。

## 3. 运动模式与运动接口

| 模式 | 常量 | 接口 | 说明 |
|------|------|------|------|
| 关节位置速度 | `robot.MOTION_MODE.J` | `robot.move_j([j1..j7])` | 7 个关节角（弧度），有平滑 |
| 关节快速响应 | `robot.MOTION_MODE.JS` | `robot.move_js([j1..j7])` | 无平滑，慎用 |
| 点到点 | `robot.MOTION_MODE.P` | `robot.move_p([x,y,z,roll,pitch,yaw])` | 笛卡尔位姿，米/弧度 |
| 直线 | `robot.MOTION_MODE.L` | `robot.move_l([x,y,z,roll,pitch,yaw])` | 直线轨迹 |
| 圆弧 | `robot.MOTION_MODE.C` | `robot.move_c(start_pose, mid_pose, end_pose)` | 每 pose 6 个浮点数 |

- 单位：关节角为**弧度**；笛卡尔位姿为**米**（x,y,z）和**弧度**（roll, pitch, yaw）。
- Nero 为 7 关节；Piper 为 6 关节，`move_j`/`move_js` 参数数量需与机型一致。

### 3.1 Piper（六轴）关节控制要点

- `move_j([j1, j2, j3, j4, j5, j6])`：6 个关节角（rad），用于位置-速度模式（平滑）。
- `move_js([j1, j2, j3, j4, j5, j6])`：6 个关节角（rad），用于 JS（Follower/MIT 透传）模式（无平滑，慎用且通常风险更高）。
- 推荐写法：用 `robot.joint_nums` 做长度判断，避免数组长度写错。

示例（Piper 六轴 move_j + 等待完成）：

```python
import time

def move_piper_joints(robot, joints6, timeout: float = 3.0) -> bool:
    assert len(joints6) == 6, "Piper/piper_* 需要 6 个关节角"
    robot.set_motion_mode(robot.MOTION_MODE.J)  # 旧规则：显式设置也可
    robot.move_j(joints6)
    return wait_motion_done(robot, timeout=timeout)
```

### 3.2 Nero（七轴）关节控制要点

- `move_j([j1, j2, j3, j4, j5, j6, j7])`：7 个关节角（rad），用于位置-速度模式（平滑）。
- `move_js([j1, j2, j3, j4, j5, j6, j7])`：7 个关节角（rad），用于 JS（Follower/MIT 透传）模式（无平滑，极高风险；除非用户明确要快速响应/跟随类控制）。
- 可将“顺序”绑定到关节编号：用户给的 `关节1..7` 应填入 `j1..j7` 的同序数组。

示例（Nero 七轴 move_j + 等待完成）：

```python
def move_nero_joints(robot, joints7, timeout: float = 3.0) -> bool:
    assert len(joints7) == 7, "Nero 需要 7 个关节角"
    robot.set_motion_mode(robot.MOTION_MODE.J)  # 旧规则：显式设置也可
    robot.move_j(joints7)
    return wait_motion_done(robot, timeout=timeout)
```

示例（关节运动 + 等待完成）：

```python
import time

def wait_motion_done(robot, timeout: float = 3.0, poll_interval: float = 0.1) -> bool:  # Shorter timeout (2-3s)
    time.sleep(0.5)
    start_t = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and getattr(status.msg, "motion_status", None) == 0:
            return True
        if time.monotonic() - start_t > timeout:
            return False
        time.sleep(poll_interval)

robot.set_motion_mode(robot.MOTION_MODE.J)
robot.move_j([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
wait_motion_done(robot, timeout=3.0)  # Shorter timeout
```

## 4. 读取状态

- `robot.get_joint_angles()`：当前关节角（返回值带 `.msg` 属性时为数组）。
- `robot.get_flange_pose()`：当前法兰位姿 `[x, y, z, roll, pitch, yaw]`。
- `robot.get_arm_status()`：运动状态等；`status.msg.motion_status == 0` 表示运动完成。
- 注意：运动完成后检测 `robot.get_arm_status().msg.motion_status == 0`（不是 == 1）

### 4.1 读取机械臂参数（可选，推荐用于调试）

- `robot.joint_nums`：关节数量（Nero=7，Piper/piper_*=6）。
- `robot.is_ok()`：数据接收监控是否正常（SDK 内部基于最近一段时间是否持续收不到数据判断；有该接口时可调用）。
- `robot.get_fps()`：数据接收频率（Hz；有该接口时可调用）。
- 关节使能状态：
  - `robot.get_joint_enable_status(joint_index)`：读取单关节使能（joint_index 通常从 `1` 开始）。
  - `robot.get_joints_enable_status_list()`：读取全部关节使能列表（按 `1..joint_nums` 顺序）。
- 电机/驱动器反馈（单关节；建议用 `for i in range(1, robot.joint_nums + 1)` 循环）：
  - `robot.get_motor_states(joint_index)`：电机高频反馈（位置/速度/电流/扭矩）。
  - `robot.get_driver_states(joint_index)`：驱动器低频反馈（电压/温度/母线电流/状态位等）。
- TCP 相关（需要你先/后可选设置 `robot.set_tcp_offset(...)`）：
  - `robot.get_tcp_pose()`：读取 TCP 位姿 `[x, y, z, roll, pitch, yaw]`。

示例：运动前后打印关键参数（兼容 6/7 轴）

```python
import time

def read_robot_params(robot, label: str = "state") -> None:
    print(f"\n[{label}] joint_nums =", robot.joint_nums)
    # 有些接口可能在不同机型/固件上可用与否；用 getattr 保守兼容
    if hasattr(robot, "is_ok"):
        try:
            print("is_ok =", robot.is_ok())
        except Exception as e:
            print("is_ok read failed:", e)
    if hasattr(robot, "get_fps"):
        try:
            print("fps =", robot.get_fps(), "Hz")
        except Exception as e:
            print("get_fps read failed:", e)

    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print("arm_status.msg.motion_status =",
              getattr(arm_status.msg, "motion_status", None))

    ja = robot.get_joint_angles()
    if ja is not None:
        print("joint_angles(rad) =", ja.msg)

    fp = robot.get_flange_pose()
    if fp is not None:
        print("flange_pose(m/rad) =", fp.msg)

    if hasattr(robot, "get_joints_enable_status_list"):
        try:
            print("joint_enable_list =", robot.get_joints_enable_status_list())
        except Exception as e:
            print("enable_list read failed:", e)

    # 电机/驱动器逐关节读取（1-based joint_index）
    for i in range(1, robot.joint_nums + 1):
        try:
            ms = robot.get_motor_states(i)
            if ms is not None:
                print(f"motor_states[{i}] =", ms.msg)
        except Exception:
            pass
        try:
            ds = robot.get_driver_states(i)
            if ds is not None:
                print(f"driver_states[{i}] =", ds.msg)
        except Exception:
            pass

    if hasattr(robot, "get_tcp_pose"):
        try:
            tcp = robot.get_tcp_pose()
            if tcp is not None:
                print("tcp_pose(m/rad) =", tcp.msg)
        except Exception:
            pass

    time.sleep(0.005)
```

## 5. 其他

- 回零：`robot.move_j([0] * 7)`（Nero 为 7 关节）；Piper 为 `robot.move_j([0] * 6)`；更通用用 `robot.move_j([0] * robot.joint_nums)`。
- 急停：`robot.electronic_emergency_stop()`；恢复需 `robot.reset()`。
- MIT 阻抗/力矩控制（高级）：`robot.set_motion_mode(robot.MOTION_MODE.MIT)`，`robot.move_mit(joint_index, p_des, v_des, kp, kd, t_ff)`，参数范围见 SDK，慎用。

## 6. 最小可运行模板（生成代码时可基于此扩展）

```python
#!/usr/bin/env python3
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory


def wait_motion_done(robot, timeout: float = 3.0, poll_interval: float = 0.1) -> bool:  # Shorter timeout (2-3s)
    time.sleep(0.5)
    start_t = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and getattr(status.msg, "motion_status", None) == 0:
            return True
        if time.monotonic() - start_t > timeout:
            return False
        time.sleep(poll_interval)


def print_basic_state(robot, label: str = "state") -> None:
    """Basic arm parameter read for debug (兼容 6/7 轴)."""
    try:
        print(f"\n[{label}] joint_nums =", robot.joint_nums)
    except Exception:
        pass

    try:
        arm_status = robot.get_arm_status()
        if arm_status is not None:
            print("arm_status.motion_status =",
                  getattr(arm_status.msg, "motion_status", None))
    except Exception:
        pass

    try:
        ja = robot.get_joint_angles()
        if ja is not None:
            print("joint_angles(rad) =", ja.msg)
    except Exception:
        pass

    try:
        fp = robot.get_flange_pose()
        if fp is not None:
            print("flange_pose(m/rad) =", fp.msg)
    except Exception:
        pass


def main():
    robot_cfg = create_agx_arm_config(
        robot="nero",
        comm="can",
        channel="can0",
        interface="socketcan",
    )
    robot = AgxArmFactory.create_arm(robot_cfg)
    robot.connect()

    # Mode switching requires 1s delay before and after
    time.sleep(1)  # 1s delay before mode switch
    robot.set_normal_mode()
    time.sleep(1)  # 1s delay after mode switch
    
    # CRITICAL: The robot MUST BE ENABLED before switching modes
    while not robot.enable():
        time.sleep(0.01)
    robot.set_speed_percent(80)

    # After each mechanical arm operation, add a small sleep (0.01 seconds)
    # CRITICAL: All movement commands must be used in normal mode
    robot.set_motion_mode(robot.MOTION_MODE.J)
    # 使用 robot.joint_nums 自动匹配 6/7 轴关节数量
    joints = [0.0] * robot.joint_nums
    joints[0] = 0.05

    # 可选调试：运动前读取参数
    # print_basic_state(robot, label="before_move")

    robot.move_j(joints)
    time.sleep(0.01)  # Small delay after move command
    wait_motion_done(robot, timeout=3.0)  # Shorter timeout

    # 可选调试：运动后读取参数
    # print_basic_state(robot, label="after_move")

    # 可选：退出前失能
    # while not robot.disable():
    #     time.sleep(0.01)


if __name__ == "__main__":
    main()
```

生成代码时请根据用户描述替换或增加运动步骤（`move_j` / `move_p` / `move_l` / `move_c` 等），并保持连接、使能、`wait_motion_done` 和单位（弧度/米）一致。
