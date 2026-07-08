# my_control_demo

双臂 UR 机械臂控制系统的**演示与测试节点**集合。提供四个独立的可执行节点，分别演示 ros2_control 轨迹控制、MoveIt 运动规划、MoveIt Servo 实时伺服以及 Mujoco 仿真控制。

> 仅支持 ROS2 Jazzy。所有节点默认针对双臂命名空间 `arm_A` / `arm_B`。

## 编译

```bash
colcon build --symlink-install --packages-select my_control_demo
source install/setup.bash
```

## 节点总览

| 可执行节点 | 源文件 | 作用 |
|---------|--------|------|
| `my_env_ros2_control` | `src/my_env_ros2_control.cpp` | 直接向 ros2_control 控制器发送关节轨迹（action / topic / forward position），验证底层控制器 |
| `my_env_move_group` | `src/my_env_move_group.cpp` | 通过 MoveIt MoveGroup 接口做规划+执行，测试单臂与双臂规划组 |
| `my_env_move_servo` | `src/my_env_move_servo.cpp` | 向 MoveIt Servo 发送连续正弦位姿目标，测试实时伺服跟踪 |
| `my_env_mujoco_control` | `src/my_env_mujoco_control.cpp` | 向 forward_position_controller 与夹爪循环发送预设动作，用于 Mujoco 仿真 |

---

## 1. my_env_ros2_control

直接与 ros2_control 控制器交互，围绕当前关节位置生成小幅航点轨迹（±delta），并校验到位情况。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `controller_type` | `scaled_joint_trajectory_controller` | 控制器类型后缀，或完整控制器名（如 `arm_A_ur_forward_position_controller`） |
| `arm_target` | `both` | 目标臂：`A` / `B` / `both` |
| `joint_state_topic` | `/joint_states` | 关节状态话题 |
| `command_mode` | `action` | 轨迹下发方式：`action` 或 `topic`（forward_position 控制器自动走 commands 话题） |
| `trajectory_time_sec` | `2.0` | 每段航点的运动时长 |
| `small_delta_rad` / `medium_delta_rad` / `large_delta_rad` | `0.03` / `0.08` / `0.15` | 各关节航点偏移幅度 |
| `goal_position_tolerance_rad` | `0.01` | 每关节到位容差，写入 action goal 的 `goal_tolerance` |

### 运行

```bash
# 双臂，scaled_joint_trajectory_controller，action 模式
ros2 run my_control_demo my_env_ros2_control

# 仅 arm_A，放宽到位容差
ros2 run my_control_demo my_env_ros2_control --ros-args -p arm_target:=A -p goal_position_tolerance_rad:=0.02
```

> 注意：航点首尾相同（回到起点），仅 `goal_tolerance` 无法判断中间是否真的运动过。真机测试时务必确认 UR 上 External Control 程序处于 play 状态，否则控制器会在机械臂不动的情况下仍报 success。

---

## 2. my_env_move_group

通过 MoveIt MoveGroup 接口对 `arm_A`、`arm_B`、`dual_arm` 三个规划组做规划并执行。会从 `/move_group` 拷贝 robot_description 等参数。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `move_group_parameter_node` | `/move_group` | 拷贝 MoveIt 参数的来源节点 |

### 运行

```bash
# 先启动底层控制器与 MoveIt
ros2 launch my_env_control start_my_env_control.launch.py launch_rviz:=false
ros2 launch my_env_moveit_config start_my_env_moveit.launch.py launch_rviz:=true

# 再运行规划测试
ros2 run my_control_demo my_env_move_group
```

---

## 3. my_env_move_servo

向 MoveIt Servo 发送**预设连续正弦位姿目标**，测试实时位姿伺服跟踪。目标位姿 = 阶段起始实际位姿 + 幅值·sin(2π·t/period)，相邻目标点的时间间隔由 `publish_rate_hz` 决定。

启动时自动通过 `/arm_<id>_servo_node/switch_command_type` 服务把每个臂切换到 **POSE** 伺服模式（command_type=2）。

### 测试流程

每阶段时长 = `num_periods × sine_period_sec`：

1. 单臂阶段：依次遍历每个选中臂（先 A 后 B），每臂执行：
   - 3 轴位置正弦（仅平移）
   - 3 轴姿态正弦（仅旋转）
   - 6 轴位姿正弦（平移 + 旋转）
2. 双臂阶段（仅 `servo_target=both`）：两臂**同时**做 6 轴位姿正弦

所以 `A` / `B` 各 3 个阶段，`both` 共 7 个阶段（3+3+1）。正弦各轴同相位同幅值，从 0 相位起，保证连续无跳变。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `servo_target` | `both` | 目标臂：`A` / `B` / `both` |
| `planning_frame` | `world` | 位姿目标参考系 |
| `publish_rate_hz` | `100.0` | 目标点发布频率（决定相邻目标点时间间隔） |
| `sine_period_sec` | `6.0` | 正弦周期时长 |
| `num_periods` | `3.0` | 每阶段持续周期数 |
| `position_amplitude_m` | `0.05` | 位置正弦幅值（米） |
| `orientation_amplitude_rad` | `0.1` | 姿态正弦幅值（弧度） |
| `settle_time_sec` | `1.0` | 阶段间保持起点稳定的时间 |

运行时每秒打印一次跟踪误差（目标位姿 vs 从 TF 读取的实际 TCP 位姿）：位置误差(m) 与姿态误差(rad/deg)。

### 运行

```bash
# 先启动控制器（forward_position_controller）与 Servo
ros2 launch my_env_control start_my_env_control.launch.py \
  launch_rviz:=false initial_ur_controller:=forward_position_controller
ros2 launch my_env_moveit_config start_my_env_servo.launch.py

# 单臂测试（3 阶段）
ros2 run my_control_demo my_env_move_servo --ros-args -p servo_target:=A

# 双臂测试（7 阶段），自定义幅值与周期
ros2 run my_control_demo my_env_move_servo --ros-args \
  -p servo_target:=both -p sine_period_sec:=8.0 -p position_amplitude_m:=0.03
```

---

## 4. my_env_mujoco_control

面向 Mujoco 仿真的循环动作演示。每 5 秒切换一次动作：初始位姿(开夹爪) → home 位姿 → 闭合夹爪。向 `/arm_{A,B}_forward_position_controller/commands` 发布关节位置，向 `/arm_{A,B}_robotiq_gripper_controller/gripper_cmd` 发送夹爪 action。

### 运行

```bash
ros2 run my_control_demo my_env_mujoco_control
```

---

## launch

`launch/my_control.launch.py` 提供一个示例启动文件，默认只启动 `my_env_move_group`，其余节点以注释形式列出，按需取消注释：

```bash
ros2 launch my_control_demo my_control.launch.py
```
