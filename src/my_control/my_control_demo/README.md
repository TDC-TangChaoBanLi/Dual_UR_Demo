# my_control_demo

双臂 UR 机械臂控制系统的**演示与测试节点**集合。提供 ros2_control 轨迹控制、MoveIt 运动规划、MoveIt Servo 实时伺服、小车联动以及 Mujoco 仿真控制节点。

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
| `railcar_sine_servo` | `src/railcar_sine_servo.cpp` | 读取小车正弦运动位置，并映射到 arm_A TCP 的 y 轴目标 |
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

测试节点不访问 Servo 的参数服务，也不自动切换指令类型。两个机械臂的 frame 和 topic 均由本节点参数传入；运行测试前需在外部把相应 Servo 节点切换到 **POSE** 模式（command_type=2）。

### 测试流程

每阶段时长 = `num_periods × sine_period_sec`：

1. 单臂阶段：依次遍历两个机械臂（先 A 后 B），每臂执行：
   - 3 轴位置正弦（仅平移）
   - 3 轴姿态正弦（仅旋转）
   - 6 轴位姿正弦（平移 + 旋转）
2. 双臂阶段：两臂**同时**做 6 轴位姿正弦

测试固定执行 7 个阶段（3+3+1）。X/Y/Z（以及 roll/pitch/yaw）依次使用 0°、120°、240° 相位；每轴减去其初始相位值，使阶段开始和完整周期结束时目标都等于阶段起始位姿，避免指令跳变。

### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arm_a_planning_frame` | `world` | arm A 位姿目标及误差计算参考系 |
| `arm_a_tcp_frame` | `arm_A__tcp` | arm A 实际 TCP 的 TF frame |
| `arm_a_pose_topic` | `/arm_A_servo_node/pose_target_cmds` | arm A 位姿目标话题 |
| `arm_a_status_topic` | `/arm_A_servo_node/status` | arm A Servo 状态话题 |
| `arm_b_planning_frame` | `world` | arm B 位姿目标及误差计算参考系 |
| `arm_b_tcp_frame` | `arm_B__tcp` | arm B 实际 TCP 的 TF frame |
| `arm_b_pose_topic` | `/arm_B_servo_node/pose_target_cmds` | arm B 位姿目标话题 |
| `arm_b_status_topic` | `/arm_B_servo_node/status` | arm B Servo 状态话题 |
| `publish_rate_hz` | `100.0` | 目标点发布频率（决定相邻目标点时间间隔） |
| `sine_period_sec` | `6.0` | 正弦周期时长 |
| `num_periods` | `3.0` | 每阶段持续周期数 |
| `position_amplitude_m` | `0.05` | 每个位置轴正弦分量的幅值（米） |
| `orientation_amplitude_rad` | `0.1` | 每个 RPY 轴正弦分量的幅值（弧度） |
| `settle_time_sec` | `1.0` | 阶段间保持起点稳定的时间 |

测试节点在每个目标发布周期采集一次“最新目标位姿 vs 从 TF 读取的最新实际 TCP 位姿”误差，并把样本保存在内存中。每个阶段结束后分别输出各机械臂的位置误差和姿态误差统计：有效样本数、不可用样本数、最大值、平均值和总体标准差；不再按固定周期打印瞬时误差。

### 运行

```bash
# 先启动控制器（forward_position_controller）与 Servo
ros2 launch my_env_control start_my_env_control.launch.py \
  launch_rviz:=false initial_ur_controller:=forward_position_controller
# 可组合组件方式会让 Servo 的状态监视回调在初始化期间得到执行；双臂本机测试推荐使用
ros2 launch my_env_moveit_config start_my_env_servo.launch.py launch_as_component:=true

# 测试节点不再调用服务；先在外部把两个 Servo 节点切换为 POSE 模式
ros2 service call /arm_A_servo_node/switch_command_type \
  moveit_msgs/srv/ServoCommandType "{command_type: 2}"
ros2 service call /arm_B_servo_node/switch_command_type \
  moveit_msgs/srv/ServoCommandType "{command_type: 2}"

# 双臂测试（共 7 个阶段），自定义幅值、周期和 arm A TCP frame
ros2 run my_control_demo my_env_move_servo --ros-args \
  -p sine_period_sec:=8.0 -p position_amplitude_m:=0.03 \
  -p arm_a_tcp_frame:=arm_A__tcp
```

---

## 4. railcar_sine_servo

启动时直接通过 TF 读取 `world` 到 `arm_A__tcp` 的变换并保存 TCP 初始位姿，不请求任何 ROS 服务。随后每 50 ms 向小车发送固定正弦指令（0.1 Hz、20 mm、5 周期），读取当前位置，并发布：

```text
目标 TCP y = 初始 TCP y + 小车当前位置(mm) / 1000
```

目标发布到 `/arm_A_servo_node/pose_target_cmds`；TCP 的 x、z 和姿态保持初始值。退出时节点会向小车发送停止帧。

```bash
ros2 run my_control_demo railcar_sine_servo

# 可按现场网络配置覆盖参数
ros2 run my_control_demo railcar_sine_servo --ros-args \
  -p railcar_ip:=192.168.1.88 -p railcar_port:=6000 \
  -p local_ip:="" -p local_port:=6000
```

TF 参考系固定为 `planning_frame=world`、`tcp_frame=arm_A__tcp`。参数默认值：`pose_topic=/arm_A_servo_node/pose_target_cmds`、`railcar_ip=192.168.1.88`、`railcar_port=6000`、`local_ip=""`、`local_port=6000`。

---

## 5. my_env_mujoco_control

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
