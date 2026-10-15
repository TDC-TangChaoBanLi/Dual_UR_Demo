# Dual_UR_Demo 手动启动与配置指南

> 本文档包含各运行模式（真实机械臂 / RViz 仿真 / MuJoCo 仿真）的手动启动流程，
> 以及 MoveIt / MoveIt Servo、RealSense 相机、OCS2 安装与使用等详细说明。
>
> 日常使用推荐直接运行仓库根目录的脚本：
> - `./init_repo.sh` — 初始化（选择环境 / 拉取子模块 / 安装依赖）
> - `./quick_start.sh` — 编译与启动（编译 / dashboard_client / 启动控制器 / 相机）

## 目录

1. [依赖安装](#1-依赖安装)
2. [编译本项目](#2-编译本项目)
3. [真实机械臂启动流程](#3-真实机械臂启动流程)
4. [MoveIt 与 MoveIt Servo](#4-moveit-与-moveit-servo)
5. [RealSense 相机](#5-realsense-相机)
6. [OCS2 适配（deb 安装）](#6-ocs2-适配deb-安装)
7. [OCS2 控制启动](#7-ocs2-控制启动)
8. [MuJoCo 仿真](#8-mujoco-仿真)

---

## 1. 依赖安装

ROS2 换清华源: [清华大学开源软件镜像站 ROS2 软件仓库](https://mirror.tuna.tsinghua.edu.cn/help/ros2/)

```bash
git clone https://github.com/TDC-TangChaoBanLi/Dual_UR_Demo.git
cd Dual_UR_Demo
git submodule update --init --recursive

sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers -y
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-ros-control-interface -y
sudo apt-get install ros-${ROS_DISTRO}-ur -y
sudo apt install ros-${ROS_DISTRO}-librealsense2* -y
sudo apt install ros-${ROS_DISTRO}-realsense2-* -y
```

> 也可以直接运行 `./init_repo.sh` 由 rosdep 与 deb 安装自动完成。

测试 UR 驱动直连：(robot_ip 为机械臂IP地址, reverse_ip 为本机地址)

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.17 reverse_ip:=192.168.1.100
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.11 reverse_ip:=192.168.1.100
```

## 2. 编译本项目

```bash
# sudo apt install colcon -y
# colcon build --symlink-install --packages-select robotiq_driver robotiq_controllers robotiq_description
colcon build --symlink-install --packages-select dh_ag95_description dh_ag95_controllers
colcon build --symlink-install --packages-select lab_bench_description lab_bench_moveit_config launch_controller
colcon build --symlink-install --packages-select lab_bench_mujoco_description
colcon build --symlink-install --packages-select test_control
```

> 使用 `./quick_start.sh` 的「编译」选项会自动按依赖顺序编译当前环境相关包。

## 3. 真实机械臂启动流程

### 3.1 提取机械臂校准文件

```bash
source install/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.17 target_filename:="src/envs/lab_bench/lab_bench_description/config/ros2_control/ur_A_kinematics_calibration.yaml"
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.11 target_filename:="src/envs/lab_bench/lab_bench_description/config/ros2_control/ur_B_kinematics_calibration.yaml"
```

### 3.2 打开 CAN0 接口（DH-AG95 夹爪）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 3.3 启动 dashboard_client 与机械臂初始化

```bash
source install/setup.bash
ros2 launch launch_controller start_dual_ur_dashboard_client.launch.py
```

使用 dashboard_client 初始化机械臂：

```bash
source install/setup.bash
ros2 service call /arm_A/dashboard_client/load_program ur_dashboard_msgs/srv/Load "{filename: 'TDC-ROS2.urp'}" # 加载机器人上的程序
ros2 service call /arm_B/dashboard_client/load_program ur_dashboard_msgs/srv/Load "{filename: 'TDC-ROS2.urp'}" # 加载机器人上的程序

ros2 service call /arm_A/dashboard_client/power_on std_srvs/srv/Trigger {} # 给机器人电机上电，上电后还需要释放刹车
ros2 service call /arm_A/dashboard_client/brake_release std_srvs/srv/Trigger {} # 释放刹车
ros2 service call /arm_B/dashboard_client/power_on std_srvs/srv/Trigger {} # 给机器人电机上电，上电后还需要释放刹车
ros2 service call /arm_B/dashboard_client/brake_release std_srvs/srv/Trigger {} # 释放刹车
```

> 使用 `./quick_start.sh` 的「启动 dashboard_client」选项会自动执行以上初始化服务并确认结果。

### 3.4 启动 ros_control（真实硬件）

```bash
source install/setup.bash
ros2 launch launch_controller start_joints_controllers.launch.py use_fake_hardware:=false launch_rviz:=false
```

### 3.5 控制器启动后：重新启动机械臂程序

> **重要**：ros2_control 控制器连接机械臂后，机械臂上的 UR 程序会被停止/中断，
> 必须**重新启动程序**（先 `stop` 再 `play`），机械臂才会通过 socket 接收控制指令。

```bash
source install/setup.bash

# 控制器启动后执行（先 stop，再 play）
ros2 service call /arm_A/dashboard_client/stop std_srvs/srv/Trigger {} # 停止当前程序
ros2 service call /arm_B/dashboard_client/stop std_srvs/srv/Trigger {}
ros2 service call /arm_A/dashboard_client/play std_srvs/srv/Trigger {} # 重新启动程序（play 失败需检查）
ros2 service call /arm_B/dashboard_client/play std_srvs/srv/Trigger {}

ros2 service call /arm_A/dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning {} # 查询当前是否有程序正在运行
ros2 service call /arm_B/dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning {} # 查询当前是否有程序正在运行
```

> 使用 `./quick_start.sh` 时：
> 1. 先通过主菜单「启动 dashboard_client」选项完成机械臂初始化
>    （启动 dashboard_client + load_program / power_on / brake_release）；
> 2. 再启动控制器（真机模式）；控制器就绪后脚本自动执行 stop → play，play 失败会抛出错误。

### 3.6 关闭机械臂

```bash
source install/setup.bash

ros2 service call /arm_A/dashboard_client/power_off std_srvs/srv/Trigger {}  # 关闭机器人电机
ros2 service call /arm_B/dashboard_client/power_off std_srvs/srv/Trigger {}  # 关闭机器人电机

ros2 service call /arm_A/dashboard_client/shutdown std_srvs/srv/Trigger {} # 关闭机械臂控制柜
ros2 service call /arm_B/dashboard_client/shutdown std_srvs/srv/Trigger {} # 关闭机械臂控制柜
```

## 4. MoveIt 与 MoveIt Servo

### 4.1 启动 MoveIt move_group

```bash
source install/setup.bash

# 切换控制器
ros2 control switch_controllers --deactivate arm_A_ur_forward_position_controller --activate arm_A_ur_scaled_joint_trajectory_controller
ros2 control switch_controllers --deactivate arm_B_ur_forward_position_controller --activate arm_B_ur_scaled_joint_trajectory_controller

# 启动 move_group
ros2 launch launch_controller start_moveit_group.launch.py env:=lab_bench launch_rviz:=true
```

> `start_moveit_group.launch.py` 不认白名单：`env` 给任意已构建的 `{env}_moveit_config` 都行，
> 带 `_mujoco` 后缀的环境自动重定向到基环境（`space_sim_mujoco` → `space_sim_moveit_config`），
> 规划器（含 chomp / stomp）参数全部来自该包的 `config/planning_pipelines.yaml`；
> `lab_bench` 可用管线为 `ompl / stomp / chomp`（**默认使用 stomp**），可在 RViz 的 MotionPlanning 面板中切换。
> 每个 `*_moveit_config` 包只保留三种调试入口：`rviz2.launch.py`（只 RViz）、
> `demo.launch.py`（**只 move_group**，默认带 RViz）、`start_<env>_servo.launch.py`（只 Servo，目前仅 lab_bench）。

### 4.2 启动 MoveIt Servo

```bash
source install/setup.bash
# 切换控制器
ros2 control switch_controllers --deactivate arm_A_ur_scaled_joint_trajectory_controller --activate arm_A_ur_forward_position_controller
ros2 control switch_controllers --deactivate arm_B_ur_scaled_joint_trajectory_controller --activate arm_B_ur_forward_position_controller

# 启动 servo
ros2 launch lab_bench_moveit_config start_lab_bench_servo.launch.py
```

### 4.3 设置 Servo 模式

```bash
source install/setup.bash
# 设置 servo 模式为 position :
ros2 service call /arm_A_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
ros2 service call /arm_B_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
# 设置 servo 模式为 twist :
# ros2 service call /arm_A_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
# ros2 service call /arm_B_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"
# 设置 servo 模式为 joint :
# ros2 service call /arm_A_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"
# ros2 service call /arm_B_servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"
```

## 5. RealSense 相机

### 5.1 权限

```bash
sudo usermod -aG video $USER # 把当前用户加入 video 组
sudo reboot # 重启
groups # 查看权限组，应该有 video
```

### 5.2 启动相机

```bash
lsusb -t # 查看所有USB设备，确保 uvcvideo 设备的速度大于 500 Mbps
rs-enumerate-devices -s # 查看 RealSense 相机序列号 (Serial Number)
```

```bash
# 单独启动 arm_A 相机节点
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_401522071845 \
  camera_name:=arm_A \
  enable_depth:=true \
  enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true

# 单独启动 arm_B 相机节点
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_335222076295 \
  camera_name:=arm_B \
  enable_depth:=true \
  enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true

# 启动多相机节点
ros2 launch realsense2_camera rs_multi_camera_launch.py \
  serial_no1:=_401522071845 \
  serial_no2:=_335222076295 \
  camera_name1:=arm_A \
  camera_name2:=arm_B \
  camera_namespace1:=camera \
  camera_namespace2:=camera \
  base_frame_id1:=realsense_link \
  base_frame_id2:=realsense_link \
  enable_depth1:=true \
  enable_depth2:=true \
  enable_color1:=true \
  enable_color2:=true \
  depth_module.depth_profile1:=640x480x30 \
  depth_module.depth_profile2:=640x480x30 \
  rgb_camera.color_profile1:=640x480x30 \
  rgb_camera.color_profile2:=640x480x30 \
  align_depth.enable1:=true \
  align_depth.enable2:=true \
  pointcloud.enable1:=false \
  pointcloud.enable2:=false \
  spatial_filter.enable1:=true \
  spatial_filter.enable2:=true \
  temporal_filter.enable1:=true \
  temporal_filter.enable2:=true

ros2 launch realsense2_camera rs_multi_camera_launch.py \
  serial_no:=_115222071006 \
  camera_name:=global \
  camera_namespace:=camera \
  enable_depth:=true \
  enable_color:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true \
  unite_imu_method:=false
```

> `./quick_start.sh` 的「启动相机」选项会自动探测相机并支持按序号或 all 启动。

## 6. OCS2 适配（deb 安装）

下载 deb 包（按顺序安装，`ocs2_ros2` 为 OCS2 核心依赖，需先安装）：
- [ocs2_ros2 (legubiao)](https://github.com/legubiao/ocs2_ros2/releases/latest)
- [robot-descriptions-common](https://github.com/fiveages-sim/robot-descriptions-common/releases/latest)
- [arms_ros2_control](https://github.com/fiveages-sim/arms_ros2_control/releases/latest)

安装 deb 包及其依赖：
```bash
# 1. OCS2 核心库（ocs2_core / ocs2_mpc / ocs2_ros_interfaces 等）
sudo dpkg -i ros-jazzy-ocs2_*.deb

# 2. 机器人描述公共库（被 arms_ros2_control 依赖）
sudo dpkg -i ros-jazzy-robot-descriptions-common_*.deb

# 3. 双臂控制栈（ocs2_arm_controller / adaptive_gripper_controller /
#    arms_target_manager / arms_rviz_control_plugin 等）
sudo dpkg -i ros-jazzy-arms-ros2-control_*.deb

# sudo apt --fix-broken install # 若缺少依赖，先修复再重试
```

> 说明：`arms_ros2_control` 提供了本项目的 OCS2 双臂控制器（`ocs2_arm_controller`）、自适应夹爪控制器（`adaptive_gripper_controller`）、目标管理器（`arms_target_manager`，发布 RViz Interactive Markers）以及 RViz 插件（`arms_rviz_control_plugin`，GripperControlPanel / OCS2FSMPanel / JointControlPanel）。
>
> 也可以使用 `./init_repo.sh` 的「安装依赖」选项自动下载安装（支持固定版本 / latest / pre-release 通道）。

## 7. OCS2 控制启动

> 前置：已完成上述 OCS2 deb 安装，并编译本项目。

### 7.1 无硬件 / 真实硬件模式（ros2_control）

```bash
source install/setup.bash

# 仿真（默认 use_fake_hardware:=true，无需连接真实机械臂）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py

# 真实硬件（需先启动 dashboard_client 并 power_on / brake_release；
# 控制器启动后需重新 stop → play UR 程序，见 3.5 节）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py use_fake_hardware:=false
```

常用参数：
| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_fake_hardware` | `true` | 使用 mock 硬件（命令镜像到状态，无需真机） |
| `ur_headless_mode` | `false` | UR 驱动 headless 模式 |
| `launch_rviz` | `true` | 是否启动 RViz |
| `use_sim_time` | `false` | 同步仿真时钟（MuJoCo 场景由专用 launch 管理） |

### 7.2 MuJoCo 仿真模式（mujoco_ros2_control）

```bash
source install/setup.bash

# 启动 MuJoCo 仿真 + OCS2 控制器 + RViz（带 MuJoCo 视窗）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=lab_bench_mujoco

# 无头模式（不弹 MuJoCo 视窗，可配合 RViz 使用）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=lab_bench_mujoco mujoco_headless:=true
```

常用参数：
| 参数 | 默认值 | 说明 |
|---|---|---|
| `mujoco_headless` | `false` | 是否隐藏 MuJoCo Simulate 视窗 |
| `mujoco_sim_speed_factor` | `1.0` | 仿真速度倍率（1.0 = 实时） |
| `launch_rviz` | `true` | 是否启动 RViz |

### 7.3 操作方式（RViz）

启动后在 RViz 中：
- **FSM 面板**（OCS2FSMPanel）：切换 `HOME → HOLD → OCS2` 状态，进入 OCS2 后机械臂会向 marker 目标运动
- **拖动 marker**：`Left Arm Target`（左臂 / arm_B）与 `Right Arm Target`（右臂 / arm_A）为 6-DOF Interactive Marker，
  可拖拽设定双臂末端目标位姿（`arms_target_manager` 会将其发布到 OCS2 控制器）
- **夹爪面板**（GripperControlPanel）：左右夹爪的开/关按钮与位置（0~1）控制，分别对应 `left/right_adaptive_gripper_controller`

**发送目标的正确操作（重要）**：

1. 先用 FSM 面板切到 **OCS2 状态**——只有在 OCS2 / MOVEJ 状态下，拖动/发送目标才会被接收；
   HOLD / HOME 状态下发送的目标会被静默忽略（切到 OCS2 后控制器以当前位姿为初始目标，机械臂不会运动）
2. 拖动 `Left Arm Target` / `Right Arm Target` marker 到目标位置
3. **单击 marker 弹出菜单 → 点击「发送目标」**（默认单次模式，仅拖动不点菜单不会下发）；
   或先点击「切换到连续发布」，之后拖动即实时下发目标
4. 机械臂向目标运动；菜单里的「发送双臂」可同时发送左右臂目标

> 使用 `./quick_start.sh` 启动时会通过 Y/n 询问是否启动 RViz（默认启动）。

## 8. MuJoCo 仿真

MuJoCo 仿真基于 [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)，将 MuJoCo 作为 ros2_control 硬件接口运行物理仿真。

### 8.1 生成仿真资源

首次使用（或修改 URDF 后）需重新生成仿真模型：

```bash
# 1. 从 xacro 生成 urdf
xacro src/envs/lab_bench/lab_bench_mujoco_description/xacro/ros2_control/lab_bench_mujoco.xacro -o /tmp/lab_bench_mujoco_description.urdf

# 2. 用 urdf2mjcf 转换为 MuJoCo XML（需先安装 urdf2mjcf）
source ~/CodeProjects/urdf2mjcf/.venv/bin/activate
urdf2mjcf /tmp/lab_bench_mujoco_description.urdf \
  -o src/envs/lab_bench/lab_bench_mujoco_description/mjcf/lab_bench_mujoco.xml \
  -m src/envs/lab_bench/lab_bench_mujoco_description/meshes \
  -j src/envs/lab_bench/lab_bench_mujoco_description/config/lab_bench_config.json -c

# 3. 编译
colcon build --symlink-install --packages-select lab_bench_mujoco_description
```

### 8.2 启动仿真

```bash
source install/setup.bash

# 普通 ros2_control 控制（forward_position / joint_trajectory 等仿真控制器）
ros2 launch launch_controller start_joints_controllers.launch.py env:=lab_bench_mujoco

# OCS2 控制（见上方「OCS2 控制启动」第 2 节）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=lab_bench_mujoco
```

### 8.3 仿真演示（预置动作循环）

```bash
source install/setup.bash
ros2 run test_control test_mujoco_control
```
