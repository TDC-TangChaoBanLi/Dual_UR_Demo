# 双臂UR机械臂ROS2控制系统

这是一个基于ROS2的双臂UR机械臂控制系统，集成了UR5和UR5e机械臂、Robotiq 2F-85夹爪和RealSense D435相机，目前支持rviz仿真。

![intrudoction](view_my_env.png)

## 项目结构概述

本项目主要包括以下几个关键组件：

1. **机械臂描述文件** - 定义双臂系统的URDF模型
2. **ros_control控制器配置** - 实现硬件接口和控制器管理
3. **MoveIt运动规划配置** - 提供高级运动规划功能

---

## 1. 机械臂描述文件系统

### 创建的文件及其作用

| 文件路径 | 作用 |
|---------|------|
| `my_env_description/urdf/my_ur_macro.urdf.xacro` | 定义单臂系统(UR+夹爪+相机)的Xacro宏 |
| `my_env_description/urdf/my_env_macro.urdf.xacro` | 定义双臂环境(含桌子、安装座等)的Xacro宏 |
| `my_env_description/urdf/my_env.urdf.xacro` | 主描述文件，整合所有组件并暴露参数接口 |
| `my_env_description/launch/view_my_env.launch.py` | 可视化查看机器人模型的启动文件 |

### 工作原理

通过Xacro宏定义实现了模块化的机器人描述:
- 单臂模块(`my_arm`)集成了UR机械臂、Robotiq夹爪和RealSense相机
- 环境模块(`my_env`)定义了共享工作台和双臂安装位置
- 使用tf_prefix区分左右臂的坐标变换

### 启动查看

```bash
ros2 launch my_env_description view_my_env.launch.py
```

---

## 2. ros_control控制器系统

### 创建的文件及其作用

| 文件路径 | 作用 |
|---------|------|
| `my_env_control/urdf/my_env_control.urdf.xacro` | 控制专用的机器人描述文件，启用了ros2_control |
| `my_env_control/launch/my_ur_control.launch.py` | 修改版UR控制器启动文件，支持命名空间隔离 |
| `my_env_control/config/my_ur_controllers.yaml` | UR控制器配置文件，支持相对路径引用 |
| `my_env_control/urdf/robotiq_2f_85_gripper_control.urdf.xacro` | 夹爪控制专用描述文件 |
| `my_env_control/launch/my_robotiq_control.launch.py` | 夹爪控制器启动文件 |
| `my_env_control/config/my_robotiq_controllers.yaml` | 夹爪控制器配置文件 |
| `my_env_control/launch/start_my_env_control.launch.py` | 总控制器启动文件，统一管理双臂系统 |

### 修改的文件及原因

1. **控制器启动文件修改**
   - 将`/controller_manager`绝对路径改为`controller_manager`相对路径
   - 移除`robot_state_publisher`和`rviz`节点，避免重复启动
   - 添加命名空间支持，实现多机械臂隔离控制

2. **控制器配置文件修改**
   - 添加`/*:`标识符支持相对路径引用
   - 为关节名称添加`$(var tf_prefix)`前缀，区分不同机械臂

### 控制示例代码

创建了`my_env_ros2_control`节点用于演示基本控制:

```cpp
// 发布关节轨迹到机械臂
auto arm_a_traj = trajectory_msgs::msg::JointTrajectory();
// 控制夹爪开合
send_goal_to_gripper(gripper_a_client_, 0.8, 50.0);
```

### 启动控制

```bash
# 启动双臂控制系统
ros2 launch my_env_control start_my_env_control.launch.py

# 启动控制示例节点
ros2 run my_env_control my_env_ros2_control
```

---

## 3. MoveIt运动规划系统

### 创建的文件及其作用

| 文件路径 | 作用 |
|---------|------|
| `my_env_moveit_config/config/*.yaml` | MoveIt各类配置文件(joint_limits, kinematics, ompl等) |
| `my_env_moveit_config/config/my_env_controlled.srdf` | 机器人语义描述文件(通过Setup Assistant生成) |
| `my_env_moveit_config/launch/my_env_moveit.launch.py` | MoveIt系统启动文件 |

### 关键配置说明

1. **规划组设置**
   - `ur_A`, `ur_B`: 分别对应左右臂
   - 使用KDL运动学求解器

2. **控制器配置**
   ```yaml
   controller_names:
     - ur_A/scaled_joint_trajectory_controller
     - robotiq_A/robotiq_gripper_controller
   ```

3. **关节限制**
   ```yaml
   joint_limits:
     arm_A_ur_shoulder_pan_joint:
       max_velocity: 1.5708
       max_acceleration: 0.7854
   ```

### 启动MoveIt

```bash
# 首先启动底层控制器
ros2 launch my_env_control start_my_env_control.launch.py launch_rviz:=false

# 然后启动MoveIt规划系统
ros2 launch my_env_moveit_config my_env_moveit.launch.py
```

---

## 使用流程

1. **查看模型**:
   ```bash
   ros2 launch my_env_description view_my_env.launch.py
   ```

2. **启动控制器**:
   ```bash
   ros2 launch my_env_control start_my_env_control.launch.py
   ```

3. **启动MoveIt**:
   ```bash
   ros2 launch my_env_moveit_config my_env_moveit.launch.py
   ```
