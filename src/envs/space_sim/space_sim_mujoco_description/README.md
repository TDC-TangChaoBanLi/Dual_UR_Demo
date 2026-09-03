## 依赖：

[GitHub: urdf2mjcf](https://github.com/TDC-TangChaoBanLi/urdf2mjcf)

[GitHub: mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control.git)



## 部署：

生成 urdf 文件：

```bash
xacro src/envs/space_sim/space_sim_mujoco_description/xacro/ros2_control/space_sim_mujoco.xacro -o /tmp/space_sim_mujoco.urdf
```

转换为 mujoco xml 文件：
```bash
source ~/CodeProjects/urdf2mjcf/.venv/bin/activate # 激活 urdf2mjcf 的 python 环境
urdf2mjcf /tmp/space_sim_mujoco.urdf -o src/envs/space_sim/space_sim_mujoco_description/mjcf/space_sim_mujoco.xml -m src/envs/space_sim/space_sim_mujoco_description/meshes -j src/envs/space_sim/space_sim_mujoco_description/config/space_sim_config.json -c
```

编译：
```bash
colcon build --symlink-install --packages-select space_sim_mujoco_description
```

## 启动：

```bash
source ./install/setup.bash
# 传统控制器（joint_trajectory / forward_position 等）
ros2 launch launch_controller start_traditional_controllers.launch.py env:=space_sim_mujoco initial_ur_controller:=forward_position_controller

# OCS2 双臂控制器（无夹爪，Interactive Marker 基座帧为 base_cube）
ros2 launch launch_controller start_ocs2_arms_controller.launch.py env:=space_sim_mujoco mujoco_headless:=true launch_rviz:=false
```

## 环境说明：

- 固定方块 `base_cube`：0.6*0.6*0.6 m，300 kg，固定在 world 原点
- 正六边棱柱 `hex_prism`：对向边距 0.3 m，高 0.05 m，固定在方块顶面中心 (0, 0, 0.325)
- 双 UR 机械臂对侧安装：arm_A 在 +X 侧面，arm_B 在 -X 侧面
- 双臂末端挂载 RealSense D435 相机及支架（视觉模型，无夹爪）
- 重力为 0（太空环境）
- mjcf 中不包含地板（太空环境无地面）

## ros2_control 配置文件：

| 文件 | 说明 |
|---|---|
| `xacro/ros2_control/space_sim_mujoco.xacro` | mujoco ros2_control 入口（MujocoSystemInterface） |
| `xacro/ros2_control/space_sim_arm_mujoco_control.xacro` | 单臂 joint/sensor 接口宏（6 关节 + FTS，无夹爪） |
| `config/ros2_control/traditional_controllers.yaml` | 传统控制器（joint_state_broadcaster + 双臂运动/状态控制器） |
| `config/ros2_control/ocs2_arms_controllers.yaml` | OCS2 双臂控制器（无夹爪） |
| `config/ocs2/task.info` | OCS2 任务配置（OCS2 控制器按 robot_name 在此包内查找） |
| `config/mujoco_pids_config.yaml` | 12 个 UR 关节 PID 增益 |
| `config/mujoco_ros2_control_plugins.yaml` | mujoco 插件（heartbeat，相机插件预留） |
| `mjcf/space_sim_mujoco.xml` | MuJoCo 场景（含 RealSense 相机视觉模型） |

## 配置预留：

- 背景贴图：`config/space_sim_config.json` 中 `worldbody.floor` / `worldbody.skybox_texture` 已预留，
  将 `add_json_floor` / `add_json_skybox` 改为 `true` 即可启用
- float 物体：`config/space_sim_config.json` 中 `worldbody.freejoint_body` 已预留，
  将需要自由漂浮的 body 名称加入列表即可自动添加 freejoint
- 夹爪：`space_sim_description/xacro/dual_arm_config.xacro` 中 `gripper_type` 改为 `dh_ag95` / `robotiq_2f85`，
  并在 `space_sim_arm_macro.xacro` 预留位置实例化夹爪宏；同时补充对应控制器配置