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
ros2 launch launch_controller start_traditional_controllers.launch.py env:=space_sim_mujoco initial_ur_controller:=forward_position_controller
```

## 环境说明：

- 固定方块 `base_cube`：0.6*0.6*0.6 m，300 kg，固定在 world 原点
- 正六边棱柱 `hex_prism`：对向边距 0.3 m，高 0.05 m，固定在方块顶面中心 (0, 0, 0.325)
- 双 UR 机械臂对侧安装：arm_A 在 +X 侧面，arm_B 在 -X 侧面
- 无夹爪、无相机（接口已预留，后续可添加）
- 重力为 0（太空环境）
- mjcf 中不包含地板（太空环境无地面）

## 配置预留：

- 背景贴图：`config/space_sim_config.json` 中 `worldbody.floor` / `worldbody.skybox_texture` 已预留，
  将 `add_json_floor` / `add_json_skybox` 改为 `true` 即可启用
- float 物体：`config/space_sim_config.json` 中 `worldbody.freejoint_body` 已预留，
  将需要自由漂浮的 body 名称加入列表即可自动添加 freejoint