

## 依赖：

[GitHub: urdf2mjcf](https://github.com/TDC-TangChaoBanLi/urdf2mjcf)

[GitHub: mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control.git)



## 部署：

生成 urdf 文件：

```bash
xacro src/envs/lab_bench/lab_bench_mujoco_description/xacro/ros2_control/lab_bench_mujoco.xacro -o /tmp/lab_bench_mujoco.urdf
```

转换为 mujoco xml 文件：
```bash
source ~/CodeProjects/urdf2mjcf/.venv/bin/activate # 激活 urdf2mjcf 的 python 环境
urdf2mjcf /tmp/lab_bench_mujoco.urdf -o src/envs/lab_bench/lab_bench_mujoco_description/mjcf/lab_bench_mujoco.xml -m src/envs/lab_bench/lab_bench_mujoco_description/meshes -j src/envs/lab_bench/lab_bench_mujoco_description/config/lab_bench_config.json -c
```

编译：
```bash
colcon build --symlink-install --packages-select lab_bench_mujoco_description
```

## 启动：

```bash
source ./install/setup.bash
ros2 launch launch_controller start_traditional_controllers.launch.py env:=lab_bench_mujoco
```

