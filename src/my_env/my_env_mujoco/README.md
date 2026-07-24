


## 部署：

生成 urdf 文件：

```bash
xacro src/my_env/my_env_mujoco/urdf/my_env_mujoco.urdf.xacro -o src/my_env/my_env_mujoco/urdf/my_env_mujoco.urdf
```

转换为 mujoco xml 文件：
```bash
source ~/CodeProjects/urdf2mjcf/.venv/bin/activate # 激活 urdf2mjcf 的 python 环境
urdf2mjcf src/my_env/my_env_mujoco/urdf/my_env_mujoco.urdf -o src/my_env/my_env_mujoco/mjcf/my_env_mujoco.xml -m src/my_env/my_env_mujoco/meshes -j src/my_env/my_env_mujoco/config/my_env_config.json -c
```

