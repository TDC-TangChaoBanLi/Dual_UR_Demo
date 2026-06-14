# 保留你原来的 bash 配置
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
fi

# 自动 source 当前工作区
if [ -f ./install/setup.bash ]; then
    source ./install/setup.bash
    echo "[ROS2] sourced ./install/setup.bash"
elif [ -f ./install/setup.sh ]; then
    source ./install/setup.sh
    echo "[ROS2] sourced ./install/setup.sh"
else
    echo "[ROS2] install/setup.bash not found. Please build first."
fi