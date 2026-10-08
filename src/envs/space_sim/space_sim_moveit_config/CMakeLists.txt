cmake_minimum_required(VERSION 3.22)
project(space_sim_moveit_config)

find_package(ament_cmake REQUIRED)

# launch/ 下只保留两种调试入口：rviz2.launch.py（仅 RViz）、demo.launch.py（仅 move_group）。
# 一般场景的 move_group 用统一入口：
#   ros2 launch launch_controller start_moveit_group.launch.py env:=space_sim
# space_sim 不需要 MoveIt Servo。
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
