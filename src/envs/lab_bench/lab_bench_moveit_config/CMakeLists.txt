cmake_minimum_required(VERSION 3.22)
project(lab_bench_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# launch/ 下只保留三种调试入口：rviz2.launch.py（仅 RViz）、demo.launch.py（仅 move_group）、
# start_lab_bench_servo.launch.py（仅 MoveIt Servo）。一般场景的 move_group 用统一入口：
#   ros2 launch launch_controller start_moveit_group.launch.py env:=lab_bench
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
