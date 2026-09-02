cmake_minimum_required(VERSION 3.5)
project(lab_bench_mujoco_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  xacro
  mjcf
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
