cmake_minimum_required(VERSION 3.5)
project(assembly_exp_mujoco_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  xacro
  mjcf
  config
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()