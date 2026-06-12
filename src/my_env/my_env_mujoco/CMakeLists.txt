cmake_minimum_required(VERSION 3.5)
project(my_env_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  # launch
  meshes
  urdf
  mjcf
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
