cmake_minimum_required(VERSION 3.5)
project(space_sim_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz xacro config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()