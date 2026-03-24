cmake_minimum_required(VERSION 3.8)
project(joint_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# mpc_ctrl executable
add_executable(mpc_ctrl 
    src/mpc_ctrl.cpp
)
target_include_directories(mpc_ctrl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mpc_ctrl
    rclcpp
    # std_msgs
)
target_compile_features(mpc_ctrl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ik_solve executable
add_executable(ik_solve 
    src/ik_solve_node.cpp
    # src/pinocchio_ik_solver.cpp
    src/moveit_ik_solver.cpp
)
target_include_directories(ik_solve PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ik_solve
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pinocchio
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
)
target_compile_features(ik_solve PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# Install targets
install(TARGETS mpc_ctrl ik_solve
    DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()