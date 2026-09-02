cmake_minimum_required(VERSION 3.8)
project(test_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(COMMON_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(test_ros2_control
    src/test_ros2_control.cpp
)
target_include_directories(test_ros2_control PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(test_ros2_control
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    trajectory_msgs
    control_msgs
)
target_compile_features(test_ros2_control PUBLIC c_std_99 cxx_std_17)

add_executable(test_move_group
    src/test_move_group.cpp
)
target_include_directories(test_move_group PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(test_move_group
    rclcpp
    rcl_interfaces
    geometry_msgs
    moveit_ros_planning_interface
    tf2
    tf2_geometry_msgs
)
target_compile_features(test_move_group PUBLIC c_std_99 cxx_std_17)

add_executable(test_move_servo
    src/test_move_servo.cpp
)
target_include_directories(test_move_servo PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(test_move_servo
    rclcpp
    control_msgs
    geometry_msgs
    moveit_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)
target_compile_features(test_move_servo PUBLIC c_std_99 cxx_std_17)


add_executable(test_mujoco_control
    src/test_mujoco_control.cpp
)
target_include_directories(test_mujoco_control PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(test_mujoco_control
    rclcpp
    std_msgs
    trajectory_msgs
    control_msgs
    rclcpp_action
)
target_compile_features(test_mujoco_control PUBLIC c_std_99 cxx_std_17)

install(TARGETS
    test_ros2_control
    test_move_group
    test_move_servo
    test_mujoco_control
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
