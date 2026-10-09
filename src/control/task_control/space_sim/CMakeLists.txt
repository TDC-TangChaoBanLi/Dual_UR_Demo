cmake_minimum_required(VERSION 3.8)
project(space_sim_task_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(multi_arm_motion_interfaces REQUIRED)
find_package(multi_arm_motion_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco_god_plugin REQUIRED)
find_package(mujoco_dock_sim_plugin REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/DockStorage.action"
    "action/ExitStorage.action"
    "action/MoveLens.action"
    "action/DockCentre.action"
    "msg/LensInitialState.msg"
    "srv/ResetPlanningEnvironment.srv"
    DEPENDENCIES geometry_msgs sensor_msgs
)

set(COMMON_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(space_sim_task_server
    src/task_server.cpp
    src/site_pose_provider.cpp
    src/dock_client.cpp
    src/scene_manager.cpp
    src/moveit_params.cpp
    src/arm_planner.cpp
    src/mesh_loader.cpp
    src/assembly_targets.cpp
    src/motion_bridge.cpp
)
target_include_directories(space_sim_task_server PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(space_sim_task_server
    Eigen3::Eigen
)
ament_target_dependencies(space_sim_task_server
    rclcpp
    rclcpp_action
    multi_arm_motion_interfaces
    multi_arm_motion_core
    rcl_interfaces
    geometry_msgs
    sensor_msgs
    shape_msgs
    trajectory_msgs
    control_msgs
    std_srvs
    moveit_msgs
    moveit_ros_planning_interface
    moveit_core
    tf2
    tf2_ros
    tf2_geometry_msgs
    ament_index_cpp
    mujoco_god_plugin
    mujoco_dock_sim_plugin
    mujoco_ros2_control_msgs
    controller_manager_msgs
)
target_compile_features(space_sim_task_server PUBLIC cxx_std_17)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(space_sim_task_server "${cpp_typesupport_target}")

install(TARGETS space_sim_task_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/assembly_flow.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    add_executable(space_sim_audit_time_parameterize test/audit_time_parameterize.cpp src/moveit_params.cpp)
    target_include_directories(space_sim_audit_time_parameterize PRIVATE ${COMMON_INCLUDE_DIRS})
    ament_target_dependencies(space_sim_audit_time_parameterize
        rclcpp rcl_interfaces moveit_msgs moveit_ros_planning_interface moveit_ros_planning moveit_core)
    target_compile_features(space_sim_audit_time_parameterize PUBLIC cxx_std_17)
    install(TARGETS space_sim_audit_time_parameterize DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
