cmake_minimum_required(VERSION 3.8)
project(my_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)




# ur_moveit_control executable
add_executable(ur_moveit_control 
    src/ur_moveit_control.cpp
)
target_include_directories(ur_moveit_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ur_moveit_control
    rclcpp
    # std_msgs
    moveit_ros_planning_interface
)
target_compile_features(ur_moveit_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# my_env_ros2_control executable
add_executable(my_env_ros2_control 
    src/my_env_ros2_control.cpp
)
target_include_directories(my_env_ros2_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(my_env_ros2_control
    rclcpp
    # std_msgs
    trajectory_msgs
    control_msgs
    rclcpp_action
)
target_compile_features(my_env_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# my_env_moveit_control executable
add_executable(my_env_moveit_control 
    src/my_env_moveit_control.cpp
)
target_include_directories(my_env_moveit_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(my_env_moveit_control
    rclcpp
    # std_msgs
)
target_compile_features(my_env_moveit_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17





# Install targets
install(TARGETS 
    ur_moveit_control 
    my_env_ros2_control 
    my_env_moveit_control
    DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
