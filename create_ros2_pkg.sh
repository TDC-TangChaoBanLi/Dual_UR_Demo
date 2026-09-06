#!/bin/bash

# Function to create C++ package structure
create_cpp_package() {
    local pkg_name=$1
    shift
    local nodes=("$@")
    
    # Create directories
    mkdir -p ${pkg_name}/src
    mkdir -p ${pkg_name}/include/${pkg_name}
    mkdir -p ${pkg_name}/launch
    
    # Create CMakeLists.txt
    cat > ${pkg_name}/CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 3.8)
project(${pkg_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

EOF

    # Add nodes to CMakeLists.txt
    for node in "${nodes[@]}"; do
        cat >> ${pkg_name}/CMakeLists.txt <<EOF
# ${node} executable
add_executable(${node} 
    src/${node}.cpp
)
target_include_directories(${node} PUBLIC
    \$<BUILD_INTERFACE:\${CMAKE_CURRENT_SOURCE_DIR}/include>
    \$<INSTALL_INTERFACE:include>)
ament_target_dependencies(${node}
    rclcpp
    # std_msgs
)
target_compile_features(${node} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
EOF
    done

    # Add installation to CMakeLists.txt
    cat >> ${pkg_name}/CMakeLists.txt <<EOF
# Install targets
install(TARGETS ${nodes[@]}
    DESTINATION lib/\${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/\${PROJECT_NAME}/launch)

ament_package()
EOF

    # Create package.xml
    cat > ${pkg_name}/package.xml <<EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>${pkg_name}</name>
    <version>0.0.0</version>
    <description>${pkg_name} package</description>
    <maintainer email="user@email.com">Your Name</maintainer>
    <license>Apache License 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>rclcpp</depend>
    <!-- <depend>std_msgs</depend> -->
    
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
EOF

    # Create node source files and launch file
    for node in "${nodes[@]}"; do
        # Create .cpp file
        cat > ${pkg_name}/src/${node}.cpp <<EOF
#include "rclcpp/rclcpp.hpp"

class ${node^} : public rclcpp::Node {
public:
    ${node^}() : Node("${node}") {
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->timer_callback(); });
    }

private:
    void timer_callback() {
        RCLCPP_INFO(get_logger(), "Hello from ${node}!");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<${node^}>());
    rclcpp::shutdown();
    return 0;
}
EOF

        # Create header file
        cat > ${pkg_name}/include/${pkg_name}/${node}.hpp <<EOF
#pragma once

// Add your header content here

EOF
    done

    # Create launch file
    cat > ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
EOF

    for node in "${nodes[@]}"; do
        cat >> ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
        Node(
            package='${pkg_name}',
            executable='${node}',
            name='${node}',
            output='screen'
        ),
EOF
    done

    cat >> ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
    ])
EOF

    echo "Created C++ package '${pkg_name}' with nodes: ${nodes[@]}"
}

# Function to create Python package structure
create_python_package() {
    local pkg_name=$1
    shift
    local nodes=("$@")
    
    # Create directories
    mkdir -p ${pkg_name}/${pkg_name}
    mkdir -p ${pkg_name}/resource
    mkdir -p ${pkg_name}/launch

    # Create resource file
    cat > ${pkg_name}/resource/${pkg_name} <<EOF

EOF
    
    # Create package.xml
    cat > ${pkg_name}/package.xml <<EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>${pkg_name}</name>
    <version>0.0.0</version>
    <description>${pkg_name} package</description>
    <maintainer email="user@email.com">Your Name</maintainer>
    <license>Apache License 2.0</license>

    <buildtool_depend>ament_python</buildtool_depend>
    <depend>rclpy</depend>
    
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_python</build_type>
    </export>
</package>
EOF

    # Create setup.py
    cat > ${pkg_name}/setup.py <<EOF
from setuptools import setup

package_name = '${pkg_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/${pkg_name}.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@email.com',
    description='${pkg_name} package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
EOF

    for node in "${nodes[@]}"; do
        cat >> ${pkg_name}/setup.py <<EOF
            '${node} = ${pkg_name}.${node}:main',
EOF
    done

    cat >> ${pkg_name}/setup.py <<EOF
        ],
    },
)
EOF

    # Create setup.cfg
    cat > ${pkg_name}/setup.cfg <<EOF
[develop]
script_dir=\$base/lib/${pkg_name}
[install]
install_scripts=\$base/lib/${pkg_name}
EOF

    # Create __init__.py
    touch ${pkg_name}/${pkg_name}/__init__.py

    # Create node files and launch file
    for node in "${nodes[@]}"; do
        # Create .py file
        cat > ${pkg_name}/${pkg_name}/${node}.py <<EOF
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ${node^}(Node):
    def __init__(self):
        super().__init__('${node}')
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Hello from ${node}!')

def main(args=None):
    rclpy.init(args=args)
    node = ${node^}()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
        chmod +x ${pkg_name}/${pkg_name}/${node}.py
    done

    # Create launch file
    cat > ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
EOF

    for node in "${nodes[@]}"; do
        cat >> ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
        Node(
            package='${pkg_name}',
            executable='${node}',
            name='${node}',
            output='screen'
        ),
EOF
    done

    cat >> ${pkg_name}/launch/${pkg_name}.launch.py <<EOF
    ])
EOF

    echo "Created Python package '${pkg_name}' with nodes: ${nodes[@]}"
}

# Main script
if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <package_name> <cpp|python> <node1> [node2 ...]"
    exit 1
fi

pkg_name=$1
shift
pkg_type=$1
shift
nodes=("$@")

# Create workspace directory
case $pkg_type in
    cpp)
        create_cpp_package $pkg_name "${nodes[@]}"
        ;;
    python)
        create_python_package $pkg_name "${nodes[@]}"
        ;;
    *)
        echo "Invalid package type: $pkg_type (must be 'cpp' or 'python')"
        exit 1
        ;;
esac

cd ..
echo "ROS2 workspace created in ${pkg_name}"