#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <map>

#include "joint_control/moveit_ik_solver.hpp"

class IkSolveNode : public rclcpp::Node {
public:
    IkSolveNode();
    ~IkSolveNode() = default;
    void initialize();

private:
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_current_pose();

    // ROS 相关参数
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_a_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_b_command_pub_;

    // IK 求解器
    moveit_ik_solver::MoveItIkSolver ik_solver_;
    
    // 参数
    std::string target_pose_topic_;
    std::string current_pose_topic_;
    std::string robot_description_topic_;

    // 各个规划组的关节名称和对应目标关节角度和当前关节角度
    std::map<std::string, std::map<std::string, double>> joint_target_angles_;
    std::map<std::string, std::map<std::string, double>> joint_current_angles_;
};
