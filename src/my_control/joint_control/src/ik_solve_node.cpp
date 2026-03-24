#include "joint_control/ik_solve_node.hpp"
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

IkSolveNode::IkSolveNode() : Node("ik_solve"), ik_solver_()
{
    // 声明参数
    this->declare_parameter("target_pose_topic", "/target_pose");
    this->declare_parameter("current_pose_topic", "/current_pose");
    this->declare_parameter("robot_description_topic", "robot_description");

    // 读取参数
    this->get_parameter("target_pose_topic", target_pose_topic_);
    this->get_parameter("current_pose_topic", current_pose_topic_);
    this->get_parameter("robot_description_topic", robot_description_topic_);

    // 订阅目标位姿
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, 10, std::bind(&IkSolveNode::target_pose_callback, this, std::placeholders::_1));

    // 发布当前位姿
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic_, 10);

    // 发布解算结果到控制器命令话题
    arm_a_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_A_forward_position_controller/commands", 10);
    arm_b_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_B_forward_position_controller/commands", 10);

}

void IkSolveNode::initialize()
{
    // 加载模型
    RCLCPP_INFO(this->get_logger(), "Loading model from topic: %s", robot_description_topic_.c_str());
    ik_solver_.loadModel(shared_from_this(), robot_description_topic_);
    RCLCPP_INFO(this->get_logger(), "Model loaded with %zu links", ik_solver_.getLinkNames().size());

    // 注册规划组
    ik_solver_.registerPlanningGroup("ur_A");
    ik_solver_.registerPlanningGroup("ur_B");
    RCLCPP_INFO(this->get_logger(), "Planning groups registered: %zu", ik_solver_.getRegisteredPlanningGroups().size());

    // 启动状态监控
    ik_solver_.startJointStateMonitor("/joint_states");
    RCLCPP_INFO(this->get_logger(), "Joint state monitor started.");
}

void IkSolveNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 从 PoseStamped 消息中提取位置和姿态
    const auto &position = msg->pose.position;
    const auto &orientation = msg->pose.orientation;
    const auto &frame_id = msg->header.frame_id;

    if (std::find(ik_solver_.getLinkNames().begin(), ik_solver_.getLinkNames().end(), frame_id) == ik_solver_.getLinkNames().end())
    {
        RCLCPP_WARN(this->get_logger(), "Link %s not found in model, skipping IK solve", frame_id.c_str());
        return;
    }

    // 转换为 Eigen::Isometry3d
    Eigen::Isometry3d target_pose;
    target_pose.translation() = Eigen::Vector3d(position.x, position.y, position.z);
    target_pose.linear() = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();

    // 设置 IK 求解选项
    moveit_ik_solver::MoveItIkSettings ik_settings;
    ik_settings.timeout = 0.05;
    ik_settings.attempts = 5;
    ik_settings.verbose = true;


    // 根据 frame_id 的前缀选择对应的机械臂
    if (frame_id.substr(0, 5) == "arm_A")
    {

        // 求解 IK，将 frame_id 作为 link 名称
        moveit_ik_solver::MoveItIkResult result = ik_solver_.solveIk("ur_A", frame_id, "world", target_pose, ik_settings);

        // 发布解算结果到控制器命令话题
        std_msgs::msg::Float64MultiArray command_msg;
        if (result.success)
        {
            RCLCPP_INFO(this->get_logger(), "IK Solution for arm A (link: %s) Success.", frame_id.c_str());
            for (size_t i = 0; i < result.joint_angles.size(); ++i)
            {
                command_msg.data.push_back(result.joint_angles[i]);
            }
            arm_a_command_pub_->publish(command_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK Solution for arm A (link: %s) Failed.", frame_id.c_str());
        }
    }
    else if (frame_id.substr(0, 5) == "arm_B")
    {
        
        // 求解 IK，将 frame_id 作为 link 名称
        moveit_ik_solver::MoveItIkResult result = ik_solver_.solveIk("ur_B", frame_id, "world", target_pose, ik_settings);

        // 发布解算结果到控制器命令话题
        std_msgs::msg::Float64MultiArray command_msg;
        if (result.success)
        {
            RCLCPP_INFO(this->get_logger(), "IK Solution for arm B (link: %s) Success.", frame_id.c_str());
            for (size_t i = 0; i < result.joint_angles.size(); ++i)
            {
                command_msg.data.push_back(result.joint_angles[i]);
            }
            arm_b_command_pub_->publish(command_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK Solution for arm B (link: %s) Failed.", frame_id.c_str());
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown frame_id: %s, skipping IK solve", frame_id.c_str());
    }
}




int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IkSolveNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}