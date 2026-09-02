#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class DualArmController : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    DualArmController() : Node("test_mujoco_control")
    {
        // 话题/动作名参数化（默认适配 lab_bench 双臂 forward_position_controller）
        arm_a_position_topic_ = this->declare_parameter<std::string>(
            "arm_a_position_topic", "/arm_A_forward_position_controller/commands");
        arm_b_position_topic_ = this->declare_parameter<std::string>(
            "arm_b_position_topic", "/arm_B_forward_position_controller/commands");
        arm_a_gripper_action_ = this->declare_parameter<std::string>(
            "arm_a_gripper_action", "/arm_A_robotiq_gripper_controller/gripper_cmd");
        arm_b_gripper_action_ = this->declare_parameter<std::string>(
            "arm_b_gripper_action", "/arm_B_robotiq_gripper_controller/gripper_cmd");

        // 创建发布者用于控制两个机械臂
        arm_A_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            arm_a_position_topic_, 10);
            
        arm_B_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            arm_b_position_topic_, 10);
            
        // 创建动作客户端用于控制两个Robotiq夹爪
        gripper_A_client_ = rclcpp_action::create_client<GripperCommand>(
            this, arm_a_gripper_action_);
            
        gripper_B_client_ = rclcpp_action::create_client<GripperCommand>(
            this, arm_b_gripper_action_);
            
        // 定时器，定期发送轨迹命令
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&DualArmController::send_trajectories, this));
            
        arm_A_pos_cmd_ = std_msgs::msg::Float64MultiArray();
        arm_B_pos_cmd_ = std_msgs::msg::Float64MultiArray();

        
        // 初始化动作计数器
        action_step_ = 0;
            
        RCLCPP_INFO(this->get_logger(), "test_mujoco_control node initialized");
        RCLCPP_INFO(this->get_logger(), "arm_a_position_topic=%s arm_b_position_topic=%s",
                    arm_a_position_topic_.c_str(), arm_b_position_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "arm_a_gripper_action=%s arm_b_gripper_action=%s",
                    arm_a_gripper_action_.c_str(), arm_b_gripper_action_.c_str());
    }

private:
    void send_trajectories()
    {
        // 根据action_step循环执行不同动作
        switch(action_step_ % 3) {
            case 0:
                send_init_positions();
                break;
            case 1:
                send_home_positions();
                break;
            case 2:
                send_goal_to_gripper(gripper_A_client_, 0.8);
                send_goal_to_gripper(gripper_B_client_, 0.8);
                break;
        }
        
        action_step_++;
        RCLCPP_INFO(this->get_logger(), "Executing action step: %d", action_step_ % 3);
    }
    
    void send_goal_to_gripper(
        rclcpp_action::Client<GripperCommand>::SharedPtr client,
        double position,
        double effort = 10.0)
    {
        if (!client->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
            return;
        }

        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = effort;

        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](GoalHandleGripperCommand::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };

        client->async_send_goal(goal_msg, send_goal_options);
    }
    
    void send_init_positions()
    {
        // 为Arm A创建关节状态消息 - 初始位置
        arm_A_pos_cmd_.data = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
        
        // 为Arm B创建关节状态消息 - 初始位置
        arm_B_pos_cmd_.data = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
        
        // 发布关节位置命令
        arm_A_publisher_->publish(arm_A_pos_cmd_);
        arm_B_publisher_->publish(arm_B_pos_cmd_);
        
        // 打开夹爪 - 发送动作目标
        send_goal_to_gripper(gripper_A_client_, 0.0); // 0.0表示完全打开
        send_goal_to_gripper(gripper_B_client_, 0.0); // 0.0表示完全打开
        
        RCLCPP_INFO(this->get_logger(), "Sent INIT positions: arms to init, grippers opened");
        rclcpp::sleep_for(5s);
    }

    void send_home_positions()
    { 
        // 为Arm A创建关节状态消息 - 初始位置
        arm_A_pos_cmd_.data = {2.3562, -2.3562, 2.3562, -1.5708, -1.5708, 0.0};
        
        // 为Arm B创建关节状态消息 - 初始位置
        arm_B_pos_cmd_.data = {-2.3562, -0.7854, -2.3562, -1.5708, 1.5708, 0.0};

        // 发布关节位置命令
        arm_A_publisher_->publish(arm_A_pos_cmd_);
        arm_B_publisher_->publish(arm_B_pos_cmd_);
        
        // 打开夹爪 - 发送动作目标
        send_goal_to_gripper(gripper_A_client_, 0.0); // 0.0表示完全打开
        send_goal_to_gripper(gripper_B_client_, 0.0); // 0.0表示完全打开
        
        RCLCPP_INFO(this->get_logger(), "Sent HOME positions: arms to home, grippers opened");
        rclcpp::sleep_for(5s); // 等待2秒以确保动作完成
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_A_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_B_publisher_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_A_client_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_B_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float64MultiArray arm_A_pos_cmd_;
    std_msgs::msg::Float64MultiArray arm_B_pos_cmd_;
    int action_step_;
    std::string arm_a_position_topic_;
    std::string arm_b_position_topic_;
    std::string arm_a_gripper_action_;
    std::string arm_b_gripper_action_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmController>());
    rclcpp::shutdown();
    return 0;
}