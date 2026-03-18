#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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

    DualArmController() : Node("my_env_ros2_control")
    {
        // 创建发布者用于控制两个机械臂
        arm_a_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/ur_A/joint_trajectory_controller/joint_trajectory", 10);
            
        arm_b_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/ur_B/joint_trajectory_controller/joint_trajectory", 10);
            
        // 创建动作客户端用于控制两个Robotiq夹爪
        gripper_a_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/robotiq_A/robotiq_gripper_controller/gripper_cmd");
            
        gripper_b_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/robotiq_B/robotiq_gripper_controller/gripper_cmd");
            
        // 定时器，定期发送轨迹命令
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&DualArmController::send_trajectories, this));
            
        // 初始化动作计数器
        action_step_ = 0;
            
        RCLCPP_INFO(this->get_logger(), "my_env_ros2_control node initialized");
    }

private:
    void send_trajectories()
    {
        // 根据action_step循环执行不同动作
        switch(action_step_ % 3) {
            case 0:
                send_home_positions();
                break;
            case 1:
                send_gripping_positions();
                break;
            case 2:
                send_place_positions();
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
            [](GoalHandleGripperCommand::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_WARN(rclcpp::get_logger("dual_arm_controller"), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("dual_arm_controller"), "Goal accepted by server, waiting for result");
                }
            };

        client->async_send_goal(goal_msg, send_goal_options);
    }
    
    void send_home_positions()
    {
        // 为Arm A创建关节轨迹消息 - 初始位置
        auto arm_a_traj = trajectory_msgs::msg::JointTrajectory();
        arm_a_traj.joint_names = {
            "arm_A_ur_shoulder_pan_joint",
            "arm_A_ur_shoulder_lift_joint",
            "arm_A_ur_elbow_joint",
            "arm_A_ur_wrist_1_joint",
            "arm_A_ur_wrist_2_joint",
            "arm_A_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_a;
        point_a.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
        point_a.time_from_start.sec = 2;
        arm_a_traj.points.push_back(point_a);
        
        // 为Arm B创建关节轨迹消息 - 初始位置
        auto arm_b_traj = trajectory_msgs::msg::JointTrajectory();
        arm_b_traj.joint_names = {
            "arm_B_ur_shoulder_pan_joint",
            "arm_B_ur_shoulder_lift_joint",
            "arm_B_ur_elbow_joint",
            "arm_B_ur_wrist_1_joint",
            "arm_B_ur_wrist_2_joint",
            "arm_B_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_b;
        point_b.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
        point_b.time_from_start.sec = 2;
        arm_b_traj.points.push_back(point_b);
        
        // 发布轨迹命令
        arm_a_publisher_->publish(arm_a_traj);
        arm_b_publisher_->publish(arm_b_traj);
        
        // 打开夹爪 - 发送动作目标
        send_goal_to_gripper(gripper_a_client_, 0.0); // 0.0表示完全打开
        send_goal_to_gripper(gripper_b_client_, 0.0); // 0.0表示完全打开
        
        RCLCPP_INFO(this->get_logger(), "Sent HOME positions: arms to home, grippers opened");
    }
    
    void send_gripping_positions()
    {
        // 为Arm A创建关节轨迹消息 - 抓取位置
        auto arm_a_traj = trajectory_msgs::msg::JointTrajectory();
        arm_a_traj.joint_names = {
            "arm_A_ur_shoulder_pan_joint",
            "arm_A_ur_shoulder_lift_joint",
            "arm_A_ur_elbow_joint",
            "arm_A_ur_wrist_1_joint",
            "arm_A_ur_wrist_2_joint",
            "arm_A_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_a;
        point_a.positions = {-0.5, -1.0, 1.8, -0.8, -1.57, 0.5};
        point_a.time_from_start.sec = 2;
        arm_a_traj.points.push_back(point_a);
        
        // 为Arm B创建关节轨迹消息 - 抓取位置
        auto arm_b_traj = trajectory_msgs::msg::JointTrajectory();
        arm_b_traj.joint_names = {
            "arm_B_ur_shoulder_pan_joint",
            "arm_B_ur_shoulder_lift_joint",
            "arm_B_ur_elbow_joint",
            "arm_B_ur_wrist_1_joint",
            "arm_B_ur_wrist_2_joint",
            "arm_B_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_b;
        point_b.positions = {0.5, -1.0, 1.8, -0.8, -1.57, -0.5};
        point_b.time_from_start.sec = 2;
        arm_b_traj.points.push_back(point_b);
        
        // 发布轨迹命令
        arm_a_publisher_->publish(arm_a_traj);
        arm_b_publisher_->publish(arm_b_traj);
        
        // 闭合夹爪 - 发送动作目标
        send_goal_to_gripper(gripper_a_client_, 0.8, 50.0); // 0.8表示较大程度闭合
        send_goal_to_gripper(gripper_b_client_, 0.8, 50.0); // 0.8表示较大程度闭合
        
        RCLCPP_INFO(this->get_logger(), "Sent GRIPPING positions: arms to target, grippers closed");
    }
    
    void send_place_positions()
    {
        // 为Arm A创建关节轨迹消息 - 放置位置
        auto arm_a_traj = trajectory_msgs::msg::JointTrajectory();
        arm_a_traj.joint_names = {
            "arm_A_ur_shoulder_pan_joint",
            "arm_A_ur_shoulder_lift_joint",
            "arm_A_ur_elbow_joint",
            "arm_A_ur_wrist_1_joint",
            "arm_A_ur_wrist_2_joint",
            "arm_A_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_a;
        point_a.positions = {1.0, -0.8, 1.5, -0.7, -1.57, -1.0};
        point_a.time_from_start.sec = 2;
        arm_a_traj.points.push_back(point_a);
        
        // 为Arm B创建关节轨迹消息 - 放置位置
        auto arm_b_traj = trajectory_msgs::msg::JointTrajectory();
        arm_b_traj.joint_names = {
            "arm_B_ur_shoulder_pan_joint",
            "arm_B_ur_shoulder_lift_joint",
            "arm_B_ur_elbow_joint",
            "arm_B_ur_wrist_1_joint",
            "arm_B_ur_wrist_2_joint",
            "arm_B_ur_wrist_3_joint"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point_b;
        point_b.positions = {-1.0, -0.8, 1.5, -0.7, -1.57, 1.0};
        point_b.time_from_start.sec = 2;
        arm_b_traj.points.push_back(point_b);
        
        // 发布轨迹命令
        arm_a_publisher_->publish(arm_a_traj);
        arm_b_publisher_->publish(arm_b_traj);
        
        // 保持夹爪闭合状态
        send_goal_to_gripper(gripper_a_client_, 0.8, 30.0); // 保持闭合
        send_goal_to_gripper(gripper_b_client_, 0.8, 30.0); // 保持闭合
        
        RCLCPP_INFO(this->get_logger(), "Sent PLACE positions: arms to place location, grippers remain closed");
    }
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_a_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_b_publisher_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_a_client_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_b_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int action_step_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmController>());
    rclcpp::shutdown();
    return 0;
}