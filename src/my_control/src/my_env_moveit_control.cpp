#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <thread>

class DualArmDemo {
public:
    DualArmDemo(rclcpp::Node::SharedPtr node) : node_(node) {
        // 初始化所有MoveGroup接口
        ur_a_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_A");
        ur_b_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_B");
        gripper_a_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper_A");
        gripper_b_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper_B");

        // 设置规划参数
        setupPlanningParameters();
        
        RCLCPP_INFO(node_->get_logger(), "双机械臂演示程序初始化完成");
    }

    // 运行所有演示
    void runAllDemos() {
        RCLCPP_INFO(node_->get_logger(), "=== 开始双机械臂演示 ===");
        
        // 等待MoveGroup初始化
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // 仅测试：ur_A、ur_B 以及两个夹爪的规划（分别测试）
        demoURAGroup();
        // 给一点时间缓冲
        std::this_thread::sleep_for(std::chrono::seconds(1));
        demoURBGroup();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        demoGripperControl();
        
        RCLCPP_INFO(node_->get_logger(), "=== 双机械臂演示完成 ===");
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ur_a_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ur_b_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_a_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_b_group_;

    void setupPlanningParameters() {
        // 设置规划时间
        ur_a_group_->setPlanningTime(10.0);
        ur_b_group_->setPlanningTime(10.0);
        gripper_a_group_->setPlanningTime(3.0);
        gripper_b_group_->setPlanningTime(3.0);

        // 设置规划尝试次数
        ur_a_group_->setNumPlanningAttempts(30);
        ur_b_group_->setNumPlanningAttempts(30);
        gripper_a_group_->setNumPlanningAttempts(10);
        gripper_b_group_->setNumPlanningAttempts(10);
        
        // 设置目标容差
        ur_a_group_->setGoalTolerance(0.01);
        ur_b_group_->setGoalTolerance(0.01);
        gripper_a_group_->setGoalTolerance(0.005);
        gripper_b_group_->setGoalTolerance(0.005);
    }

    // 对ur_A规划组进行规划（保持不变）
    void demoURAGroup() {
        RCLCPP_INFO(node_->get_logger(), "=== UR_A规划组演示 ===");
        
        // 1 运动到预设的"home"状态
        RCLCPP_INFO(node_->get_logger(), "1 运动到home状态");
        if (moveToNamedTarget(ur_a_group_, "home")) {
            RCLCPP_INFO(node_->get_logger(), "UR_A组运动到home成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "UR_A组运动到home失败");
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 2 手动设置位姿状态
        RCLCPP_INFO(node_->get_logger(), "2 手动设置位姿状态");
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.1;
        target_pose.position.y = -0.2;
        target_pose.position.z = 1.5;
        target_pose.orientation.x = 0.5;
        target_pose.orientation.y = -0.5;
        target_pose.orientation.z = -0.5;
        target_pose.orientation.w = 0.5;
        
        ur_a_group_->setPoseTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = ur_a_group_->plan(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "UR_A组位姿规划成功");
            ur_a_group_->execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "UR_A组位姿规划失败");
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 3 再次回到home状态
        RCLCPP_INFO(node_->get_logger(), "3 再次运动到home状态");
        moveToNamedTarget(ur_a_group_, "home");
    }

    // 对 ur_B 规划组进行规划（与 ur_A 对称）
    void demoURBGroup() {
        RCLCPP_INFO(node_->get_logger(), "=== 示例: UR_B 规划组演示 ===");

        // 运动到预设的"home"状态
        RCLCPP_INFO(node_->get_logger(), "1 运动到home状态");
        if (moveToNamedTarget(ur_b_group_, "home")) {
            RCLCPP_INFO(node_->get_logger(), "UR_B组运动到home成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "UR_B组运动到home失败");
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 手动设置位姿状态
        RCLCPP_INFO(node_->get_logger(), "2 手动设置位姿状态");

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = -0.1;
        target_pose.position.y = -0.2;
        target_pose.position.z = 1.5;
        target_pose.orientation.x = 0.5;
        target_pose.orientation.y = 0.5;
        target_pose.orientation.z = 0.5;
        target_pose.orientation.w = 0.5;

        ur_b_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = ur_b_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "UR_B组位姿规划成功");
            ur_b_group_->execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "UR_B组位姿规划失败");
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 再次回到home状态
        RCLCPP_INFO(node_->get_logger(), "3 再次运动到home状态");
        moveToNamedTarget(ur_b_group_, "home");
    }

    // 单独控制夹爪（保持不变）
    void demoGripperControl() {
        RCLCPP_INFO(node_->get_logger(), "=== 夹爪控制演示 ===");
        
        // 4.1 控制gripper_A
        RCLCPP_INFO(node_->get_logger(), "1 控制gripper_A夹爪");
        
        // 打开夹爪A
        RCLCPP_INFO(node_->get_logger(), "打开gripper_A");
        if (moveToNamedTarget(gripper_a_group_, "open")) {
            RCLCPP_INFO(node_->get_logger(), "gripper_A打开成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "gripper_A打开失败");
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 关闭夹爪A
        RCLCPP_INFO(node_->get_logger(), "关闭gripper_A");
        if (moveToNamedTarget(gripper_a_group_, "close")) {
            RCLCPP_INFO(node_->get_logger(), "gripper_A关闭成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "gripper_A关闭失败");
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 控制gripper_B
        RCLCPP_INFO(node_->get_logger(), "2 控制gripper_B夹爪");
        
        // 打开夹爪B
        RCLCPP_INFO(node_->get_logger(), "打开gripper_B");
        if (moveToNamedTarget(gripper_b_group_, "open")) {
            RCLCPP_INFO(node_->get_logger(), "gripper_B打开成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "gripper_B打开失败");
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 关闭夹爪B
        RCLCPP_INFO(node_->get_logger(), "关闭gripper_B");
        if (moveToNamedTarget(gripper_b_group_, "close")) {
            RCLCPP_INFO(node_->get_logger(), "gripper_B关闭成功");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "gripper_B关闭失败");
        }
    }

    // 辅助函数: 运动到命名目标
    bool moveToNamedTarget(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group, 
                            const std::string& target_name) {
        // 使用当前机器人状态作为起始状态，避免使用过期/脏的 transforms
        group->setStartStateToCurrentState();
        group->setNamedTarget(target_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = group->plan(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            group->execute(plan);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "运动到 %s 失败", target_name.c_str());
            return false;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("my_env_moveit_control");
    
    // 创建演示对象
    auto demo = std::make_shared<DualArmDemo>(node);
    
    // 在单独的线程中运行演示
    std::thread demo_thread([demo]() {
        // 等待ROS2系统完全启动
        std::this_thread::sleep_for(std::chrono::seconds(2));
        demo->runAllDemos();
    });
    
    // 运行节点
    rclcpp::spin(node);
    
    // 等待演示线程结束
    if (demo_thread.joinable()) {
        demo_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}
