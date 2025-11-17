#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char* argv[])
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ur_moveit_control");
    auto logger = node->get_logger();

    // 创建一个多线程执行器，MoveGroupInterface 需要它
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // 1. 创建 MoveIt MoveGroup Interface
    // 参数 "ur_manipulator" 必须与你在 MoveIt Setup Assistant 中配置的规划组名称一致
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    // 设置规划时间、目标容差等参数（可选）
    move_group.setPlanningTime(10.0); // 秒
    move_group.setGoalTolerance(0.01); // 米

    // 2. 设置目标位姿
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; // 默认朝向
    target_pose.position.x = 0.3;    // 单位：米
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.4;
    move_group.setPoseTarget(target_pose);

    // 3. 进行规划并移动到目标
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success) {
    RCLCPP_INFO(logger, "规划成功，开始移动！");
    move_group.execute(my_plan);
    } else {
    RCLCPP_ERROR(logger, "规划失败！");
    }

    // 关闭 ROS2
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}