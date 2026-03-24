#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_ik_solver {

struct MoveItIkSettings {
    double timeout = 0.05;
    unsigned int attempts = 1;
    bool verbose = false;
};

struct MoveItIkResult {
    bool success = false;
    std::vector<double> joint_angles;   // planning_group 变量顺序
};

class MoveItIkSolver {
public:
    MoveItIkSolver();

    // 这里只加载 MoveIt 模型；file_path 实际上传 robot_description 参数名
    bool loadModel(const rclcpp::Node::SharedPtr& node,
                   const std::string& robot_description_topic = "/robot_description");

    // 提前注册需要用到的 planning groups
    bool registerPlanningGroup(const std::string& planning_group);
    bool registerPlanningGroups(const std::vector<std::string>& planning_groups);

    std::vector<std::string> getRegisteredPlanningGroups() const;
    std::vector<std::string> getJointNames(const std::string& planning_group) const;
    std::vector<std::string> getLinkNames() const;

    // 启动 joint_states 订阅，用于动态维护 robot_state_
    bool startJointStateMonitor(const std::string& joint_state_topic = "/joint_states");

    bool hasRobotState() const;

    // ---------------- IK：手动输入初始关节角 ----------------
    MoveItIkResult solveIk(const std::string& planning_group,
                           const std::vector<double>& initial_joint_angles,
                           const std::string& end_effector_name,
                           const std::string& relative_name,
                           const Eigen::Isometry3d& target_pose,
                           const MoveItIkSettings& settings = MoveItIkSettings());

    // ---------------- IK：直接使用 robot_state_ 当前状态 ----------------
    MoveItIkResult solveIk(const std::string& planning_group,
                           const std::string& end_effector_name,
                           const std::string& relative_name,
                           const Eigen::Isometry3d& target_pose,
                           const MoveItIkSettings& settings = MoveItIkSettings());

    // ---------------- FK：手动输入关节角 ----------------
    Eigen::Matrix4d solveFk(const std::string& planning_group,
                            const std::vector<double>& joint_angles,
                            const std::string& end_effector_name) const;

    // ---------------- FK：直接使用 robot_state_ 当前状态 ----------------
    Eigen::Matrix4d solveFk(const std::string& planning_group,
                            const std::string& end_effector_name) const;

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    const moveit::core::JointModelGroup* getJointModelGroupChecked(
        const std::string& planning_group) const;

    moveit::core::LinkModel* getFrameId(const std::string& name) const;

    Eigen::MatrixXd getJac(const std::string& planning_group,
                           const std::vector<double>& joint_angles,
                           const std::string& end_effector_name) const;

    Eigen::Matrix4d isometryToMat4(const Eigen::Isometry3d& T) const;
    Eigen::Isometry3d makeTargetInModelFrame(const moveit::core::RobotState& state,
                                             const std::string& relative_name,
                                             const Eigen::Isometry3d& target_pose) const;

    std::vector<double> getCurrentGroupPositionsUnsafe(const std::string& planning_group) const;

private:
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;

    std::unordered_map<std::string, const moveit::core::JointModelGroup*> joint_model_groups_;
    std::unordered_map<std::string, std::vector<std::string>> group_variable_names_map_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    mutable std::mutex robot_state_mutex_;
    bool model_loaded_ = false;
    bool robot_state_ready_ = false;
};

} // namespace moveit_ik_solver