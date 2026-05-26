#pragma once

#include <Eigen/Geometry>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_ik_solver
{

    struct MoveItIkSettings
    {
        double timeout{0.05};
        unsigned int attempts{10};
        bool verbose{false};
    };

    struct MoveItIkResult
    {
        bool success{false};

        // 按最终使用的 planning group 变量顺序返回
        std::vector<double> joint_angles;

        // 更适合外部直接使用
        std::map<std::string, double> joint_map;

        // 实际使用/推断出的 group
        std::string planning_group;
    };

    class MoveItIkSolver
    {
    public:
        using JointValueMap = std::map<std::string, double>;

        MoveItIkSolver();

        bool loadModel(const rclcpp::Node::SharedPtr &node,
                       const std::string &robot_description_param = "robot_description");

        bool registerPlanningGroup(const std::string &planning_group);
        bool registerPlanningGroups(const std::vector<std::string> &planning_groups);
        bool startJointStateMonitor(const std::string &joint_state_topic = "/joint_states");

        std::vector<std::string> getRegisteredPlanningGroups() const;
        std::vector<std::string> getJointNames(const std::string &planning_group) const;
        std::vector<std::string> getLinkNames() const;
        
        // 规划组各个关节名及其状态
        JointValueMap getCurrentJointValues(const std::string &planning_group) const;
        // JointValueMap getTargetJointValues(const std::string &planning_group) const;

        // ========================= IK =========================

        // 1) 带 seed joint map：不需要显式传 planning_group，内部自动推断
        MoveItIkResult solveIk(const JointValueMap &initial_joint_map,
                               const std::string &end_effector_name,
                               const Eigen::Isometry3d &target_pose,
                               const MoveItIkSettings &settings = MoveItIkSettings{},
                               const std::string &relative_name = "world");

        // 2) 基于实时维护的当前状态做 IK：必须显式指定 planning_group
        MoveItIkResult solveIk(const std::string &planning_group,
                               const std::string &end_effector_name,
                               const Eigen::Isometry3d &target_pose,
                               const MoveItIkSettings &settings = MoveItIkSettings{},
                               const std::string &relative_name = "world");

        // ========================= FK =========================

        // 1) 带 joint map：不需要 planning_group
        Eigen::Isometry3d solveFk(const JointValueMap &joint_map,
                                  const std::string &target_name,
                                  const std::string &relative_name = "world") const;

        // 2) 使用当前维护的 robot_state_：不需要 planning_group
        Eigen::Isometry3d solveFk(const std::string &target_name,
                                  const std::string &relative_name = "world") const;

        // ====================== Jacobian ======================

        // Jacobian 仍然需要 planning_group，因为列空间属于某个 group
        Eigen::MatrixXd getJac(const std::string &planning_group,
                               const JointValueMap &joint_map,
                               const std::string &end_effector_name) const;

        Eigen::Matrix4d isometryToMat4(const Eigen::Isometry3d &T) const;

    private:
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        const moveit::core::JointModelGroup *getJointModelGroupChecked(
            const std::string &planning_group) const;

        // 若 name 是 link 名，则直接返回该 link
        // 若 name 是 joint 名，则返回该 joint 的 child link
        moveit::core::LinkModel *resolveFrameLink(const std::string &name) const;

        // 把 relative_name 坐标系中的目标位姿转换到模型/world坐标系
        Eigen::Isometry3d makeTargetInModelFrame(const moveit::core::RobotState &state,
                                                 const std::string &relative_name,
                                                 const Eigen::Isometry3d &target_pose) const;

        // 从 joint map 构造 local RobotState：先复制当前状态（若存在），再覆盖 joint map
        moveit::core::RobotState makeRobotStateFromJointMap(
            const JointValueMap &joint_map) const;

        // 只允许在 joint_states 已启动并收到有效状态后调用
        moveit::core::RobotState getMaintainedRobotStateCopyChecked() const;

        // 从 state 中提取某个 group 的当前关节值
        JointValueMap extractGroupJointMap(const moveit::core::RobotState &state,
                                           const moveit::core::JointModelGroup *jmg) const;

        // 根据 joint_map + ee 自动推断 planning_group
        const moveit::core::JointModelGroup *inferPlanningGroupFromJointMap(
            const JointValueMap &joint_map,
            const std::string &end_effector_name) const;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotModelPtr kinematic_model_;
        moveit::core::RobotStatePtr robot_state_;

        bool model_loaded_{false};
        bool robot_state_ready_{false};
        bool joint_state_monitor_started_{false};
        bool joint_state_received_{false};

        mutable std::mutex robot_state_mutex_;

        // 仍然需要缓存注册过的 group，IK/Jacobian 都要用
        std::unordered_map<std::string, const moveit::core::JointModelGroup *> joint_model_groups_;
        std::unordered_map<std::string, std::vector<std::string>> group_variable_names_map_;
    };

} // namespace moveit_ik_solver