#include "joint_control/moveit_ik_solver.hpp"

#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace moveit_ik_solver {

MoveItIkSolver::MoveItIkSolver()
    : model_loaded_(false), robot_state_ready_(false) {}

bool MoveItIkSolver::loadModel(const rclcpp::Node::SharedPtr& node,
                               const std::string& robot_description_topic) {
    node_ = node;

    try {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, robot_description_topic, true);

        kinematic_model_ = robot_model_loader_->getModel();
        if (!kinematic_model_) {
            RCLCPP_ERROR(node_->get_logger(), "[MoveItIkSolver] Failed to get RobotModel.");
            model_loaded_ = false;
            return false;
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();
        robot_state_->update();

        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            robot_state_ready_ = true;  // 至少已有默认状态
        }

        joint_model_groups_.clear();
        group_variable_names_map_.clear();

        model_loaded_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[MoveItIkSolver] Failed to load model from topic '"
                  << robot_description_topic << "': " << e.what() << std::endl;
        model_loaded_ = false;
        return false;
    }
}

bool MoveItIkSolver::registerPlanningGroup(const std::string& planning_group) {
    if (!model_loaded_) {
        std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
        return false;
    }

    const moveit::core::JointModelGroup* jmg =
        kinematic_model_->getJointModelGroup(planning_group);

    if (!jmg) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[MoveItIkSolver] JointModelGroup '%s' not found.",
                         planning_group.c_str());
        }
        return false;
    }

    joint_model_groups_[planning_group] = jmg;
    group_variable_names_map_[planning_group] = jmg->getVariableNames();
    return true;
}

bool MoveItIkSolver::registerPlanningGroups(
    const std::vector<std::string>& planning_groups) {
    bool all_ok = true;
    for (const auto& g : planning_groups) {
        if (!registerPlanningGroup(g)) {
            all_ok = false;
        }
    }
    return all_ok;
}

std::vector<std::string> MoveItIkSolver::getRegisteredPlanningGroups() const {
    std::vector<std::string> groups;
    groups.reserve(joint_model_groups_.size());
    for (const auto& kv : joint_model_groups_) {
        groups.push_back(kv.first);
    }
    return groups;
}

std::vector<std::string> MoveItIkSolver::getJointNames(
    const std::string& planning_group) const {
    if (!model_loaded_) return {};

    auto it = group_variable_names_map_.find(planning_group);
    if (it == group_variable_names_map_.end()) {
        return {};
    }
    return it->second;
}

std::vector<std::string> MoveItIkSolver::getLinkNames() const {
    if (!model_loaded_) return {};

    std::vector<std::string> link_names;
    link_names = kinematic_model_->getLinkModelNames();
    return link_names;
}

bool MoveItIkSolver::startJointStateMonitor(const std::string& joint_state_topic) {
    if (!model_loaded_ || !node_) {
        std::cerr << "[MoveItIkSolver] Call loadModel() before startJointStateMonitor()."
                  << std::endl;
        return false;
    }

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&MoveItIkSolver::jointStateCallback, this, std::placeholders::_1));

    return true;
}

bool MoveItIkSolver::hasRobotState() const {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    return robot_state_ready_;
}

void MoveItIkSolver::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!model_loaded_ || !robot_state_) return;

    std::lock_guard<std::mutex> lock(robot_state_mutex_);

    const size_t n = msg->name.size();

    for (size_t i = 0; i < n; ++i) {
        const std::string& joint_name = msg->name[i];

        if (!kinematic_model_->hasJointModel(joint_name)) {
            continue;
        }

        const moveit::core::JointModel* jm = kinematic_model_->getJointModel(joint_name);
        if (!jm) continue;

        const std::vector<std::string>& vars = jm->getVariableNames();
        const std::size_t var_count = vars.size();

        if (var_count == 0) continue;

        // 这里只做最常见的一维关节更新
        if (var_count == 1 && i < msg->position.size()) {
            robot_state_->setVariablePosition(vars[0], msg->position[i]);

            if (i < msg->velocity.size()) {
                robot_state_->setVariableVelocity(vars[0], msg->velocity[i]);
            }
            continue;
        }

        // 多变量关节（很少见于普通机械臂）这里跳过，避免错误映射
    }

    robot_state_->update();
    robot_state_ready_ = true;
}

const moveit::core::JointModelGroup* MoveItIkSolver::getJointModelGroupChecked(
    const std::string& planning_group) const {
    auto it = joint_model_groups_.find(planning_group);
    if (it == joint_model_groups_.end() || it->second == nullptr) {
        throw std::runtime_error("Planning group not registered: " + planning_group);
    }
    return it->second;
}

moveit::core::LinkModel* MoveItIkSolver::getFrameId(const std::string& name) const {
    if (!model_loaded_) return nullptr;

    if (kinematic_model_->hasLinkModel(name)) {
        return kinematic_model_->getLinkModel(name);
    }

    const moveit::core::JointModel* jm = kinematic_model_->getJointModel(name);
    if (jm) {
        const auto parent_link = jm->getParentLinkModel();
        if (parent_link) {
            return const_cast<moveit::core::LinkModel*>(parent_link);
        }
    }

    return nullptr;
}

Eigen::Matrix4d MoveItIkSolver::isometryToMat4(const Eigen::Isometry3d& T) const {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3,3>(0,0) = T.linear();
    M.block<3,1>(0,3) = T.translation();
    return M;
}

Eigen::Isometry3d MoveItIkSolver::makeTargetInModelFrame(
    const moveit::core::RobotState& state,
    const std::string& relative_name,
    const Eigen::Isometry3d& target_pose) const {
    if (relative_name == "world" || relative_name.empty()) {
        return target_pose;
    }

    moveit::core::LinkModel* ref_link = getFrameId(relative_name);
    if (!ref_link) {
        throw std::runtime_error("Reference frame/link not found: " + relative_name);
    }

    Eigen::Isometry3d model_T_ref = state.getGlobalLinkTransform(ref_link);
    return model_T_ref * target_pose;
}

std::vector<double> MoveItIkSolver::getCurrentGroupPositionsUnsafe(
    const std::string& planning_group) const {
    const moveit::core::JointModelGroup* joint_model_group =
        getJointModelGroupChecked(planning_group);

    std::vector<double> q;
    robot_state_->copyJointGroupPositions(joint_model_group, q);
    return q;
}

MoveItIkResult MoveItIkSolver::solveIk(
    const std::string& planning_group,
    const std::vector<double>& initial_joint_angles,
    const std::string& end_effector_name,
    const std::string& relative_name,
    const Eigen::Isometry3d& target_pose,
    const MoveItIkSettings& settings) {
    MoveItIkResult result;
    result.success = false;

    if (!model_loaded_) {
        std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
        return result;
    }

    const moveit::core::JointModelGroup* joint_model_group = nullptr;
    std::vector<std::string> group_variable_names;
    try {
        joint_model_group = getJointModelGroupChecked(planning_group);
        group_variable_names = group_variable_names_map_.at(planning_group);
    } catch (const std::exception& e) {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    if (initial_joint_angles.size() != group_variable_names.size()) {
        std::cerr << "[MoveItIkSolver] Size mismatch: initial_joint_angles ("
                  << initial_joint_angles.size() << ") vs group variables ("
                  << group_variable_names.size() << ")" << std::endl;
        return result;
    }

    moveit::core::LinkModel* ee_link = getFrameId(end_effector_name);
    if (!ee_link) {
        std::cerr << "[MoveItIkSolver] End effector '" << end_effector_name
                  << "' not found as link/joint name." << std::endl;
        return result;
    }

    moveit::core::RobotState local_state(kinematic_model_);

    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (robot_state_) {
            local_state = *robot_state_;
        } else {
            local_state.setToDefaultValues();
        }
    }

    local_state.setJointGroupPositions(joint_model_group, initial_joint_angles);
    local_state.update();

    Eigen::Isometry3d target_in_model;
    try {
        target_in_model = makeTargetInModelFrame(local_state, relative_name, target_pose);
    } catch (const std::exception& e) {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    bool found_ik = false;
    std::vector<double> sol;
    std::vector<double> seed = initial_joint_angles;

    for (unsigned int attempt = 0; attempt < std::max(1u, settings.attempts); ++attempt) {
        local_state.setJointGroupPositions(joint_model_group, seed);
        local_state.update();

        found_ik = local_state.setFromIK(
            joint_model_group,
            target_in_model,
            ee_link->getName(),
            settings.timeout);

        if (found_ik) {
            local_state.enforceBounds(joint_model_group);
            local_state.copyJointGroupPositions(joint_model_group, sol);
            break;
        }

        local_state.setToRandomPositions(joint_model_group);
        local_state.copyJointGroupPositions(joint_model_group, seed);
    }

    if (!found_ik) {
        if (settings.verbose) {
            std::cout << "[MoveItIkSolver] IK did not converge for planning_group='"
                      << planning_group << "'." << std::endl;
        }
        return result;
    }

    result.success = true;
    result.joint_angles = sol;
    return result;
}

MoveItIkResult MoveItIkSolver::solveIk(
    const std::string& planning_group,
    const std::string& end_effector_name,
    const std::string& relative_name,
    const Eigen::Isometry3d& target_pose,
    const MoveItIkSettings& settings) {
    if (!model_loaded_) {
        std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
        return MoveItIkResult{};
    }

    std::vector<double> q0;
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (!robot_state_ready_) {
            std::cerr << "[MoveItIkSolver] robot_state_ is not ready." << std::endl;
            return MoveItIkResult{};
        }
        try {
            q0 = getCurrentGroupPositionsUnsafe(planning_group);
        } catch (const std::exception& e) {
            std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
            return MoveItIkResult{};
        }
    }

    return solveIk(planning_group, q0, end_effector_name, relative_name, target_pose, settings);
}

Eigen::Matrix4d MoveItIkSolver::solveFk(
    const std::string& planning_group,
    const std::vector<double>& joint_angles,
    const std::string& end_effector_name) const {
    if (!model_loaded_) {
        throw std::runtime_error("Model not loaded.");
    }

    const moveit::core::JointModelGroup* joint_model_group =
        getJointModelGroupChecked(planning_group);

    const auto& group_variable_names = group_variable_names_map_.at(planning_group);
    if (joint_angles.size() != group_variable_names.size()) {
        throw std::runtime_error("solveFk joint_angles size mismatch with planning group variables.");
    }

    moveit::core::LinkModel* ee_link = getFrameId(end_effector_name);
    if (!ee_link) {
        throw std::runtime_error("End effector not found: " + end_effector_name);
    }

    moveit::core::RobotState local_state(kinematic_model_);
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (robot_state_) {
            local_state = *robot_state_;
        } else {
            local_state.setToDefaultValues();
        }
    }

    local_state.setJointGroupPositions(joint_model_group, joint_angles);
    local_state.update();

    return isometryToMat4(local_state.getGlobalLinkTransform(ee_link));
}

Eigen::Matrix4d MoveItIkSolver::solveFk(
    const std::string& planning_group,
    const std::string& end_effector_name) const {
    if (!model_loaded_) {
        throw std::runtime_error("Model not loaded.");
    }

    std::vector<double> q;
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (!robot_state_ready_) {
            throw std::runtime_error("robot_state_ is not ready.");
        }
        q = getCurrentGroupPositionsUnsafe(planning_group);
    }

    return solveFk(planning_group, q, end_effector_name);
}

Eigen::MatrixXd MoveItIkSolver::getJac(
    const std::string& planning_group,
    const std::vector<double>& joint_angles,
    const std::string& end_effector_name) const {
    if (!model_loaded_) {
        throw std::runtime_error("Model not loaded.");
    }

    const moveit::core::JointModelGroup* joint_model_group =
        getJointModelGroupChecked(planning_group);

    const auto& group_variable_names = group_variable_names_map_.at(planning_group);
    if (joint_angles.size() != group_variable_names.size()) {
        throw std::runtime_error("getJac joint_angles size mismatch with planning group variables.");
    }

    moveit::core::LinkModel* ee_link = getFrameId(end_effector_name);
    if (!ee_link) {
        throw std::runtime_error("End effector not found: " + end_effector_name);
    }

    moveit::core::RobotState local_state(kinematic_model_);
    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        if (robot_state_) {
            local_state = *robot_state_;
        } else {
            local_state.setToDefaultValues();
        }
    }

    local_state.setJointGroupPositions(joint_model_group, joint_angles);
    local_state.update();

    Eigen::MatrixXd jacobian;
    local_state.getJacobian(joint_model_group, ee_link, Eigen::Vector3d::Zero(), jacobian);
    return jacobian;
}

} // namespace moveit_ik_solver