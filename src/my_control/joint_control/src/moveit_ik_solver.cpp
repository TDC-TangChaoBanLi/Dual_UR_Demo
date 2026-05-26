#include "joint_control/moveit_ik_solver.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace moveit_ik_solver
{

    MoveItIkSolver::MoveItIkSolver()
        : model_loaded_(false),
          robot_state_ready_(false),
          joint_state_monitor_started_(false),
          joint_state_received_(false) {}



    // 加载机器人模型
    // @param node ROS2节点指针
    // @param robot_description_param 机器人描述参数名称
    // @return 是否加载成功
    bool MoveItIkSolver::loadModel(const rclcpp::Node::SharedPtr &node,
                                   const std::string &robot_description_param)
    {
        node_ = node;

        try
        {
            robot_model_loader_ =
                std::make_shared<robot_model_loader::RobotModelLoader>(
                    node_, robot_description_param, true);

            kinematic_model_ = robot_model_loader_->getModel();
            if (!kinematic_model_)
            {
                RCLCPP_ERROR(node_->get_logger(), "[MoveItIkSolver] Failed to get RobotModel.");
                model_loaded_ = false;
                return false;
            }

            {
                std::lock_guard<std::mutex> lock(robot_state_mutex_);
                robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
                robot_state_->setToDefaultValues();
                robot_state_->update();

                // 这里只是初始化，不代表已经有实时关节状态
                robot_state_ready_ = false;
                joint_state_monitor_started_ = false;
            }

            joint_model_groups_.clear();
            group_variable_names_map_.clear();

            model_loaded_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[MoveItIkSolver] Failed to load model from param '"
                      << robot_description_param << "': " << e.what() << std::endl;
            model_loaded_ = false;
            return false;
        }
    }

    // 注册规划组
    // @param planning_group 规划组名称
    // @return 是否注册成功
    bool MoveItIkSolver::registerPlanningGroup(const std::string &planning_group)
    {
        if (!model_loaded_)
        {
            std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
            return false;
        }

        const moveit::core::JointModelGroup *jmg =
            kinematic_model_->getJointModelGroup(planning_group);

        if (!jmg)
        {
            if (node_)
            {
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

    // 注册规划组
    // @param planning_groups 规划组名称列表
    // @return 是否注册成功
    bool MoveItIkSolver::registerPlanningGroups(
        const std::vector<std::string> &planning_groups)
    {
        bool all_ok = true;
        for (const auto &g : planning_groups)
        {
            if (!registerPlanningGroup(g))
            {
                all_ok = false;
            }
        }
        return all_ok;
    }

    // 获取已注册的规划组名称列表
    // @return 规划组名称列表
    std::vector<std::string> MoveItIkSolver::getRegisteredPlanningGroups() const
    {
        std::vector<std::string> groups;
        groups.reserve(joint_model_groups_.size());
        for (const auto &kv : joint_model_groups_)
        {
            groups.push_back(kv.first);
        }
        return groups;
    }

    // 获取规划组中的关节名称列表
    // @param planning_group 规划组名称
    // @return 关节名称列表
    std::vector<std::string> MoveItIkSolver::getJointNames(
        const std::string &planning_group) const
    {
        if (!model_loaded_)
            return {};

        auto it = group_variable_names_map_.find(planning_group);
        if (it == group_variable_names_map_.end())
        {
            return {};
        }
        return it->second;
    }

    // 获取机器人模型中的所有链接名称列表
    // @return 链接名称列表
    std::vector<std::string> MoveItIkSolver::getLinkNames() const
    {
        if (!model_loaded_)
            return {};
        return kinematic_model_->getLinkModelNames();
    }

    // 启动关节状态监控
    // @param joint_state_topic 关节状态话题
    // @return 是否启动成功
    bool MoveItIkSolver::startJointStateMonitor(const std::string &joint_state_topic)
    {
        if (!model_loaded_ || !node_)
        {
            std::cerr << "[MoveItIkSolver] Call loadModel() before startJointStateMonitor()."
                      << std::endl;
            return false;
        }

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&MoveItIkSolver::jointStateCallback, this, std::placeholders::_1));

        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            joint_state_monitor_started_ = true;
            robot_state_ready_ = false;
        }

        return true;
    }

    // 关节状态话题回调函数
    // @param msg 关节状态消息
    void MoveItIkSolver::jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!model_loaded_ || !robot_state_)
            return;

        std::lock_guard<std::mutex> lock(robot_state_mutex_);

        const size_t n = msg->name.size();
        for (size_t i = 0; i < n; ++i)
        {
            const std::string &joint_name = msg->name[i];

            if (!kinematic_model_->hasJointModel(joint_name))
            {
                continue;
            }

            const moveit::core::JointModel *jm = kinematic_model_->getJointModel(joint_name);
            if (!jm)
                continue;

            const std::vector<std::string> &vars = jm->getVariableNames();
            const std::size_t var_count = vars.size();
            if (var_count == 0)
                continue;

            joint_state_received_ = true;

            // 只处理最常见的单变量关节
            if (var_count == 1 && i < msg->position.size())
            {
                robot_state_->setVariablePosition(vars[0], msg->position[i]);

                if (i < msg->velocity.size())
                {
                    robot_state_->setVariableVelocity(vars[0], msg->velocity[i]);
                }
                continue;
            }

            // 多变量关节这里不处理，避免错误映射
        }

        robot_state_->update();
        robot_state_ready_ = true;
    }

    // 获取已注册的规划组中的关节模型组
    // @param planning_group 规划组名称
    // @return 关节模型组指针
    // @throws std::runtime_error 如果规划组未注册
    const moveit::core::JointModelGroup *MoveItIkSolver::getJointModelGroupChecked(
        const std::string &planning_group) const
    {
        auto it = joint_model_groups_.find(planning_group);
        if (it == joint_model_groups_.end() || it->second == nullptr)
        {
            throw std::runtime_error("Planning group not registered: " + planning_group);
        }
        return it->second;
    }

    // 解析框架名称为链接模型
    // @param name 框架名称
    // @return 链接模型指针
    // @throws std::runtime_error 如果框架名称未注册为链接或关节
    moveit::core::LinkModel *MoveItIkSolver::resolveFrameLink(
        const std::string &name) const
    {
        if (!model_loaded_)
            return nullptr;

        if (kinematic_model_->hasLinkModel(name))
        {
            return kinematic_model_->getLinkModel(name);
        }

        const moveit::core::JointModel *jm = kinematic_model_->getJointModel(name);
        if (jm)
        {
            const auto child_link = jm->getChildLinkModel();
            if (child_link)
            {
                return const_cast<moveit::core::LinkModel *>(child_link);
            }
        }

        return nullptr;
    }

    // 把 relative_name 坐标系中的目标位姿转换到模型/world坐标系
    // @param state 机器人状态
    // @param relative_name 相对坐标系名称
    // @param target_pose 目标位姿
    // @return 转换后的目标位姿
    // @throws std::runtime_error 如果参考框架未注册为链接或关节
    Eigen::Isometry3d MoveItIkSolver::makeTargetInModelFrame(
        const moveit::core::RobotState &state,
        const std::string &relative_name,
        const Eigen::Isometry3d &target_pose) const
    {
        if (relative_name == "world" || relative_name.empty())
        {
            return target_pose;
        }

        moveit::core::LinkModel *ref_link = resolveFrameLink(relative_name);
        if (!ref_link)
        {
            throw std::runtime_error("Reference frame/link/joint not found: " + relative_name);
        }

        const Eigen::Isometry3d model_T_ref = state.getGlobalLinkTransform(ref_link);
        return model_T_ref * target_pose;
    }

    moveit::core::RobotState MoveItIkSolver::makeRobotStateFromJointMap(
        const JointValueMap &joint_map) const
    {
        if (!model_loaded_ || !kinematic_model_)
        {
            throw std::runtime_error("Model not loaded.");
        }

        moveit::core::RobotState local_state(kinematic_model_);

        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            if (robot_state_)
            {
                local_state = *robot_state_;
            }
            else
            {
                local_state.setToDefaultValues();
                local_state.update();
            }
        }

        for (const auto &kv : joint_map)
        {
            const std::string &joint_name = kv.first;
            const double value = kv.second;

            if (!kinematic_model_->hasJointModel(joint_name))
            {
                throw std::runtime_error("Unknown joint in joint_map: " + joint_name);
            }

            const moveit::core::JointModel *jm = kinematic_model_->getJointModel(joint_name);
            if (!jm)
            {
                throw std::runtime_error("JointModel lookup failed for: " + joint_name);
            }

            const auto &vars = jm->getVariableNames();
            if (vars.size() != 1)
            {
                throw std::runtime_error(
                    "Only 1-DOF joints are supported in joint_map interface. Bad joint: " +
                    joint_name);
            }

            local_state.setVariablePosition(vars[0], value);
        }

        local_state.update();
        return local_state;
    }

    // 获取已维护的机器人状态副本
    // @return 机器人状态副本
    // @throws std::runtime_error 如果模型未加载或未注册关节状态监控
    moveit::core::RobotState MoveItIkSolver::getMaintainedRobotStateCopyChecked() const
    {
        if (!model_loaded_ || !kinematic_model_)
        {
            throw std::runtime_error("Model not loaded.");
        }

        std::lock_guard<std::mutex> lock(robot_state_mutex_);

        if (!joint_state_monitor_started_)
        {
            throw std::runtime_error(
                "Joint state monitor has not been started. "
                "Call startJointStateMonitor() before using current-state FK/IK.");
        }
        if (!joint_state_received_)
        {
            throw std::runtime_error(
                "Joint state has not been received. "
                "Wait until at least one joint_states message is received.");
        }

        if (!robot_state_ || !robot_state_ready_)
        {
            throw std::runtime_error(
                "robot_state_ is not ready. Wait until at least one joint_states message is received.");
        }

        return *robot_state_;
    }
    
    // 从 state 中提取某个 group 的当前关节值
    // @param state 机器人状态
    // @param jmg 关节模型组
    // @return 关节值映射
    // @throws std::runtime_error 如果关节模型组未注册
    MoveItIkSolver::JointValueMap MoveItIkSolver::extractGroupJointMap(
        const moveit::core::RobotState &state,
        const moveit::core::JointModelGroup *jmg) const
    {
        JointValueMap out;
        const auto &names = jmg->getVariableNames();
        std::vector<double> values;
        state.copyJointGroupPositions(jmg, values);

        if (names.size() != values.size())
        {
            throw std::runtime_error("extractGroupJointMap size mismatch.");
        }

        for (std::size_t i = 0; i < names.size(); ++i)
        {
            out[names[i]] = values[i];
        }
        return out;
    }
    
    // 从关节值映射中推断规划 group
    // @param joint_map 关节值映射
    // @param end_effector_name 结束执行器名称
    // @return 规划 group
    // @throws std::runtime_error 如果未找到匹配的规划 group
    const moveit::core::JointModelGroup *MoveItIkSolver::inferPlanningGroupFromJointMap(
        const JointValueMap &joint_map,
        const std::string &end_effector_name) const
    {
        moveit::core::LinkModel *ee_link = resolveFrameLink(end_effector_name);
        if (!ee_link)
        {
            throw std::runtime_error("End effector not found: " + end_effector_name);
        }

        const moveit::core::JointModelGroup *best = nullptr;
        std::string best_name;

        for (const auto &kv : joint_model_groups_)
        {
            const auto *jmg = kv.second;
            if (!jmg)
                continue;

            const auto &vars = jmg->getVariableNames();

            bool all_joints_covered = true;
            for (const auto &item : joint_map)
            {
                const std::string &joint_name = item.first;

                if (!kinematic_model_->hasJointModel(joint_name))
                {
                    throw std::runtime_error("Unknown joint in joint_map: " + joint_name);
                }

                const moveit::core::JointModel *jm = kinematic_model_->getJointModel(joint_name);
                if (!jm)
                {
                    throw std::runtime_error("JointModel lookup failed for: " + joint_name);
                }

                const auto &joint_vars = jm->getVariableNames();
                if (joint_vars.size() != 1)
                {
                    throw std::runtime_error(
                        "Only 1-DOF joints are supported in joint_map interface. Bad joint: " +
                        joint_name);
                }

                if (std::find(vars.begin(), vars.end(), joint_vars[0]) == vars.end())
                {
                    all_joints_covered = false;
                    break;
                }
            }

            if (!all_joints_covered)
            {
                continue;
            }

            const auto &link_names = jmg->getLinkModelNames();
            if (std::find(link_names.begin(), link_names.end(), ee_link->getName()) ==
                link_names.end())
            {
                continue;
            }

            if (!jmg->canSetStateFromIK(ee_link->getName()))
            {
                continue;
            }

            // 满足条件时优先选自由度更少的 group
            if (!best || jmg->getVariableCount() < best->getVariableCount())
            {
                best = jmg;
                best_name = kv.first;
            }
        }

        if (!best)
        {
            throw std::runtime_error(
                "Failed to infer planning group from joint_map and end effector: " +
                end_effector_name +
                ". Make sure relevant planning groups are registered and have IK solvers.");
        }

        return best;
    }


    MoveItIkSolver::JointValueMap MoveItIkSolver::getCurrentJointValues(const std::string &planning_group) const
    {
        const moveit::core::JointModelGroup *joint_model_group = nullptr;
        try
        {
            joint_model_group = getJointModelGroupChecked(planning_group);
        }
        catch (const std::exception &e)
        {
            std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
            return {};
        }
        return extractGroupJointMap(*robot_state_, joint_model_group);
    }
    

    // ========================= IK =========================
    constexpr double kTwoPi = 2.0 * M_PI;

    double wrapToNearestEquivalent(double q_sol,
                                   double q_ref,
                                   double q_min,
                                   double q_max)
    {
        double best_q = q_sol;
        double best_dist = std::abs(q_sol - q_ref);

        // 考虑当前值及其 +/- 2pi, +/- 4pi 等有限个等价分支
        // 对于你的关节范围是 ±2pi，这里 k=-2~2 已经足够
        for (int k = -2; k <= 2; ++k)
        {
            double candidate = q_sol + static_cast<double>(k) * kTwoPi;
            if (candidate < q_min || candidate > q_max)
            {
                continue;
            }

            double dist = std::abs(candidate - q_ref);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_q = candidate;
            }
        }

        return best_q;
    }

    std::vector<double> getJointGroupSeedPositions(
        const moveit::core::RobotState &state,
        const moveit::core::JointModelGroup *joint_model_group)
    {
        std::vector<double> seed;
        state.copyJointGroupPositions(joint_model_group, seed);
        return seed;
    }

    void makeSolutionNearestToSeed(
        const moveit::core::JointModelGroup *joint_model_group,
        const std::vector<double> &seed,
        std::vector<double> &solution)
    {
        if (seed.size() != solution.size())
        {
            return;
        }

        const auto &active_joints = joint_model_group->getActiveJointModels();
        std::size_t value_index = 0;

        for (const auto *jm : active_joints)
        {
            const std::size_t nv = jm->getVariableCount();

            // 只处理单自由度 revolute 关节
            if (jm->getType() == moveit::core::JointModel::REVOLUTE && nv == 1)
            {
                const std::string &var_name = jm->getVariableNames()[0];
                const auto &bounds = jm->getVariableBounds(var_name);

                const double q_ref = seed[value_index];
                const double q_sol = solution[value_index];

                solution[value_index] = wrapToNearestEquivalent(
                    q_sol,
                    q_ref,
                    bounds.min_position_,
                    bounds.max_position_);
            }

            value_index += nv;
        }
    }

    void clampSolutionToJointBounds(
        const moveit::core::JointModelGroup *joint_model_group,
        std::vector<double> &solution)
    {
        const auto &active_joints = joint_model_group->getActiveJointModels();
        std::size_t value_index = 0;

        for (const auto *jm : active_joints)
        {
            const std::size_t nv = jm->getVariableCount();

            for (std::size_t i = 0; i < nv; ++i)
            {
                const std::string &var_name = jm->getVariableNames()[i];
                const auto &bounds = jm->getVariableBounds(var_name);

                solution[value_index + i] = std::min(
                    std::max(solution[value_index + i], bounds.min_position_),
                    bounds.max_position_);
            }

            value_index += nv;
        }
    }


MoveItIkResult MoveItIkSolver::solveIk(const JointValueMap &initial_joint_map,
                                       const std::string &end_effector_name,
                                       const Eigen::Isometry3d &target_pose,
                                       const MoveItIkSettings &settings,
                                       const std::string &relative_name)
{
    MoveItIkResult result;
    result.success = false;

    if (!model_loaded_)
    {
        std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
        return result;
    }

    moveit::core::LinkModel *ee_link = resolveFrameLink(end_effector_name);
    if (!ee_link)
    {
        std::cerr << "[MoveItIkSolver] End effector '" << end_effector_name
                  << "' not found as link/joint name." << std::endl;
        return result;
    }

    const moveit::core::JointModelGroup *joint_model_group = nullptr;
    try
    {
        joint_model_group = inferPlanningGroupFromJointMap(initial_joint_map, end_effector_name);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    moveit::core::RobotState local_state(kinematic_model_);
    try
    {
        local_state = makeRobotStateFromJointMap(initial_joint_map);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    Eigen::Isometry3d target_in_model;
    try
    {
        target_in_model = makeTargetInModelFrame(local_state, relative_name, target_pose);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    // 记录 IK 前的 seed，用于后续“最近等价解”展开
    const std::vector<double> seed = getJointGroupSeedPositions(local_state, joint_model_group);

    // 为了避免多圈关节跳支路，这里不再做随机重试
    // 始终使用当前 initial_joint_map 对应的状态作为 IK seed
    const bool found_ik = local_state.setFromIK(
        joint_model_group,
        target_in_model,
        ee_link->getName(),
        settings.timeout);

    if (!found_ik)
    {
        if (settings.verbose)
        {
            std::cout << "[MoveItIkSolver] IK did not converge for inferred group."
                      << std::endl;
        }
        return result;
    }

    std::vector<double> sol;
    local_state.copyJointGroupPositions(joint_model_group, sol);

    // 关键：把解展开到距离 seed 最近的等价分支，避免 0 <-> +/-2pi 跳变
    makeSolutionNearestToSeed(joint_model_group, seed, sol);

    // 仅做简单裁剪，不直接调用 joint_group 级别的 enforceBounds 以免重新包角
    clampSolutionToJointBounds(joint_model_group, sol);

    // 将处理后的最终解写回状态
    local_state.setJointGroupPositions(joint_model_group, sol);
    local_state.update();

    result.success = true;
    result.joint_angles = sol;
    result.joint_map = extractGroupJointMap(local_state, joint_model_group);

    for (const auto &kv : joint_model_groups_)
    {
        if (kv.second == joint_model_group)
        {
            result.planning_group = kv.first;
            break;
        }
    }

    return result;
}


MoveItIkResult MoveItIkSolver::solveIk(const std::string &planning_group,
                                       const std::string &end_effector_name,
                                       const Eigen::Isometry3d &target_pose,
                                       const MoveItIkSettings &settings,
                                       const std::string &relative_name)
{
    MoveItIkResult result;
    result.success = false;

    if (!model_loaded_)
    {
        std::cerr << "[MoveItIkSolver] Model not loaded." << std::endl;
        return result;
    }

    const moveit::core::JointModelGroup *joint_model_group = nullptr;
    try
    {
        joint_model_group = getJointModelGroupChecked(planning_group);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    moveit::core::LinkModel *ee_link = resolveFrameLink(end_effector_name);
    if (!ee_link)
    {
        std::cerr << "[MoveItIkSolver] End effector not found: "
                  << end_effector_name << std::endl;
        return result;
    }

    moveit::core::RobotState local_state(kinematic_model_);
    try
    {
        local_state = getMaintainedRobotStateCopyChecked();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    Eigen::Isometry3d target_in_model;
    try
    {
        target_in_model = makeTargetInModelFrame(local_state, relative_name, target_pose);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MoveItIkSolver] " << e.what() << std::endl;
        return result;
    }

    // 使用实时维护的当前机器人状态作为 seed
    const std::vector<double> seed = getJointGroupSeedPositions(local_state, joint_model_group);

    // 同样不再随机重试，防止 IK 落到另一圈等价分支
    const bool found_ik = local_state.setFromIK(
        joint_model_group,
        target_in_model,
        ee_link->getName(),
        settings.timeout);

    if (!found_ik)
    {
        if (settings.verbose)
        {
            std::cout << "[MoveItIkSolver] IK did not converge for planning_group='"
                      << planning_group << "'." << std::endl;
        }
        return result;
    }

    std::vector<double> sol;
    local_state.copyJointGroupPositions(joint_model_group, sol);

    // 关键：就近展开
    makeSolutionNearestToSeed(joint_model_group, seed, sol);

    // 仅裁剪到物理范围
    clampSolutionToJointBounds(joint_model_group, sol);

    local_state.setJointGroupPositions(joint_model_group, sol);
    local_state.update();

    result.success = true;
    result.joint_angles = sol;
    result.joint_map = extractGroupJointMap(local_state, joint_model_group);
    result.planning_group = planning_group;
    return result;
}
    // ========================= FK =========================

    Eigen::Isometry3d MoveItIkSolver::solveFk(const JointValueMap &joint_map,
                                              const std::string &target_name,
                                              const std::string &relative_name) const
    {
        if (!model_loaded_)
        {
            throw std::runtime_error("Model not loaded.");
        }

        moveit::core::LinkModel *target_link = resolveFrameLink(target_name);
        if (!target_link)
        {
            throw std::runtime_error("Target frame/link/joint not found: " + target_name);
        }

        moveit::core::RobotState local_state = makeRobotStateFromJointMap(joint_map);
        const Eigen::Isometry3d global_pose = local_state.getGlobalLinkTransform(target_link);

        if (relative_name.empty() || relative_name == "world")
        {
            return global_pose;
        }

        moveit::core::LinkModel *ref_link = resolveFrameLink(relative_name);
        if (!ref_link)
        {
            throw std::runtime_error("Reference frame/link/joint not found: " + relative_name);
        }

        const Eigen::Isometry3d ref_global_pose = local_state.getGlobalLinkTransform(ref_link);
        return ref_global_pose.inverse() * global_pose;
    }

    Eigen::Isometry3d MoveItIkSolver::solveFk(const std::string &target_name,
                                              const std::string &relative_name) const
    {
        if (!model_loaded_)
        {
            throw std::runtime_error("Model not loaded.");
        }

        moveit::core::LinkModel *target_link = resolveFrameLink(target_name);
        if (!target_link)
        {
            throw std::runtime_error("Target frame/link/joint not found: " + target_name);
        }

        moveit::core::RobotState local_state = getMaintainedRobotStateCopyChecked();
        local_state.update();

        const Eigen::Isometry3d global_pose = local_state.getGlobalLinkTransform(target_link);

        if (relative_name.empty() || relative_name == "world")
        {
            return global_pose;
        }

        moveit::core::LinkModel *ref_link = resolveFrameLink(relative_name);
        if (!ref_link)
        {
            throw std::runtime_error("Reference frame/link/joint not found: " + relative_name);
        }

        const Eigen::Isometry3d ref_global_pose = local_state.getGlobalLinkTransform(ref_link);
        return ref_global_pose.inverse() * global_pose;
    }

    // ====================== Jacobian ======================

    Eigen::MatrixXd MoveItIkSolver::getJac(const std::string &planning_group,
                                           const JointValueMap &joint_map,
                                           const std::string &end_effector_name) const
    {
        if (!model_loaded_)
        {
            throw std::runtime_error("Model not loaded.");
        }

        const moveit::core::JointModelGroup *jmg =
            getJointModelGroupChecked(planning_group);

        moveit::core::LinkModel *ee_link = resolveFrameLink(end_effector_name);
        if (!ee_link)
        {
            throw std::runtime_error("End effector not found: " + end_effector_name);
        }

        moveit::core::RobotState local_state = makeRobotStateFromJointMap(joint_map);

        Eigen::MatrixXd jacobian;
        local_state.getJacobian(jmg, ee_link, Eigen::Vector3d::Zero(), jacobian);
        return jacobian;
    }

} // namespace moveit_ik_solver