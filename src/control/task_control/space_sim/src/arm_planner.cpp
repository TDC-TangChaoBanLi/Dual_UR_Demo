#include "space_sim_task_control/arm_planner.hpp"
#include "space_sim_task_control/diagnostics.hpp"
#include <multi_arm_motion_core/distance.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <thread>
#include <utility>

#include "rclcpp/duration.hpp"
#include "rcutils/logging.h"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit/trajectory_processing/ruckig_traj_smoothing.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"

namespace space_sim_task_control
{

// --------------------------------------------------------------------------- //
// ExecutionHandle
// --------------------------------------------------------------------------- //

void ExecutionHandle::stop()
{
  stop_requested_ = true;
  if (owner_ != nullptr) {
    owner_->mg().stop();
  }
}

bool ExecutionHandle::finished() const
{
  return future_.valid() &&
         future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

bool ExecutionHandle::wait(double timeout_s)
{
  if (!future_.valid()) {
    error_ = "执行句柄无效";
    ok_ = false;
    return true;
  }
  const double effective = timeout_s > 0.0 ? timeout_s : 0.0;
  if (effective > 0.0 &&
    future_.wait_for(std::chrono::duration<double>(effective)) != std::future_status::ready)
  {
    error_ = "执行超时（" + std::to_string(static_cast<int>(effective)) +
             " s 未完成，已请求 stop；检查轨迹时间戳/仿真跟踪）";
    stop();
    ok_ = false;
    return false;
  }
  const auto code = future_.get();
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    std::ostringstream os;
    os << "执行失败(code " << code.val << ")";
    if (stop_requested_) {
      os << " [已被 stop 抢占]";
    }
    error_ = os.str();
    ok_ = false;
    return true;
  }
  ok_ = true;
  return true;
}

// --------------------------------------------------------------------------- //
// ArmPlanner
// --------------------------------------------------------------------------- //

ArmPlanner::ArmPlanner(
  const rclcpp::Node::SharedPtr & node, const std::string & group, const std::string & tip_link)
: node_(node), group_(group), tip_link_(tip_link),
  mg_(
    node, group, std::make_shared<tf2_ros::Buffer>(node->get_clock()),
    rclcpp::Duration::from_seconds(30.0))
{
  if (!tip_link_.empty()) {
    mg_.setEndEffectorLink(tip_link_);
  }
  // 位姿目标 -> IK 用（move_group 侧解，客户端没有运动学插件）
  ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
}

void ArmPlanner::configure(double planning_time, int attempts, double vel_scale, double acc_scale)
{
  planning_time_ = planning_time;
  attempts_ = attempts;
  mg_.setPlanningTime(planning_time_);
  mg_.setNumPlanningAttempts(static_cast<unsigned int>(attempts_));
  mg_.setMaxVelocityScalingFactor(vel_scale);
  mg_.setMaxAccelerationScalingFactor(acc_scale);
}

void ArmPlanner::applyPlanner(const PlannerSpec & spec)
{
  // 只设 planner id 不会切管线（chomp / stomp 各自是独立管线），两个字段要一起设。
  if (!spec.pipeline.empty()) {
    mg_.setPlanningPipelineId(spec.pipeline);
  }
  if (!spec.planner_id.empty()) {
    mg_.setPlannerId(spec.planner_id);
  }
}

const PlannerSpec & ArmPlanner::cartesianFallbackSpec() const
{
  for (const auto & spec : planners_) {
    if (spec.pipeline == "ompl") {
      return spec;
    }
  }
  return planners_.front();
}

bool ArmPlanner::planWithChain(
  moveit_msgs::msg::RobotTrajectory & trajectory, std::string & log, bool skip_joint_only)
{
  // 目标已经是位姿约束时，只吃关节目标的规划器（CHOMP/PTP）必然被 move_group 拒掉
  // （INVALID_GOAL_CONSTRAINTS），跳过它们省一次往返；但全跳完就退回完整链（不静默无动作）。
  std::vector<PlannerSpec> specs;
  if (skip_joint_only) {
    for (const auto & spec : planners_) {
      if (!needsJointGoal(spec)) {
        specs.push_back(spec);
      }
    }
  }
  if (specs.empty()) {
    specs = planners_;
  }

  std::ostringstream oss;
  for (const auto & spec : specs) {
    applyPlanner(spec);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto t0 = std::chrono::steady_clock::now();
    const auto code = mg_.plan(plan);
    const double dt =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      trajectory = plan.trajectory;
      RCLCPP_INFO(
        node_->get_logger(), "[规划成功] %s 规划器=%s（%.2fs / %zu 点）", diagnostic_context_.c_str(),
        toString(spec).c_str(), dt, trajectory.joint_trajectory.points.size());
      log.clear();
      return true;
    }
    oss << toString(spec) << " 失败(" << moveItFailure(code.val) << ", " << dt << "s); ";
  }
  log = oss.str();
  RCLCPP_WARN(
    node_->get_logger(), "[规划失败] %s 规划器链全部失败: %s", diagnostic_context_.c_str(), log.c_str());
  return false;
}

std::vector<double> ArmPlanner::currentJoints() const
{
  return const_cast<moveit::planning_interface::MoveGroupInterface &>(mg_).getCurrentJointValues();
}

bool ArmPlanner::currentStateValid(std::string & detail) const
{
  if (!validity_client_->service_is_ready()) { detail = "碰撞服务不可用"; return false; }
  const auto state = const_cast<moveit::planning_interface::MoveGroupInterface &>(mg_)
    .getCurrentState(2.0);
  if (!state) { detail = "当前机器人状态不可用"; return false; }
  auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  req->group_name = "arm_BOTH";
  req->robot_state.joint_state.name = allJoints();
  for (const auto & name : req->robot_state.joint_state.name)
    req->robot_state.joint_state.position.push_back(state->getVariablePosition(name));
  req->robot_state.is_diff = true;
  auto future = validity_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    detail = "碰撞检查超时";
    return false;
  }
  const auto response = future.get();
  if (response->valid) { detail.clear(); return true; }
  if (response->contacts.empty()) { detail = "状态碰撞但未返回接触对"; return false; }
  const auto & contact = response->contacts.front();
  detail = contact.contact_body_1 + "<->" + contact.contact_body_2;
  return false;
}

Eigen::Isometry3d ArmPlanner::currentTipPose() const
{
  const auto pose =
    const_cast<moveit::planning_interface::MoveGroupInterface &>(mg_).getCurrentPose(tip_link_);
  return fromMsg(pose.pose);
}

void ArmPlanner::logMotionSnapshot(const std::string & label,
  const Eigen::Isometry3d * target, const std::vector<double> * joint_goal) const
{
  if (!rcutils_logging_logger_is_enabled_for(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) return;
  const auto current = mg_.getCurrentState(2.0);
  if (!current) {
    RCLCPP_DEBUG(node_->get_logger(), "[运动状态] %s %s 无法读取关节状态", diagnostic_context_.c_str(), label.c_str());
    return;
  }
  moveit::core::RobotState state(*current);
  state.update();
  const auto & names = state.getRobotModel()->getVariableNames();
  std::vector<double> positions(state.getVariablePositions(), state.getVariablePositions() + names.size());
  RCLCPP_DEBUG(node_->get_logger(), "[运动状态] %s %s TCP=%s 完整机器人%s",
    diagnostic_context_.c_str(), label.c_str(), poseDescription(state.getGlobalLinkTransform(tip_link_)).c_str(),
    jointDescription(names, positions).c_str());
  if (target) RCLCPP_DEBUG(node_->get_logger(), "[请求目标] %s TCP=%s（关节终点由IK/规划求解）",
    diagnostic_context_.c_str(), poseDescription(*target).c_str());
  if (joint_goal) {
    const auto & own_names = mg_.getActiveJoints();
    RCLCPP_DEBUG(node_->get_logger(), "[请求目标] %s %s", diagnostic_context_.c_str(),
      jointDescription(own_names, *joint_goal).c_str());
    if (own_names.size() == joint_goal->size()) {
      state.setVariablePositions(own_names, *joint_goal);
      state.update();
      RCLCPP_DEBUG(node_->get_logger(), "[请求目标FK] %s TCP=%s", diagnostic_context_.c_str(),
        poseDescription(state.getGlobalLinkTransform(tip_link_)).c_str());
    }
  }
}

void ArmPlanner::logTrajectoryEndpoints(const moveit_msgs::msg::RobotTrajectory & trajectory) const
{
  if (!rcutils_logging_logger_is_enabled_for(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG)) return;
  const auto current = mg_.getCurrentState(2.0);
  const auto & jt = trajectory.joint_trajectory;
  if (!current || jt.points.empty()) return;
  moveit::core::RobotState state(*current);
  for (const auto & endpoint : {std::make_pair("规划起点", &jt.points.front()),
                                std::make_pair("规划终点", &jt.points.back())}) {
    if (endpoint.second->positions.size() != jt.joint_names.size()) continue;
    state.setVariablePositions(jt.joint_names, endpoint.second->positions);
    state.update();
    RCLCPP_DEBUG(node_->get_logger(), "[轨迹端点] %s %s TCP=%s %s",
      diagnostic_context_.c_str(), endpoint.first,
      poseDescription(state.getGlobalLinkTransform(tip_link_)).c_str(),
      jointDescription(jt.joint_names, endpoint.second->positions).c_str());
  }
}

bool ArmPlanner::needsIkFirst() const
{
  return std::any_of(
    planners_.begin(), planners_.end(), [](const PlannerSpec & spec) { return needsJointGoal(spec); });
}

bool ArmPlanner::resolvePoseToJoints(
  const Eigen::Isometry3d & target, const std::vector<double> & seed,
  std::vector<double> & out, std::string & err, bool geometry_only)
{
  if (ik_client_ == nullptr || !ik_client_->service_is_ready()) {
    err = "/compute_ik 不可用";
    return false;
  }
  const std::vector<std::string> & names = mg_.getActiveJoints();
  if (names.empty()) {
    err = "规划组 " + group_ + " 没有活动关节";
    return false;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  auto & ikr = req->ik_request;
  ikr.group_name = group_;
  ikr.ik_link_name = tip_link_;
  ikr.timeout.sec = geometry_only ? 0 : 1;
  ikr.timeout.nanosec = geometry_only ? 150000000u : 0u;
  ikr.pose_stamped.header.frame_id = kPlanningFrame;
  ikr.pose_stamped.pose = toMsg(target);

  // 种子：优先用调用者给的（dry_run 链式规划时是上一段终点），尺寸对不上就用当前关节
  std::vector<double> seed_joints = seed;
  const std::vector<double> current = currentJoints();
  if (seed_joints.size() != names.size()) {
    seed_joints = current;
  }
  sensor_msgs::msg::JointState js;
  js.name = names;
  js.position = seed_joints;
  ikr.robot_state.joint_state = js;
  ikr.robot_state.is_diff = true;

  auto solve_with = [this, &req](bool avoid_collisions) -> std::shared_ptr<moveit_msgs::srv::GetPositionIK::Response> {
    req->ik_request.avoid_collisions = avoid_collisions;
    auto future = ik_client_->async_send_request(req);
    if (future.wait_for(std::chrono::duration<double>(5.0)) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  };

  // 优化器专用请求先求运动学目标；端点和整条路径仍由 STOMP/MoveIt
  // 验证碰撞。不可达目标无需再做一次关闭碰撞的相同 IK 求解。
  auto res = solve_with(!geometry_only);
  if (!geometry_only && res != nullptr && res->error_code.val != 1) {
    RCLCPP_INFO(
      node_->get_logger(), "[IK回退] %s 碰撞检查IK失败：%s；下一步=纯运动学IK求目标，仍须规划和碰撞验证",
      diagnostic_context_.c_str(), moveItFailure(res->error_code.val).c_str());
    res = solve_with(false);
  }
  if (res == nullptr) {
    err = "IK 服务超时";
    return false;
  }
  if (res->error_code.val != 1) {
    err = "IK 失败：" + moveItFailure(res->error_code.val);
    return false;
  }

  std::map<std::string, double> solved;
  const auto & sol = res->solution.joint_state;
  for (size_t i = 0; i < sol.name.size() && i < sol.position.size(); ++i) {
    solved[sol.name[i]] = sol.position[i];
  }
  out.clear();
  for (const auto & name : names) {
    const auto it = solved.find(name);
    if (it == solved.end()) {
      err = "IK 解缺少关节 " + name;
      return false;
    }
    out.push_back(it->second);
  }
  // UR 关节允许若干圈等价角。优先保留距真实当前状态最近且满足限值的
  // 等价解，避免短距离目标被 IK 表示成 2π 绕行。
  const auto & reference = virtual_start_ ? seed_joints : current;
  const auto model = mg_.getRobotModel();
  if (model && reference.size() == out.size()) {
    for (size_t i = 0; i < out.size(); ++i) {
      const auto * joint = model->getJointModel(names[i]);
      if (!joint || joint->getVariableBounds().empty()) continue;
      const auto & bounds = joint->getVariableBounds().front();
      double best = out[i];
      for (int wrap = -2; wrap <= 2; ++wrap) {
        const double candidate = out[i] + wrap * 2.0 * M_PI;
        if (bounds.position_bounded_ &&
            (candidate < bounds.min_position_ - 1e-6 ||
             candidate > bounds.max_position_ + 1e-6)) continue;
        if (std::abs(candidate - reference[i]) < std::abs(best - reference[i]))
          best = candidate;
      }
      out[i] = best;
    }
  }
  return true;
}

bool ArmPlanner::hasIk(const Eigen::Isometry3d & target, std::string & err)
{
  const auto current = currentJoints();
  if (current.size() != 6) {
    err = "无法获取当前 6 关节状态";
    return false;
  }
  std::vector<std::vector<double>> seeds{current};
  for (const auto & change : std::vector<std::pair<size_t, double>>{
      {0, M_PI}, {0, -M_PI}, {2, 1.0}, {2, -1.0}, {5, M_PI}, {5, -M_PI}}) {
    auto seed = current;
    seed[change.first] += change.second;
    seeds.push_back(std::move(seed));
  }
  for (const auto & seed : seeds) {
    std::vector<double> solution;
    if (resolvePoseToJoints(target, seed, solution, err)) return true;
  }
  return false;
}

bool ArmPlanner::planPose(
  const Eigen::Isometry3d & target, const std::vector<double> & seed_joints,
  moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err,
  bool optimizer_only, const Eigen::Isometry3d * lens_in_tcp, double vertical_tolerance,
  const std::vector<std::vector<double>> & rejected_endpoints)
{
  // 每次请求单独设置路径约束，返回时清除，避免约束泄漏到装配/回 home。
  struct ClearPathConstraints {
    moveit::planning_interface::MoveGroupInterface & group;
    ~ClearPathConstraints()
    {
      group.clearPathConstraints();
      group.clearTrajectoryConstraints();
    }
  } clear_constraints{mg_};
  mg_.clearPathConstraints();
  mg_.clearTrajectoryConstraints();
  if (lens_in_tcp) {
    const Eigen::Isometry3d start = currentTipPose();
    const Eigen::Vector3d normal_in_tcp = lens_in_tcp->linear().col(2).normalized();
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = kPlanningFrame;
    oc.link_name = tip_link_;
    oc.orientation = toMsg(start).orientation;
    oc.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
    // 标准角度容差全部开放；工作区 STOMP 插件使用下面的标量镜面约束。
    oc.absolute_x_axis_tolerance = M_PI + 0.001;
    oc.absolute_y_axis_tolerance = M_PI + 0.001;
    oc.absolute_z_axis_tolerance = M_PI + 0.001;
    oc.weight = 1.0;
    moveit_msgs::msg::Constraints constraints;
    std::ostringstream marker;
    marker.precision(17);
    marker << "space_sim_mirror_vertical " << normal_in_tcp.x() << " "
           << normal_in_tcp.y() << " " << normal_in_tcp.z() << " " << vertical_tolerance;
    constraints.name = marker.str();
    constraints.orientation_constraints.push_back(oc);
    mg_.setPathConstraints(constraints);
    RCLCPP_DEBUG(node_->get_logger(),
      "[路径约束] %s STOMP镜面竖直：abs(normal_world.z) <= %.4f，法向旋转和水平朝向自由",
      diagnostic_context_.c_str(), vertical_tolerance);
  }
  if (optimizer_only && lens_in_tcp) {
    // 给 STOMP 提供平移/旋转同时插补的初值，避免默认关节直线初值
    // 大幅违反镜面路径约束。该参考轨迹仅用于初始化，从不直接执行；
    // 即便含碰撞，也必须由 STOMP 优化并经完整碰撞/姿态复检后才能执行。
    moveit_msgs::msg::RobotTrajectory reference;
    const double fraction = mg_.computeCartesianPath({toMsg(target)}, 0.005, reference, false);
    if (fraction >= 1.0 - 1e-9 && reference.joint_trajectory.points.size() > 1) {
      moveit_msgs::msg::TrajectoryConstraints seed;
      const auto & jt = reference.joint_trajectory;
      for (const auto & point : jt.points) {
        moveit_msgs::msg::Constraints waypoint;
        for (size_t j = 0; j < jt.joint_names.size(); ++j) {
          moveit_msgs::msg::JointConstraint joint;
          joint.joint_name = jt.joint_names[j];
          joint.position = point.positions[j];
          waypoint.joint_constraints.push_back(joint);
        }
        seed.constraints.push_back(std::move(waypoint));
      }
      mg_.setTrajectoryConstraints(seed);
      std::map<std::string, double> endpoint;
      for (size_t j = 0; j < jt.joint_names.size(); ++j)
        endpoint[jt.joint_names[j]] = jt.points.back().positions[j];
      std::vector<double> joints;
      for (const auto & name : mg_.getActiveJoints()) joints.push_back(endpoint.at(name));
      RCLCPP_DEBUG(node_->get_logger(),
        "[规划初值] %s 使用 %zu 点同时平移/旋转参考初始化 STOMP（参考不直接执行）",
        diagnostic_context_.c_str(), jt.points.size());
      std::string seed_err;
      if (planJoints(joints, trajectory, seed_err, true) &&
          checkTaskSpacePath(trajectory, 0.0, lens_in_tcp, vertical_tolerance, seed_err))
        return true;
      RCLCPP_WARN(node_->get_logger(), "[回退] %s STOMP参考初值或路径验证失败：%s；下一步=不同IK终点，仍用STOMP",
        diagnostic_context_.c_str(), seed_err.c_str());
      mg_.clearTrajectoryConstraints();
    } else {
      RCLCPP_INFO(node_->get_logger(), "[规划初值回退] %s 笛卡尔参考完成率=%.1f%%；下一步=不同IK终点+STOMP默认初值，参考不执行",
        diagnostic_context_.c_str(), fraction * 100.0);
    }
  }
  // 保留固定顺序的 IK 支路作为不同终点；STOMP 对完整轨迹优化约束代价。
  const std::vector<double> current = currentJoints();
  std::vector<std::vector<double>> seeds;
  if (!seed_joints.empty()) seeds.push_back(seed_joints);
  seeds.push_back(current);
  if (current.size() >= 6) {
    for (const auto & change : std::vector<std::pair<size_t, double>>{
        {0, M_PI}, {0, -M_PI}, {2, 1.0}, {2, -1.0}, {5, M_PI}, {5, -M_PI}}) {
      auto seed = current;
      seed[change.first] += change.second;
      seeds.push_back(std::move(seed));
    }
  }
  std::vector<std::vector<double>> attempted = rejected_endpoints;
  std::string failures;
  size_t seed_index = 0;
  for (const auto & seed : seeds) {
    ++seed_index;
    RCLCPP_DEBUG(node_->get_logger(), "[IK种子] %s 候选=%zu/%zu %s",
      diagnostic_context_.c_str(), seed_index, seeds.size(),
      jointDescription(mg_.getActiveJoints(), seed).c_str());
    std::vector<double> joints;
    std::string ik_err;
    if (!resolvePoseToJoints(target, seed, joints, ik_err, optimizer_only)) {
      failures += "IK: " + ik_err + "; ";
      RCLCPP_WARN(node_->get_logger(), "[IK候选失败] %s 种子=%zu/%zu 原因=%s；下一步=%s",
        diagnostic_context_.c_str(), seed_index, seeds.size(), ik_err.c_str(),
        seed_index < seeds.size() ? "下一固定IK种子" : "返回当前目标失败");
      continue;
    }
    bool same = false;
    for (const auto & old : attempted) {
      if (old.size() != joints.size()) continue;
      double d = 0.0;
      for (size_t j = 0; j < joints.size(); ++j)
        d = std::max(d, std::abs(old[j] - joints[j]));
      if (d < 1e-3) { same = true; break; }
    }
    if (same) continue;
    attempted.push_back(joints);
    std::string plan_err;
    if (planJoints(joints, trajectory, plan_err, optimizer_only)) {
      // 一个 IK 支路的 STOMP 轨迹不满足镜面竖直时，继续其他支路，
      // 不把首个无碰撞但姿态无效的轨迹交给执行器。
      if ((!lens_in_tcp ||
          checkTaskSpacePath(trajectory, 0.0, lens_in_tcp, vertical_tolerance, plan_err)) &&
          (!approach_check_ || validateFollowingApproach(trajectory, plan_err)))
        return true;
    }
    failures += plan_err + "; ";
    RCLCPP_WARN(node_->get_logger(), "[终点候选失败] %s IK种子=%zu/%zu 原因=%s；下一步=其他未尝试IK解",
      diagnostic_context_.c_str(), seed_index, seeds.size(), plan_err.c_str());
  }
  err = failures.empty() ? "没有可用 IK 解" : failures;
  return false;
}

bool ArmPlanner::validateFollowingApproach(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err)
{
  if (!approach_check_) return true;
  const auto check = *approach_check_;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = 1023;
  auto future = scene_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    err = "接近段预检无法读取规划场景"; return false;
  }
  planning_scene::PlanningScene scene(mg_.getRobotModel());
  scene.setPlanningSceneMsg(future.get()->scene);
  // Only the reserved lens/dock is an intentional contact in this preview.
  scene.getAllowedCollisionMatrixNonConst().setEntry(
    "space_sim_carried_lens_" + std::to_string(check.lens), group_ + "_gripper_dock", true);
  moveit_msgs::msg::RobotTrajectory approach;
  setVirtualStartFromTrajectory(trajectory);
  struct RestoreStart { ArmPlanner & planner; ~RestoreStart() { planner.resetStartStateToCurrent(); } } restore{*this};
  // Generate an IK preview; every resulting state is checked below against
  // the authoritative geometry, ACM and execution clearance before acceptance.
  const double fraction = cartesianTo(check.target, .002, false, approach, err);
  if (!err.empty() || fraction < .999 || approach.joint_trajectory.points.empty()) {
    err = "后续2cm接近段预检失败，完成率=" + std::to_string(fraction) + "; " + err;
    return false;
  }
  auto state = scene.getCurrentState();
  auto previous = approach.joint_trajectory.points.front().positions;
  for (const auto & point : approach.joint_trajectory.points) {
    double gap = 0.;
    for (size_t j = 0; j < previous.size(); ++j) gap = std::max(gap, std::abs(point.positions[j]-previous[j]));
    const int samples = std::max(1, static_cast<int>(std::ceil(gap/.01)));
    for (int k = 0; k <= samples; ++k) {
    for (size_t j = 0; j < approach.joint_trajectory.joint_names.size(); ++j)
      state.setVariablePosition(approach.joint_trajectory.joint_names[j],
        previous[j] + (point.positions[j]-previous[j])*k/samples);
    state.update();
    const double distance = multi_arm_motion::minimumDistance(scene, state, group_, check.clearance + .001);
    if (!state.satisfiesBounds(state.getJointModelGroup(group_)) || distance < check.clearance || std::isnan(distance)) {
      err = "后续2cm接近段IK分支间距/限位预检失败，间距=" + std::to_string(distance) +
        " m，要求=" + std::to_string(check.clearance) + " m";
      return false;
    }
    }
    previous = point.positions;
  }
  return true;
}

bool ArmPlanner::planJoints(
  const std::vector<double> & joints, moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & err, bool optimizer_only)
{
  logMotionSnapshot("关节规划请求（含IK求得的目标）", nullptr, &joints);
  if (!mg_.setJointValueTarget(joints)) {
    err = "关节目标无效";
    return false;
  }
  std::string log;
  if (optimizer_only) {
    applyPlanner({"stomp", "space_sim_stomp/StompPlanner"});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = mg_.plan(plan);
    if (code.val != 1) {
      err = "STOMP 规划失败：" + moveItFailure(code.val);
      return false;
    }
    trajectory = std::move(plan.trajectory);
    RCLCPP_INFO(node_->get_logger(), "[规划成功] %s 规划器=STOMP-only（%zu 点）",
      diagnostic_context_.c_str(), trajectory.joint_trajectory.points.size());
    return true;
  }
  if (planWithChain(trajectory, log)) {
    return true;
  }
  std::string direct_err;
  RCLCPP_WARN(node_->get_logger(), "[参考生成回退] %s 静态STOMP失败：%s；下一步=关节空间确定性直线插值参考+逐段碰撞检查；hybrid后端仍须动态STOMP优化和MPC复检",
    diagnostic_context_.c_str(), log.c_str());
  if (group_ != "arm_BOTH" && !virtual_start_ &&
      planCollisionCheckedDirect(joints, trajectory, direct_err)) {
    RCLCPP_INFO(node_->get_logger(), "[参考生成成功] %s 使用=关节空间确定性直线插值，已通过静态碰撞检查；执行验证尚未完成",
      diagnostic_context_.c_str());
    return true;
  }
  err = "STOMP 关节目标规划失败: " + log +
    (direct_err.empty() ? "" : "；确定性直连: " + direct_err);
  return false;
}

bool ArmPlanner::planCollisionCheckedDirect(
  const std::vector<double> & target, moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & err)
{
  const ArmSpec * spec = findArm(group_);
  if (!spec || target.size() != 6 || !validity_client_->service_is_ready()) {
    err = "关节状态或双臂碰撞服务不可用";
    return false;
  }
  const auto current = mg_.getCurrentState(2.0);
  if (!current) { err = "没有当前完整机器人状态"; return false; }
  const auto names = allJoints();
  std::vector<double> base;
  base.reserve(names.size());
  for (const auto & name : names) base.push_back(current->getVariablePosition(name));
  std::vector<double> start;
  for (const auto & name : spec->joints) start.push_back(current->getVariablePosition(name));
  double max_delta = 0.0;
  for (size_t j = 0; j < 6; ++j)
    max_delta = std::max(max_delta, std::abs(target[j] - start[j]));
  const int segments = std::max(2, static_cast<int>(std::ceil(max_delta / 0.015)));
  if (segments > 600) { err = "关节直连超过采样上限"; return false; }
  std::string last_collision;
  const auto valid_at = [&](const std::vector<double> & own) {
    auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    req->group_name = "arm_BOTH";
    req->robot_state.joint_state.name = names;
    req->robot_state.joint_state.position = base;
    for (size_t j = 0; j < spec->joints.size(); ++j) {
      const auto it = std::find(names.begin(), names.end(), spec->joints[j]);
      if (it == names.end()) return false;
      req->robot_state.joint_state.position[std::distance(names.begin(), it)] = own[j];
    }
    req->robot_state.is_diff = true;
    auto future = validity_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      last_collision = "碰撞服务超时";
      return false;
    }
    const auto response = future.get();
    if (response->valid) return true;
    if (!response->contacts.empty()) {
      const auto & c = response->contacts.front();
      last_collision = c.contact_body_1 + "<->" + c.contact_body_2;
    } else {
      last_collision = "状态无效但未返回接触对";
    }
    return false;
  };
  moveit_msgs::msg::RobotTrajectory direct;
  direct.joint_trajectory.joint_names = spec->joints;
  for (int i = 0; i <= segments; ++i) {
    const double u = static_cast<double>(i) / segments;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(6);
    for (size_t j = 0; j < 6; ++j)
      point.positions[j] = start[j] + u * (target[j] - start[j]);
    if (!valid_at(point.positions)) {
      err = "采样点 " + std::to_string(i) + "/" + std::to_string(segments) +
        " 未通过: " + last_collision;
      return false;
    }
    direct.joint_trajectory.points.push_back(std::move(point));
  }
  if (!prepareTrajectory(direct, err)) return false;
  const auto & points = direct.joint_trajectory.points;
  for (size_t i = 0; i < points.size(); ++i) {
    if (!valid_at(points[i].positions)) {
      err = "平滑后轨迹点 " + std::to_string(i) + " 未通过: " + last_collision;
      return false;
    }
    if (i == 0) continue;
    double gap = 0.0;
    for (size_t j = 0; j < 6; ++j)
      gap = std::max(gap, std::abs(points[i].positions[j] - points[i-1].positions[j]));
    const int checks = std::max(1, static_cast<int>(std::ceil(gap / 0.015)));
    for (int k = 1; k < checks; ++k) {
      std::vector<double> between(6);
      for (size_t j = 0; j < 6; ++j)
        between[j] = points[i-1].positions[j] +
          (points[i].positions[j] - points[i-1].positions[j]) * k / checks;
      if (!valid_at(between)) {
        err = "平滑后轨迹段 " + std::to_string(i) + " 未通过: " + last_collision;
        return false;
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(),
    "[规划] %s STOMP 未找到路径，使用通过 %d 段 MoveIt 双臂碰撞检查的确定性关节直线",
    group_.c_str(), segments);
  trajectory = std::move(direct);
  return true;
}

bool ArmPlanner::resolvePoseByPlanning(
  const Eigen::Isometry3d & target, std::vector<double> & joints, std::string & err)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!planPose(target, {}, trajectory, err) || trajectory.joint_trajectory.points.empty())
    return false;
  joints = trajectory.joint_trajectory.points.back().positions;
  return true;
}

bool ArmPlanner::prepareTrajectory(moveit_msgs::msg::RobotTrajectory & traj, std::string & err)
{
  auto & jt = traj.joint_trajectory;
  if (jt.points.empty() || jt.joint_names.empty()) {
    err = "规划器返回空轨迹";
    return false;
  }
  // CHOMP 可能输出连续重复点；先去掉它们，再做速度、加速度和 jerk 限制。
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> unique;
  for (const auto & p : jt.points) {
    if (p.positions.size() != jt.joint_names.size()) {
      err = "轨迹点关节数不一致";
      return false;
    }
    double max_delta = 0.0;
    if (!unique.empty()) {
      for (size_t j = 0; j < p.positions.size(); ++j)
        max_delta = std::max(max_delta, std::abs(p.positions[j] - unique.back().positions[j]));
    }
    if (unique.empty() || max_delta > 1e-7) unique.push_back(p);
  }
  if (unique.size() < 2) {
    err = "轨迹没有有效运动";
    return false;
  }
  jt.points = std::move(unique);
  const auto state = mg_.getCurrentState(2.0);
  if (!state) {
    err = "时间参数化时无法取得机器人状态";
    return false;
  }
  robot_trajectory::RobotTrajectory rt(mg_.getRobotModel(), group_);
  rt.setRobotTrajectoryMsg(*state, traj);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(
      rt, mg_.getMaxVelocityScalingFactor(), mg_.getMaxAccelerationScalingFactor())) {
    // CHOMP 在窄空间可能产生很小的折返；TOTG 对精确 180 度折返拒绝赋时。
    // 每个点设为零速度，并按三次插值的速度/加速度上界给保守时长，
    // 再交给 Ruckig 作 jerk 受限平滑；缺任何关节限值时拒绝执行。
    double elapsed = 0.0;
    jt.points.front().time_from_start = rclcpp::Duration(0, 0);
    jt.points.front().velocities.assign(jt.joint_names.size(), 0.0);
    jt.points.front().accelerations.assign(jt.joint_names.size(), 0.0);
    for (size_t i = 1; i < jt.points.size(); ++i) {
      double dt = 0.02;
      for (size_t j = 0; j < jt.joint_names.size(); ++j) {
        const auto * model = mg_.getRobotModel()->getJointModel(jt.joint_names[j]);
        if (!model || model->getVariableBounds().empty()) {
          err = "缺少关节限制: " + jt.joint_names[j];
          return false;
        }
        const auto & bounds = model->getVariableBounds().front();
        if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_ ||
            bounds.max_velocity_ <= 0.0 || bounds.max_acceleration_ <= 0.0) {
          err = "缺少速度或加速度限制: " + jt.joint_names[j];
          return false;
        }
        const double dq = std::abs(jt.points[i].positions[j] - jt.points[i-1].positions[j]);
        const double v = bounds.max_velocity_ * mg_.getMaxVelocityScalingFactor();
        const double a = bounds.max_acceleration_ * mg_.getMaxAccelerationScalingFactor();
        dt = std::max(dt, 1.5 * dq / v);
        dt = std::max(dt, std::sqrt(6.0 * dq / a));
      }
      elapsed += dt;
      jt.points[i].time_from_start = rclcpp::Duration::from_seconds(elapsed);
      jt.points[i].velocities.assign(jt.joint_names.size(), 0.0);
      jt.points[i].accelerations.assign(jt.joint_names.size(), 0.0);
    }
    rt.setRobotTrajectoryMsg(*state, traj);
    RCLCPP_WARN(node_->get_logger(), "TOTG 拒绝折返路径，采用保守速度/加速度赋时后交给 Ruckig");
  }
  if (!trajectory_processing::RuckigSmoothing::applySmoothing(
      rt, 1.0, 1.0, true)) {
    err = "Ruckig 平滑失败";
    return false;
  }
  rt.getRobotTrajectoryMsg(traj);
  double last = -1.0;
  for (const auto & p : traj.joint_trajectory.points) {
    const double t = static_cast<double>(p.time_from_start.sec) +
      static_cast<double>(p.time_from_start.nanosec) * 1e-9;
    if (!std::isfinite(t) || t <= last) {
      err = "平滑后轨迹时间戳不严格递增";
      return false;
    }
    last = t;
  }
  return true;
}

bool ArmPlanner::checkCollisionPath(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err) const
{
  const auto current = mg_.getCurrentState(2.0);
  const auto & jt = trajectory.joint_trajectory;
  if (!current || jt.points.empty() || !validity_client_->service_is_ready()) {
    err = "STOMP 平滑后碰撞复检缺少状态、轨迹或服务";
    return false;
  }
  const auto names = allJoints();
  auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  req->group_name = "arm_BOTH";
  req->robot_state.is_diff = true;
  req->robot_state.joint_state.name = names;
  for (const auto & name : names)
    req->robot_state.joint_state.position.push_back(current->getVariablePosition(name));
  std::vector<size_t> indices;
  for (const auto & name : jt.joint_names) {
    const auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) { err = "碰撞复检未知关节 " + name; return false; }
    indices.push_back(std::distance(names.begin(), it));
  }
  for (size_t i = 0; i < jt.points.size(); ++i) {
    const auto & end = jt.points[i].positions;
    const auto & start = jt.points[i == 0 ? 0 : i - 1].positions;
    if (end.size() != indices.size() || start.size() != indices.size()) {
      err = "碰撞复检轨迹关节维度错误"; return false;
    }
    double gap = 0.0;
    for (size_t j = 0; j < indices.size(); ++j)
      gap = std::max(gap, std::abs(end[j] - start[j]));
    const int samples = std::max(1, static_cast<int>(std::ceil(gap / 0.015)));
    for (int k = 1; k <= samples; ++k) {
      for (size_t j = 0; j < indices.size(); ++j)
        req->robot_state.joint_state.position[indices[j]] =
          start[j] + (end[j] - start[j]) * k / samples;
      auto future = validity_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        err = "STOMP 平滑后碰撞复检服务超时"; return false;
      }
      const auto response = future.get();
      if (!response->valid) {
        err = "STOMP 平滑后轨迹碰撞，段 " + std::to_string(i);
        if (!response->contacts.empty()) {
          const auto & c = response->contacts.front();
          err += ": " + c.contact_body_1 + "<->" + c.contact_body_2;
        }
        return false;
      }
    }
  }
  return true;
}

bool ArmPlanner::checkTaskSpacePath(
  const moveit_msgs::msg::RobotTrajectory & trajectory, double axis_tolerance_m,
  const Eigen::Isometry3d * lens_in_tcp, double vertical_tolerance,
  std::string & err) const
{
  const auto current = mg_.getCurrentState(2.0);
  if (!current) { err = "无法读取轨迹的 FK 初始状态"; return false; }
  moveit::core::RobotState state(*current);
  const auto & jt = trajectory.joint_trajectory;
  if (jt.points.empty()) { err = "空轨迹"; return false; }
  std::vector<Eigen::Isometry3d> tips;
  tips.reserve(jt.points.size());
  for (size_t i = 0; i < jt.points.size(); ++i) {
    const auto & point = jt.points[i];
    if (point.positions.size() != jt.joint_names.size()) {
      err = "轨迹关节维度错误"; return false;
    }
    state.setVariablePositions(jt.joint_names, point.positions);
    state.update();
    const Eigen::Isometry3d tip = state.getGlobalLinkTransform(tip_link_);
    if (lens_in_tcp != nullptr) {
      const Eigen::Vector3d normal = (tip * (*lens_in_tcp)).linear().col(2);
      if (std::abs(normal.z()) > vertical_tolerance) {
        err = "STOMP 轨迹使镜面偏离竖直姿态";
        return false;
      }
      if (i > 0) {
        const auto & previous = jt.points[i - 1].positions;
        double gap = 0.0;
        for (size_t j = 0; j < point.positions.size(); ++j)
          gap = std::max(gap, std::abs(point.positions[j] - previous[j]));
        const int samples = std::max(1, static_cast<int>(std::ceil(gap / 0.01)));
        for (int k = 1; k < samples; ++k) {
          std::vector<double> between(point.positions.size());
          for (size_t j = 0; j < between.size(); ++j)
            between[j] = previous[j] + (point.positions[j] - previous[j]) * k / samples;
          state.setVariablePositions(jt.joint_names, between);
          state.update();
          const auto intermediate_normal =
            (state.getGlobalLinkTransform(tip_link_) * (*lens_in_tcp)).linear().col(2).eval();
          if (std::abs(intermediate_normal.z()) > vertical_tolerance) {
            err = "STOMP 轨迹段间镜面偏离竖直姿态";
            return false;
          }
        }
      }
    }
    tips.push_back(tip);
  }
  if (axis_tolerance_m > 0.0) {
    const Eigen::Vector3d a = tips.front().translation();
    const Eigen::Vector3d b = tips.back().translation();
    const Eigen::Vector3d delta = b - a;
    for (const auto & tip : tips) {
      const double u = delta.squaredNorm() > 1e-12 ?
        std::clamp((tip.translation() - a).dot(delta) / delta.squaredNorm(), 0.0, 1.0) : 0.0;
      if ((tip.translation() - (a + u * delta)).norm() > axis_tolerance_m) {
        err = "STOMP 轨迹离开接口轴线容差";
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<ExecutionHandle> ArmPlanner::executeAsync(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  auto handle = std::make_shared<ExecutionHandle>();
  handle->owner_ = this;
  handle->timeout_s_ = execute_timeout_s_;

  auto promise = std::make_shared<std::promise<moveit::core::MoveItErrorCode>>();
  handle->future_ = promise->get_future().share();
  // 值捕获副本：避免后台线程引用已释放的对象
  moveit_msgs::msg::RobotTrajectory copy = trajectory;
  std::string timing_err;
  if (!prepareTrajectory(copy, timing_err)) {
    RCLCPP_ERROR(node_->get_logger(), "拒绝执行无效轨迹: %s", timing_err.c_str());
    promise->set_value(moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN);
    return handle;
  }
  std::thread(
    [this, promise, handle, copy]() {
      moveit::core::MoveItErrorCode code = moveit::core::MoveItErrorCode::FAILURE;
      try {
        code = mg_.execute(copy);
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "执行抛出异常: %s", e.what());
      }
      try {
        promise->set_value(code);
      } catch (const std::future_error &) {
        // 主线程已放弃等待，忽略
      }
    })
    .detach();
  return handle;
}

bool ArmPlanner::execute(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err)
{
  auto handle = executeAsync(trajectory);
  if (!handle->wait(execute_timeout_s_)) {
    err = handle->error();
    return false;
  }
  if (!handle->ok()) {
    err = handle->error();
    return false;
  }
  return true;
}

void ArmPlanner::repairTrajectoryTiming(moveit_msgs::msg::RobotTrajectory & traj)
{
  // ---------------------------------------------------------------------------
  // 为什么需要：ros2_control 的 JointTrajectoryController 要求
  // time_from_start **严格递增**，而优化型规划器（chomp/stomp）的输出根本没做时间参数化。
  // 实测（2026-09-26，chomp + Ruckig，/display_planned_path 抓下来的轨迹）：
  //   101 个点里只有末点有 18.62 s，前面全部是 0 → JTC 直接拒绝：
  //   "Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively"
  //   ⇒ 执行返回 code -4（CONTROL_FAILED），两臂都不动。
  // 本机 MoveIt 2.12 可用的响应适配器只有 TOTG / Ruckig：
  //   * TOTG：对含折返/重复点的路径直接抛
  //     "The path requires a 180 deg. turn, which is not supported by the current implementation"
  //     （chomp 报告 “Chomp path is collision free” 之后流水线仍判 FAILURE，白烧 17 s）；
  //   * Ruckig：要求输入轨迹已带时长，否则平滑后的时间戳仍然是 0（就是上面这个坑）。
  //   * ISP（AddIterativeSplineParameterization）在本版本已不提供，只剩头文件也没有。
  // 所以在这里自己保证“可执行”：去重复点 + 必要时按关节速度上限重新赋时。
  // ompl / Pilz 的轨迹本来就有合法时间戳，本函数对它们是 no-op。
  // ---------------------------------------------------------------------------
  auto & jt = traj.joint_trajectory;
  const size_t before = jt.points.size();
  if (before < 2) {
    return;
  }

  // ① 去掉与上一点完全相同的重复点（零长度段）
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> pts;
  pts.reserve(before);
  for (const auto & p : jt.points) {
    if (!pts.empty() && p.positions.size() == pts.back().positions.size()) {
      double dmax = 0.0;
      for (size_t j = 0; j < p.positions.size(); ++j) {
        dmax = std::max(dmax, std::fabs(p.positions[j] - pts.back().positions[j]));
      }
      if (dmax < 1e-9) {
        continue;
      }
    }
    pts.push_back(p);
  }
  if (pts.size() < 2) {
    return;
  }

  // ② 时间戳严格递增就直接用
  bool monotone = true;
  for (size_t i = 1; i < pts.size(); ++i) {
    if (rclcpp::Duration(pts[i].time_from_start) <=
        rclcpp::Duration(pts[i - 1].time_from_start))
    {
      monotone = false;
      break;
    }
  }
  jt.points = pts;
  if (monotone) {
    return;
  }

  // ③ 重新赋时：每段 dt = max_j |dq_j| / (v_max_j * velocity_scale)
  std::vector<double> vmax(jt.joint_names.size(), 1.0);
  const auto model = mg_.getRobotModel();
  if (model) {
    for (size_t j = 0; j < jt.joint_names.size(); ++j) {
      const auto * jm = model->getJointModel(jt.joint_names[j]);
      if (jm == nullptr) {
        continue;
      }
      const auto & bounds = jm->getVariableBounds();
      if (!bounds.empty() && bounds[0].max_velocity_ > 0.0) {
        vmax[j] = bounds[0].max_velocity_;
      }
    }
  }
  const double scale = mg_.getMaxVelocityScalingFactor() > 0.0 ? mg_.getMaxVelocityScalingFactor() : 1.0;
  double t = 0.0;
  double dq_max = 0.0;
  double dt_max = 0.0;
  jt.points.front().time_from_start = rclcpp::Duration(0, 0);
  for (size_t i = 1; i < jt.points.size(); ++i) {
    double dt = 1e-3;
    for (size_t j = 0; j < jt.points[i].positions.size(); ++j) {
      const double dq =
        std::fabs(jt.points[i].positions[j] - jt.points[i - 1].positions[j]);
      dq_max = std::max(dq_max, dq);
      const double v = (j < vmax.size() ? vmax[j] : 1.0) * scale;
      if (v > 1e-6) {
        dt = std::max(dt, dq / v);
      }
    }
    t += dt;
    dt_max = std::max(dt_max, dt);
    jt.points[i].time_from_start = rclcpp::Duration::from_seconds(t);
    // 速度/加速度不再可信，清掉（JTC 用 splines 插值只吃位置）
    jt.points[i].velocities.clear();
    jt.points[i].accelerations.clear();
    jt.points[i].effort.clear();
  }
  jt.points.front().velocities.clear();
  jt.points.front().accelerations.clear();
  jt.points.front().effort.clear();
  RCLCPP_WARN(
    node_->get_logger(),
    "[执行] %s 轨迹时间戳非严格递增（规划器输出未做时间参数化）-> 已按关节速度上限重新赋时："
    "%zu 点（去重后 %zu），总时长 %.2f s，最大单段 |dq|=%.3f rad / dt=%.2f s",
    group_.c_str(), before, jt.points.size(), t, dq_max, dt_max);
}

double ArmPlanner::cartesianTo(
  const Eigen::Isometry3d & target, double eef_step, bool avoid_collisions,
  moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err)
{  // 先试解析式直线规划器（Pilz LIN）：它把位置与姿态同步线性插值，且确定性求解，
  // 对“旋转 + 平移”这种混合段比 computeCartesianPath 稳。
  if (!line_pipeline_.empty() && !line_planner_id_.empty()) {
    const std::string save_pipeline = mg_.getPlanningPipelineId();
    const std::string save_planner = mg_.getPlannerId();
    mg_.setPlanningPipelineId(line_pipeline_);
    mg_.setPlannerId(line_planner_id_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = kPlanningFrame;
    pose.pose = toMsg(target);
    mg_.setPoseTarget(pose, tip_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto code = mg_.plan(plan);
    mg_.setPlanningPipelineId(save_pipeline);
    mg_.setPlannerId(save_planner);
    if (code == moveit::core::MoveItErrorCode::SUCCESS) {
      trajectory = plan.trajectory;
      RCLCPP_INFO(
        node_->get_logger(), "[直线] %s/%s 求解成功（%zu 点）", line_pipeline_.c_str(),
        line_planner_id_.c_str(), trajectory.joint_trajectory.points.size());
      return 1.0;
    }
    RCLCPP_WARN(
      node_->get_logger(), "[直线] %s/%s 求解失败(code %d)，回退 computeCartesianPath",
      line_pipeline_.c_str(), line_planner_id_.c_str(), code.val);
  }

  applyPlanner(cartesianFallbackSpec());
  std::vector<geometry_msgs::msg::Pose> waypoints{toMsg(target)};
  const double fraction =
    mg_.computeCartesianPath(waypoints, eef_step, trajectory, avoid_collisions);
  RCLCPP_DEBUG(node_->get_logger(), "[笛卡尔计算] %s 方法=computeCartesianPath 步长=%.4f m 碰撞检查=%s 完成率=%.1f%% 点数=%zu",
    diagnostic_context_.c_str(), eef_step, avoid_collisions ? "开启" : "关闭", fraction * 100.0,
    trajectory.joint_trajectory.points.size());
  if (fraction >= 0.0 && fraction < 1.0) {
    RCLCPP_WARN(node_->get_logger(), "[笛卡尔路径不完整] %s 完成率=%.1f%%；此接口未返回中断点原因，不能区分IK不可达、跳变或碰撞；是否执行前缀由当前阶段决定",
      diagnostic_context_.c_str(), fraction * 100.0);
  }
  if (fraction < 0.0) {
    err = "笛卡尔路径计算失败";
    return 0.0;
  }
  return fraction;
}

std::map<std::string, double> ArmPlanner::namedStateValues(const std::string & name) const
{
  auto & mg = const_cast<moveit::planning_interface::MoveGroupInterface &>(mg_);
  auto values = mg.getNamedTargetValues(name);
  if (!values.empty()) {
    return values;
  }
  // MoveGroupInterface 的全名形如 "arm_A_home"，SRDF 里的名字是 "home"，两种都试一遍
  for (const auto & candidate : mg.getNamedTargets()) {
    const bool suffix_match =
      candidate.size() > name.size() &&
      candidate.compare(candidate.size() - name.size(), name.size(), name) == 0;
    if (candidate == name || candidate == group_ + "/" + name || suffix_match) {
      values = mg.getNamedTargetValues(candidate);
      if (!values.empty()) {
        return values;
      }
    }
  }
  return {};
}

std::vector<std::string> ArmPlanner::namedTargets() const
{
  return const_cast<moveit::planning_interface::MoveGroupInterface &>(mg_).getNamedTargets();
}

void ArmPlanner::setVirtualStartFromTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto & jt = trajectory.joint_trajectory;
  if (jt.points.empty() || jt.joint_names.empty()) {
    return;
  }
  moveit_msgs::msg::RobotState state;
  state.joint_state.name = jt.joint_names;
  state.joint_state.position = jt.points.back().positions;
  state.is_diff = true;  // 只覆盖本组关节，其余保持当前状态
  mg_.setStartState(state);
  virtual_start_ = true;
}

void ArmPlanner::resetStartStateToCurrent()
{
  virtual_start_ = false;
  mg_.setStartStateToCurrentState();
}

}  // namespace space_sim_task_control
