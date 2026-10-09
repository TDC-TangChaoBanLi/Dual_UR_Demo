#include "space_sim_task_control/motion_bridge.hpp"
#include "space_sim_task_control/diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace space_sim_task_control
{
namespace
{
bool ensureCurrentState(ArmPlanner & planner, std::string & err)
{
  if (!planner.mg().getCurrentState(2.0)) {
    err = "无法获取当前关节状态（检查 /joint_states 与 use_sim_time）";
    return false;
  }
  return true;
}

double seconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

std::string trajectoryRecord(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  std::ostringstream out;
  out.precision(17);
  out << "{\"joints\":[";
  const auto & jt = trajectory.joint_trajectory;
  for (size_t i = 0; i < jt.joint_names.size(); ++i) {
    if (i) out << ',';
    out << '"' << jt.joint_names[i] << '"';
  }
  out << "],\"points\":[";
  for (size_t i = 0; i < jt.points.size(); ++i) {
    if (i) out << ',';
    out << "{\"t\":" << seconds(jt.points[i].time_from_start) << ",\"q\":[";
    for (size_t j = 0; j < jt.points[i].positions.size(); ++j) {
      if (j) out << ',';
      out << jt.points[i].positions[j];
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}
}  // namespace

void DecoupledMotionBridge::setNode(const rclcpp::Node::SharedPtr & node)
{
  const auto backend=node->declare_parameter<std::string>("motion_backend", "direct");
  if(backend!="direct"&&backend!="hybrid")throw std::invalid_argument("motion_backend must be direct or hybrid");
  hybrid_ = backend == "hybrid";
  planning_time_ = node->get_parameter("planning_time_s").as_double();
  velocity_scale_ = node->get_parameter("velocity_scale").as_double();
  acceleration_scale_ = node->get_parameter("acceleration_scale").as_double();
  if (hybrid_) {
    hybrid_client_ = rclcpp_action::create_client<Hybrid>(node, "/execute_motion");
    stop_ = node->create_client<std_srvs::srv::Trigger>("/stop_motion");
    resume_ = node->create_client<std_srvs::srv::Trigger>("/resume_motion");
    reset_predictions_ = node->create_client<std_srvs::srv::Trigger>("/reset_motion_predictions");
  }
  validity_ = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  for (const auto & arm : arms()) {
    const std::string action = "/" + arm.name + "_ur_joint_trajectory_controller/follow_joint_trajectory";
    controllers_[arm.name] = rclcpp_action::create_client<Follow>(node, action);
  }
}

void DecoupledMotionBridge::stopExecution()
{
  if (hybrid_) {
    stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  } else {
    for (const auto & entry : controllers_) entry.second->async_cancel_all_goals();
  }
}

bool DecoupledMotionBridge::resetExecutor(bool resume, std::string & err)
{
  if (!hybrid_) return true;
  const auto call = [&](const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client) {
    if (!client->wait_for_service(std::chrono::seconds(5))) {err = "MPC 生命周期服务不可用"; return false;}
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {err = "MPC 生命周期服务超时"; return false;}
    const auto result = future.get();
    if (!result->success) {err = result->message; return false;}
    return true;
  };
  if (resume) return call(resume_);
  if (!call(stop_)) return false;
  // Wait for goal cancellation and the last stopping command before a simulator reset.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  return call(reset_predictions_);
}

bool DecoupledMotionBridge::executeHybrid(
  const std::string & arm, const MotionRequest & request,
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err)
{
  if (!hybrid_client_->wait_for_action_server(std::chrono::seconds(10))) {
    err = "通用 /execute_motion action 不可用"; return false;
  }
  auto & planner = *planners_.at(arm);
  Hybrid::Goal goal;
  goal.reference = trajectory.joint_trajectory;
  goal.reference.header.stamp = builtin_interfaces::msg::Time{};
  if (goal.reference.points.empty()) {
    err = "规划返回空轨迹"; return false;
  }
  if (seconds(goal.reference.points.back().time_from_start) <= 0.0) {
    const auto & first = goal.reference.points.front().positions;
    for (const auto & point : goal.reference.points) {
      if (point.positions.size() != first.size()) {
        err = "零时长轨迹关节维度不一致"; return false;
      }
      for (size_t i = 0; i < first.size(); ++i) {
        if (!std::isfinite(point.positions[i]) ||
            std::abs(point.positions[i] - first[i]) > 1e-9) {
          err = "拒绝有位移的零时长轨迹"; return false;
        }
      }
    }
    auto hold = goal.reference.points.front();
    hold.time_from_start = builtin_interfaces::msg::Duration{};
    hold.velocities.assign(first.size(), 0.0);
    hold.accelerations.assign(first.size(), 0.0);
    goal.reference.points = {hold, hold};
    goal.reference.points.back().time_from_start.nanosec = 200000000;
  }
  goal.optimize_reference = request.kind != MotionRequest::Kind::kCartesian;
  goal.request.group_name = planner.group();
  const double global_budget=request.planning_time_s>0 ? request.planning_time_s : planning_time_;
  if(!std::isfinite(global_budget)||global_budget<=0||global_budget>30) {
    err="动态全局规划预算必须在 (0,30] 秒内";return false;
  }
  goal.request.allowed_planning_time = goal.optimize_reference ? global_budget : 3.0;
  goal.request.max_velocity_scaling_factor = velocity_scale_;
  goal.request.max_acceleration_scaling_factor = acceleration_scale_;
  goal.timeout = std::max(controller_timeout_s_,
    seconds(goal.reference.points.back().time_from_start)*3.0+global_budget+30.0);
  moveit_msgs::msg::Constraints target;
  for (size_t i=0;i<goal.reference.joint_names.size();++i) {
    moveit_msgs::msg::JointConstraint c;
    c.joint_name=goal.reference.joint_names[i];c.position=goal.reference.points.back().positions[i];
    c.tolerance_above=c.tolerance_below=1e-5;c.weight=1.0;
    target.joint_constraints.push_back(c);
  }
  goal.request.goal_constraints.push_back(target);
  if (request.mirror_vertical) {
    const Eigen::Vector3d normal=request.lens_in_tcp.linear().col(2);
    std::ostringstream name;name.precision(17);
    name << "link_normal_horizontal " << normal.x() << ' ' << normal.y() << ' ' << normal.z()
         << ' ' << request.vertical_tolerance;
    goal.request.path_constraints.name=name.str();
    moveit_msgs::msg::OrientationConstraint c;c.header.frame_id="world";c.link_name=planner.tipLink();
    c.orientation.w=1;c.absolute_x_axis_tolerance=c.absolute_y_axis_tolerance=c.absolute_z_axis_tolerance=M_PI;
    c.weight=1;goal.request.path_constraints.orientation_constraints.push_back(c);
  }
  if (request.kind == MotionRequest::Kind::kCartesian) {
    auto state=planner.mg().getCurrentState(2.0);
    if (!state) {err="笛卡尔执行约束无法读取当前状态";return false;}
    for(size_t i=0;i<goal.reference.joint_names.size();++i)
      state->setVariablePosition(goal.reference.joint_names[i],goal.reference.points.front().positions[i]);
    state->update();
    const auto start=state->getGlobalLinkTransform(planner.tipLink());
    for(size_t i=0;i<goal.reference.joint_names.size();++i)
      state->setVariablePosition(goal.reference.joint_names[i],goal.reference.points.back().positions[i]);
    state->update();
    const Eigen::Vector3d end=state->getGlobalLinkTransform(planner.tipLink()).translation();
    const Eigen::Vector3d delta=end-start.translation(),middle=(end+start.translation())*.5;
    const Eigen::Quaterniond rotation=delta.norm()>1e-9 ?
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),delta.normalized()) : Eigen::Quaterniond::Identity();
    moveit_msgs::msg::PositionConstraint c;c.header.frame_id="world";c.link_name=planner.tipLink();c.weight=1;
    shape_msgs::msg::SolidPrimitive cylinder;cylinder.type=shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions={delta.norm()+.004,.002};
    geometry_msgs::msg::Pose pose;pose.position.x=middle.x();pose.position.y=middle.y();pose.position.z=middle.z();
    pose.orientation.x=rotation.x();pose.orientation.y=rotation.y();pose.orientation.z=rotation.z();pose.orientation.w=rotation.w();
    c.constraint_region.primitives.push_back(cylinder);c.constraint_region.primitive_poses.push_back(pose);
    goal.request.path_constraints.position_constraints.push_back(c);
    if (!request.mirror_vertical) {
      const Eigen::Quaterniond orientation(start.linear());
      moveit_msgs::msg::OrientationConstraint fixed;
      fixed.header.frame_id="world";fixed.link_name=planner.tipLink();fixed.weight=1;
      fixed.orientation.x=orientation.x();fixed.orientation.y=orientation.y();
      fixed.orientation.z=orientation.z();fixed.orientation.w=orientation.w();
      fixed.absolute_x_axis_tolerance=fixed.absolute_y_axis_tolerance=fixed.absolute_z_axis_tolerance=.02;
      goal.request.path_constraints.orientation_constraints.push_back(fixed);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("space_sim_motion"),"[执行后端] %s %s + MPC（碰撞/路径约束复检）",
    planner.diagnosticContext().c_str(),goal.optimize_reference?"动态STOMP种子优化":"笛卡尔参考");
  auto accepted=hybrid_client_->async_send_goal(goal);
  if(accepted.wait_for(std::chrono::seconds(10))!=std::future_status::ready) {err="动态规划执行服务接受目标超时";return false;}
  auto handle=accepted.get();
  if(!handle) {err="动态规划执行服务拒绝目标（执行器停止、反馈、组占用或请求无效）";return false;}
  auto future=hybrid_client_->async_get_result(handle);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(goal.timeout+10);
  while(future.wait_for(std::chrono::milliseconds(200))!=std::future_status::ready) {
    if(request.execution_guard&&!request.execution_guard(err)) {
      hybrid_client_->async_cancel_goal(handle);return false;
    }
    if(std::chrono::steady_clock::now()>=deadline) {
      hybrid_client_->async_cancel_goal(handle);err="动态规划执行服务结果超时";return false;
    }
  }
  const auto result=future.get();
  if(result.code!=rclcpp_action::ResultCode::SUCCEEDED||!result.result||result.result->error_code.val!=1) {
    err=result.result?"动态规划执行失败 code="+std::to_string(result.result->error_code.val)+" "+result.result->message:"动态规划执行服务未返回结果";
    return false;
  }
  return true;
}

void DecoupledMotionBridge::setPlanner(const std::string & arm, ArmPlanner * planner)
{
  planners_[arm] = planner;
}

std::vector<double> DecoupledMotionBridge::interpolate(
  const moveit_msgs::msg::RobotTrajectory & traj, double at)
{
  const auto & pts = traj.joint_trajectory.points;
  if (pts.empty()) return {};
  if (at <= seconds(pts.front().time_from_start)) return pts.front().positions;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double end = seconds(pts[i].time_from_start);
    if (at <= end) {
      const double begin = seconds(pts[i - 1].time_from_start);
      const double alpha = std::clamp((at - begin) / std::max(end - begin, 1e-9), 0.0, 1.0);
      std::vector<double> q = pts[i].positions;
      for (size_t j = 0; j < q.size(); ++j) {
        q[j] = pts[i - 1].positions[j] * (1.0 - alpha) + pts[i].positions[j] * alpha;
      }
      return q;
    }
  }
  return pts.back().positions;
}

bool DecoupledMotionBridge::validateOverlap(
  const std::string & arm, const moveit_msgs::msg::RobotTrajectory & traj, std::string & err)
{
  const std::string peer = arm == "arm_A" ? "arm_B" : "arm_A";
  const auto it = active_.find(peer);
  if (it == active_.end()) return true;
  if (!validity_ || !validity_->service_is_ready()) {
    err = "两臂重叠执行时 /check_state_validity 不可用";
    return false;
  }
  const auto & mine = traj.joint_trajectory;
  const auto & other = it->second.trajectory.joint_trajectory;
  if (mine.points.empty() || other.points.empty()) {
    err = "无法验证空轨迹";
    return false;
  }
  const double duration = seconds(mine.points.back().time_from_start);
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - it->second.started).count();
  const double step = std::max(0.05, duration / 400.0);
  for (double t = 0.0; t <= duration + 1e-9; t += step) {
    const auto q1 = interpolate(traj, std::min(t, duration));
    const auto q2 = interpolate(it->second.trajectory, elapsed + t);
    if (q1.size() != mine.joint_names.size() || q2.size() != other.joint_names.size()) {
      err = "轨迹关节数不一致";
      return false;
    }
    auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    req->group_name = "arm_BOTH";
    req->robot_state.is_diff = true;
    req->robot_state.joint_state.name = mine.joint_names;
    req->robot_state.joint_state.position = q1;
    req->robot_state.joint_state.name.insert(
      req->robot_state.joint_state.name.end(), other.joint_names.begin(), other.joint_names.end());
    req->robot_state.joint_state.position.insert(
      req->robot_state.joint_state.position.end(), q2.begin(), q2.end());
    auto future = validity_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      err = "双臂轨迹碰撞检查服务超时";
      return false;
    }
    const auto response = future.get();
    if (!response->valid) {
      std::string contact = "未返回接触对";
      if (!response->contacts.empty()) {
        contact = response->contacts.front().contact_body_1 + "<->" +
          response->contacts.front().contact_body_2;
      }
      err = "双臂轨迹在 t=" + std::to_string(t) + " s 存在碰撞，接触对=" + contact +
        "；等待对侧离开共享区域后重规划";
      RCLCPP_DEBUG(rclcpp::get_logger("space_sim_motion"),
        "[碰撞复现状态] %s t=%.9f s %s", planners_.at(arm)->diagnosticContext().c_str(), t,
        jointDescription(req->robot_state.joint_state.name, req->robot_state.joint_state.position).c_str());
      return false;
    }
  }
  return true;
}

bool DecoupledMotionBridge::executeDirect(
  const std::string & arm, const MotionRequest & request,
  const moveit_msgs::msg::RobotTrajectory & traj, std::string & err)
{
  const auto client = controllers_.at(arm);
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    err = arm + " 的 FollowJointTrajectory action 不可用";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!validateOverlap(arm, traj, err)) return false;
    active_[arm] = Active{traj, std::chrono::steady_clock::now()};
  }
  RCLCPP_DEBUG(rclcpp::get_logger("space_sim_motion"),
    "[控制器开始] %s 点数=%zu，预计 %.2f s", planners_.at(arm)->diagnosticContext().c_str(),
    traj.joint_trajectory.points.size(),
    seconds(traj.joint_trajectory.points.back().time_from_start));
  Follow::Goal goal;
  goal.trajectory = traj.joint_trajectory;
  auto future = client->async_send_goal(goal);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    err = "控制器未接受目标（超时）";
    std::lock_guard<std::mutex> lock(mutex_);
    active_.erase(arm);
    return false;
  }
  auto handle = future.get();
  if (!handle) {
    err = "控制器拒绝目标";
    std::lock_guard<std::mutex> lock(mutex_);
    active_.erase(arm);
    return false;
  }
  auto result = client->async_get_result(handle);
  const auto planned_s = seconds(traj.joint_trajectory.points.back().time_from_start);
  const auto timeout = std::chrono::duration<double>(
    std::max(controller_timeout_s_, planned_s * 2.0 + 10.0));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool guard_failed = false;
  while (result.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < deadline) {
    if (request.execution_guard && !request.execution_guard(err)) {
      client->async_cancel_goal(handle);
      guard_failed = true;
      break;
    }
  }
  if (guard_failed) {
    // Keep the task guard's concrete failure reason.
  } else if (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    client->async_cancel_goal(handle);
    err = "控制器执行超时";
  } else {
    const auto wrapped = result.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped.result || wrapped.result->error_code != Follow::Result::SUCCESSFUL) {
      err = "控制器执行失败，code=" + std::to_string(
        wrapped.result ? wrapped.result->error_code : -1) + " detail=" +
        (wrapped.result ? wrapped.result->error_string : "未返回控制器结果");
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_.erase(arm);
  }
  RCLCPP_INFO(rclcpp::get_logger("space_sim_motion"),
    "[控制器结束] %s 结果=%s", planners_.at(arm)->diagnosticContext().c_str(),
    err.empty() ? "成功" : err.c_str());
  return err.empty();
}

bool DecoupledMotionBridge::move(
  const std::string & arm, const MotionRequest & req, std::string & err)
{
  const auto it = planners_.find(arm);
  if (it == planners_.end() || !it->second) {
    err = "没有为 " + arm + " 注册规划器";
    return false;
  }
  ArmPlanner & planner = *it->second;
  struct EndSnapshot {
    ArmPlanner & planner;
    ~EndSnapshot()
    {
      try { planner.logMotionSnapshot("阶段结束实测（成功或失败）"); }
      catch (const std::exception & e) {
        RCLCPP_DEBUG(rclcpp::get_logger("space_sim_motion"), "[状态日志失败] %s %s",
          planner.diagnosticContext().c_str(), e.what());
      }
    }
  } end_snapshot{planner};
  std::vector<std::vector<double>> rejected_endpoints;
  std::string last_global_failure;
  for (int attempt = 0; attempt < 4; ++attempt) {
    if (!planner.hasVirtualStart()) {
      if (!ensureCurrentState(planner, err)) return false;
      planner.resetStartStateToCurrent();
    }
    moveit_msgs::msg::RobotTrajectory traj;
    planner.logMotionSnapshot("规划前实测/尝试" + std::to_string(attempt + 1),
      req.kind == MotionRequest::Kind::kJoints ? nullptr : &req.target,
      req.kind == MotionRequest::Kind::kJoints ? &req.joints : nullptr);
    const bool cartesian = req.kind == MotionRequest::Kind::kCartesian;
    if (req.kind == MotionRequest::Kind::kJoints) {
      if (!planner.planJoints(req.joints, traj, err)) return false;
    } else if (cartesian && !req.dry_run) {
      if (!req.avoid_collisions) {
        err = "禁止无碰撞检查的笛卡尔执行";
        return false;
      }
      const double need = req.min_fraction > 0.0 ? req.min_fraction : 1.0;
      err.clear();
      const double fraction = planner.cartesianTo(req.target, req.cartesian_step_m, true, traj, err);
      if (!err.empty() || !std::isfinite(fraction) || fraction < 0.0) return false;
      if (fraction < need && !req.allow_partial_cartesian) {
        err = "笛卡尔路径不完整（IK或碰撞等原因，服务未返回具体原因），完成比例=" + std::to_string(fraction);
        return false;
      }
      if (req.allow_partial_cartesian) {
        RCLCPP_INFO(rclcpp::get_logger("space_sim_motion"),
          "[%s] %s：可执行下降直线比例 %.1f%%（不使用旋转或 STOMP 回退）",
          planner.diagnosticContext().c_str(), req.what.c_str(), fraction * 100.0);
        if (fraction == 0.0 || traj.joint_trajectory.points.size() < 2) {
          // 不执行空轨迹。后续移出仍必须重新规划并通过完整碰撞检查。
          err.clear();
          return true;
        }
      }
    } else if (!planner.planPose(
        req.target, req.joints, traj, err, req.optimizer_only,
        req.mirror_vertical ? &req.lens_in_tcp : nullptr, req.vertical_tolerance, rejected_endpoints)) {
      if (!last_global_failure.empty())
        err = last_global_failure + "；其他IK终点候选耗尽: " + err;
      return false;
    }
    if (req.dry_run) {
      planner.setVirtualStartFromTrajectory(traj);
      return true;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("space_sim_motion"), "[规划轨迹记录] %s data=%s",
      planner.diagnosticContext().c_str(), trajectoryRecord(traj).c_str());
    if (!planner.prepareTrajectory(traj, err)) return false;
    RCLCPP_DEBUG(rclcpp::get_logger("space_sim_motion"), "[平滑轨迹记录] %s data=%s",
      planner.diagnosticContext().c_str(), trajectoryRecord(traj).c_str());
    planner.logTrajectoryEndpoints(traj);
    if ((req.axis_tolerance_m > 0.0 || req.mirror_vertical) &&
        !planner.checkTaskSpacePath(
          traj, req.axis_tolerance_m, req.mirror_vertical ? &req.lens_in_tcp : nullptr,
          req.vertical_tolerance, err)) return false;
    if ((req.optimizer_only || req.allow_partial_cartesian) &&
        !planner.checkCollisionPath(traj, err)) return false;
    if (req.speed_scale < 1.0) {
      const double scale = std::clamp(req.speed_scale, 0.1, 1.0);
      for (auto & point : traj.joint_trajectory.points) {
        const double t = seconds(point.time_from_start) / scale;
        point.time_from_start.sec = static_cast<int32_t>(std::floor(t));
        point.time_from_start.nanosec = static_cast<uint32_t>(
          std::round((t - std::floor(t)) * 1e9));
        if (point.time_from_start.nanosec >= 1000000000u) {
          ++point.time_from_start.sec;
          point.time_from_start.nanosec -= 1000000000u;
        }
        for (auto & velocity : point.velocities) velocity *= scale;
        for (auto & acceleration : point.accelerations) acceleration *= scale * scale;
      }
    }
    err.clear();
    if (req.execution_guard && !req.execution_guard(err)) return false;
    if (hybrid_ ? executeHybrid(arm, req, traj, err) : executeDirect(arm, req, traj, err)) {
      if (req.execution_guard && !req.execution_guard(err)) return false;
      planner.resetStartStateToCurrent();
      return true;
    }
    if (hybrid_ && req.kind == MotionRequest::Kind::kPose &&
        (err.find("STOMP optimization failed; no planner fallback") != std::string::npos ||
         err.find("commit clearance violation") != std::string::npos ||
         err.find("commit collision") != std::string::npos)) {
      last_global_failure = err;
      rejected_endpoints.push_back(traj.joint_trajectory.points.back().positions);
      RCLCPP_WARN(rclcpp::get_logger("space_sim_motion"),
        "[规划回退] %s 原因=%s；下一步=其他确定性IK终点，仍用动态STOMP；未执行失败参考",
        planner.diagnosticContext().c_str(), err.c_str());
      continue;
    }
    // 只有按时间采样证实会撞上正在执行的对侧轨迹时才让行；
    // 对侧离开后必须用新的关节起点重新规划，而不是执行旧轨迹。
    if (err.rfind("双臂轨迹在 t=", 0) != 0) return false;
    const std::string peer = arm == "arm_A" ? "arm_B" : "arm_A";
    RCLCPP_WARN(rclcpp::get_logger("space_sim_motion"),
      "[碰撞协调回退] %s 原因=%s；下一步=等待在途 %s 离开共享区，再按实时状态重新规划",
      planner.diagnosticContext().c_str(), err.c_str(), peer.c_str());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (std::chrono::steady_clock::now() < deadline) {
      bool peer_active;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        peer_active = active_.count(peer) != 0;
      }
      if (!peer_active) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_.count(peer) != 0) {
        err = "对侧轨迹长时间占用共享区";
        return false;
      }
    }
    planner.resetStartStateToCurrent();
  }
  err = last_global_failure.empty() ? "共享区域重新规划超过 4 次" :
    last_global_failure + "；动态STOMP终点尝试达到4次上限";
  return false;
}

}  // namespace space_sim_task_control
