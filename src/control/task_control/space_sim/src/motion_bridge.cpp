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
  validity_ = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  for (const auto & arm : arms()) {
    const std::string action = "/" + arm.name + "_ur_joint_trajectory_controller/follow_joint_trajectory";
    controllers_[arm.name] = rclcpp_action::create_client<Follow>(node, action);
  }
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
  const std::string & arm, const moveit_msgs::msg::RobotTrajectory & traj, std::string & err)
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
  if (result.wait_for(timeout) != std::future_status::ready) {
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
        req.mirror_vertical ? &req.lens_in_tcp : nullptr, req.vertical_tolerance)) {
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
    if (executeDirect(arm, traj, err)) {
      planner.resetStartStateToCurrent();
      return true;
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
  err = "共享区域重新规划超过 4 次";
  return false;
}

}  // namespace space_sim_task_control
