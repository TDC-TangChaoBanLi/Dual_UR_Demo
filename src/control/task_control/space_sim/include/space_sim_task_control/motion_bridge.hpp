// 单臂动作由各自的 MoveIt 规划器与轨迹控制器并发执行；
// 同时在途的两条轨迹仍按 arm_BOTH 的完整状态复检碰撞。
#pragma once

#include <map>
#include <mutex>
#include <chrono>
#include <functional>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_arm_motion_interfaces/action/execute_motion.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "space_sim_task_control/arm_planner.hpp"
#include "space_sim_task_control/types.hpp"

namespace space_sim_task_control
{

struct MotionRequest
{
  enum class Kind
  {
    kPose,       ///< 自由规划到位姿（target）
    kJoints,     ///< 自由规划到关节（joints）
    kCartesian   ///< 沿直线到位姿（target），要求完成比例 >= min_fraction
  };

  Kind kind = Kind::kPose;
  Eigen::Isometry3d target{Eigen::Isometry3d::Identity()};
  std::vector<double> joints;
  std::string what;
  bool avoid_collisions = true;
  double min_fraction = -1.0;  ///< <=0 时要求完整路径
  bool allow_partial_cartesian = false;  ///< 仅存储接口下降允许执行已通过检查的前缀
  double cartesian_step_m = 0.005;
  bool dry_run = false;
  double speed_scale = 1.0;  ///< 时间缩放，仅降低动态速度，不改几何轨迹
  double planning_time_s = 0.0;  ///< >0: 本段动态全局优化预算；0 使用默认值
  double axis_tolerance_m = 0.0;  ///< >0: CHOMP 轨迹必须在起终点连线管道内
  bool mirror_vertical = false;
  bool optimizer_only = false;  ///< 移出储存区只接受 STOMP，不使用插补或关节直连兜底
  Eigen::Isometry3d lens_in_tcp{Eigen::Isometry3d::Identity()};
  double vertical_tolerance = 0.10;  ///< 镜面法向的世界 z 分量上限
  std::function<bool(std::string &)> execution_guard;  ///< 任务层实时持片检查
};

class MotionBridge
{
public:
  virtual ~MotionBridge() = default;
  /// 执行本臂的一次运动；返回 false 时 err 说明原因
  virtual bool move(
    const std::string & arm, const MotionRequest & req, std::string & err) = 0;
  /// 不允许免碰撞检查的直线重试。
  virtual bool supportsCartesianFallback() const { return false; }
};

/// 每条臂各自用一个 ArmPlanner（各自 node/executor 线程）并发运行。
class DecoupledMotionBridge : public MotionBridge
{
public:
  void setPlanner(const std::string & arm, ArmPlanner * planner);
  void setNode(const rclcpp::Node::SharedPtr & node);
  void setControllerTimeout(double seconds) { controller_timeout_s_ = seconds; }
  bool hybridEnabled() const { return hybrid_; }
  bool resetExecutor(bool resume, std::string & err);
  void stopExecution();  ///< 持片丢失等场景失效时停止全部执行，不重置仿真或预测

  bool move(const std::string & arm, const MotionRequest & req, std::string & err) override;

private:
  using Follow = control_msgs::action::FollowJointTrajectory;
  struct Active {
    moveit_msgs::msg::RobotTrajectory trajectory;
    std::chrono::steady_clock::time_point started;
  };
  bool executeDirect(const std::string & arm, const MotionRequest & request,
                     const moveit_msgs::msg::RobotTrajectory & traj,
                     std::string & err);
  bool executeHybrid(const std::string & arm, const MotionRequest & request,
                     const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err);
  bool validateOverlap(const std::string & arm, const moveit_msgs::msg::RobotTrajectory & traj,
                       std::string & err);
  static std::vector<double> interpolate(const moveit_msgs::msg::RobotTrajectory & traj, double seconds);
  std::map<std::string, ArmPlanner *> planners_;
  std::map<std::string, rclcpp_action::Client<Follow>::SharedPtr> controllers_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_;
  std::mutex mutex_;
  std::map<std::string, Active> active_;
  double controller_timeout_s_ = 120.0;
  using Hybrid = multi_arm_motion_interfaces::action::ExecuteMotion;
  rclcpp_action::Client<Hybrid>::SharedPtr hybrid_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_, resume_, reset_predictions_;
  bool hybrid_ = false;
  double planning_time_ = 12.0, velocity_scale_ = .1, acceleration_scale_ = .1;
};

}  // namespace space_sim_task_control
