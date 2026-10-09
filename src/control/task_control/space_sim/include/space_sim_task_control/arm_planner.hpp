// MoveGroupInterface 封装：位姿目标规划（含 planner 兜底）、关节目标规划、笛卡尔直线、
// 以及**可抢占的异步执行**。
//
// 并发说明：MoveGroupInterface 的 plan()/execute() 都是**同步阻塞**的；每个实例内部有自己的
// action client 与回调，因此**每条臂各自持有独立的 node + executor 线程**。本类不做加锁，
// 不允许多线程同时调用同一实例的规划类接口（executeAsync 的后台线程只跑 execute，属例外）。
#pragma once

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "Eigen/Geometry"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "space_sim_task_control/planner_registry.hpp"
#include "space_sim_task_control/types.hpp"

namespace space_sim_task_control
{

class ArmPlanner;

/// 一次非阻塞执行的句柄：可等待、可抢占（stop）
class ExecutionHandle
{
public:
  /// 等待执行结束；返回 true = 在超时前结束（结果看 ok()）
  bool wait(double timeout_s);
  bool ok() const { return ok_; }
  const std::string & error() const { return error_; }
  bool finished() const;
  void stop();

private:
  friend class ArmPlanner;
  ArmPlanner * owner_{nullptr};
  std::shared_future<moveit::core::MoveItErrorCode> future_;
  double timeout_s_{0.0};
  std::atomic<bool> stop_requested_{false};
  bool ok_{false};
  std::string error_;
};

class ArmPlanner
{
public:
  ArmPlanner(
    const rclcpp::Node::SharedPtr & node, const std::string & group, const std::string & tip_link);

  void configure(double planning_time, int attempts, double vel_scale, double acc_scale);

  /// 执行超时（超过则 stop() 并报错）；0 = 不限制
  void setExecuteTimeout(double seconds) { execute_timeout_s_ = seconds; }

  /// 规划器链（按顺序尝试，失败才试下一个）。空 vector = 使用默认 STOMP。
  /// 每次尝试都会同时设置 planning_pipeline_id 与 planner_id —— 只设 planner id 不会切管线，
  /// CHOMP 是独立管线。规划器**参数**不在这里，见 {env}_moveit_config 的
  /// config/planning_pipelines.yaml。
  void setPlanners(std::vector<PlannerSpec> planners)
  {
    if (planners.empty()) {
      return;
    }
    planners_ = std::move(planners);
  }
  const std::vector<PlannerSpec> & planners() const { return planners_; }
  void setDiagnosticContext(const std::string & context) { diagnostic_context_ = context; }
  const std::string & diagnosticContext() const { return diagnostic_context_; }
  void logMotionSnapshot(const std::string & label,
    const Eigen::Isometry3d * target = nullptr, const std::vector<double> * joint_goal = nullptr) const;
  void logTrajectoryEndpoints(const moveit_msgs::msg::RobotTrajectory & trajectory) const;

  /// 直线段优先用的规划器。Pilz 的 LIN 是**解析式笛卡尔插补**（旋转与平移同步插值），
  /// 比 computeCartesianPath 稳，也天然适合“旋转 + 平移”这种混合段。
  /// pipeline_id / planner_id 为空则只用 computeCartesianPath。
  void setLinePlanner(const std::string & pipeline_id, const std::string & planner_id)
  {
    line_pipeline_ = pipeline_id;
    line_planner_id_ = planner_id;
  }

  /// 位姿目标：求确定性 IK 后使用 STOMP 优化关节轨迹。
  /// 镜面约束同时传入 STOMP 路径代价并在规划后复检。
  bool hasIk(const Eigen::Isometry3d & target, std::string & err);
  void checkFollowingApproach(const Eigen::Isometry3d & target, int lens, double clearance)
  { approach_check_ = ApproachCheck{target, lens, clearance}; }
  void clearFollowingApproach() { approach_check_.reset(); }

  bool planPose(
    const Eigen::Isometry3d & target, const std::vector<double> & seed_joints,
    moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err,
    bool optimizer_only = false, const Eigen::Isometry3d * lens_in_tcp = nullptr,
    double vertical_tolerance = 0.10,
    const std::vector<std::vector<double>> & rejected_endpoints = {});

  /// 关节目标（双臂同步规划时也用 arm_BOTH 组走这里）
  bool planJoints(
    const std::vector<double> & joints, moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & err, bool optimizer_only = false);

  /// 位姿 -> 关节解的**规划兜底**：用本臂自己的规划器发"位姿约束"目标做一次避障规划
  /// （MoveIt 管线自带 IK 采样 + 多次重启，比 /compute_ik 单种子稳健），取轨迹终点当关节目标。
  /// 用在 coupled 模式下 /compute_ik 解不出位姿时 —— 语义上这一段已经放宽成
  /// "只要不碰撞、不保证直线"（典型：抓取后沿存储面退出、以及移出存储区）。
  bool resolvePoseByPlanning(
    const Eigen::Isometry3d & target, std::vector<double> & joints, std::string & err);

  /// 执行一段轨迹（阻塞到结束或超时）
  bool execute(const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err);
  bool prepareTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err);
  bool checkCollisionPath(
    const moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err) const;
  bool checkTaskSpacePath(
    const moveit_msgs::msg::RobotTrajectory & trajectory, double axis_tolerance_m,
    const Eigen::Isometry3d * lens_in_tcp, double vertical_tolerance,
    std::string & err) const;

  /// 非阻塞执行：立即返回句柄（句柄内部用后台线程跑 execute）
  std::shared_ptr<ExecutionHandle> executeAsync(
    const moveit_msgs::msg::RobotTrajectory & trajectory);

  /// 从当前位姿沿直线到目标（用于 0.02 m 的接近/插入/退出），返回完成比例
  double cartesianTo(
    const Eigen::Isometry3d & target, double eef_step, bool avoid_collisions,
    moveit_msgs::msg::RobotTrajectory & trajectory, std::string & err);

  /// dry_run 下的「链式规划」：把上一段规划的终点（差值形式）作为下一段的起点，
  /// 这样即使不执行也能验证整条动作序列的连通性。
  void setVirtualStartFromTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory);
  /// 丢弃虚拟起点（回到“以当前实际状态为起点”）
  void resetStartStateToCurrent();
  bool hasVirtualStart() const { return virtual_start_; }

  std::vector<double> currentJoints() const;
  /// 使用 MoveIt 当前规划场景检查全双臂状态（含已附着/已安装镜片）。
  bool currentStateValid(std::string & detail) const;
  Eigen::Isometry3d currentTipPose() const;
  const std::string & group() const { return group_; }
  const std::string & tipLink() const { return tip_link_; }

  /// SRDF / 记忆里的命名状态（如 "arm_A_home"）→ 关节值映射；不存在时返回空 map。
  /// 名字兼容两种写法：SRDF 里的状态名（home）与 MoveGroupInterface 的全名（arm_A_home）。
  std::map<std::string, double> namedStateValues(const std::string & name) const;
  /// 可用的命名状态名（日志/兜底用）
  std::vector<std::string> namedTargets() const;

  moveit::planning_interface::MoveGroupInterface & mg() { return mg_; }

private:
  /// 把 spec 的两个字段写进 MoveGroupInterface（空串 = 不动该项）
  void applyPlanner(const PlannerSpec & spec);
  /// computeCartesianPath 用哪条 spec（只做插补/IK，planner 仅用于关节跳变修复）
  const PlannerSpec & cartesianFallbackSpec() const;
  /// 依次用规划器链规划（调用前目标已设好）；成功返回 true，失败把每次结果写进 log
  /// `skip_joint_only` = 跳过只吃关节目标的规划器（位姿约束路径用；全部跳完则退回完整链）
  bool planWithChain(
    moveit_msgs::msg::RobotTrajectory & trajectory, std::string & log,
    bool skip_joint_only = false);
  /// 链里是否有“只接受关节目标”的规划器（CHOMP / PTP）
  bool needsIkFirst() const;
  /// 用 /compute_ik（move_group 侧，客户端没有运动学插件）把位姿解成关节目标；
  /// 先要避障解，拿不到再退回不避障解。
  bool resolvePoseToJoints(
    const Eigen::Isometry3d & target, const std::vector<double> & seed,
    std::vector<double> & out, std::string & err, bool geometry_only = false);

  /// 发送前修正轨迹时间律（见 cpp 里的详细说明）：
  /// ① 去掉与上一点完全相同的重复点（零长度段会让 TOTG 抛 "180 deg. turn"）；
  /// ② 时间戳严格递增就直接用；
  /// ③ 否则按各关节的 velocity limit × velocity_scale 逐段重新赋时（JTC 要求严格递增）。
  void repairTrajectoryTiming(moveit_msgs::msg::RobotTrajectory & traj);
  bool planCollisionCheckedDirect(
    const std::vector<double> & target, moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & err);
  bool validateFollowingApproach(const moveit_msgs::msg::RobotTrajectory &, std::string &);
  struct ApproachCheck { Eigen::Isometry3d target; int lens; double clearance; };
  std::optional<ApproachCheck> approach_check_;

  rclcpp::Node::SharedPtr node_;
  std::string group_;
  std::string diagnostic_context_;
  std::string tip_link_;
  moveit::planning_interface::MoveGroupInterface mg_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  /// 默认只使用 STOMP 优化。
  std::vector<PlannerSpec> planners_{{"stomp", "space_sim_stomp/StompPlanner"}};
  std::string line_pipeline_;    ///< 直线段优先管线（如 pilz_industrial_motion_planner）
  std::string line_planner_id_;  ///< 直线段优先 planner id（如 LIN）
  double planning_time_ = 5.0;
  int attempts_ = 3;
  double execute_timeout_s_ = 120.0;
  bool virtual_start_ = false;
};

}  // namespace space_sim_task_control
