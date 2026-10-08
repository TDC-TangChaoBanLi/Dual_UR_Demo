// 规划器登记表：把「规划器名」翻译成 MoveIt 的 (planning_pipeline_id, planner_id)。
//
// 为什么需要：`MoveGroupInterface::setPlannerId()` **只设置 planner id，不会切管线**；
// 想用 chomp / stomp 必须同时 `setPlanningPipelineId()`（它们各自是独立管线）。本表把两个字段
// 绑在一起，避免漏设。
//
// ⚠️ 本包**不定义任何规划器参数**：`chomp.*` / `stomp.*` / Pilz 的 `cartesian_limits` 等一律写在
// `{env}_moveit_config/config/planning_pipelines.yaml` 里，由 move_group 节点读取；本包只负责
// 选择「用哪条管线 + 哪个 planner id」，并把这两个字段放进 MotionPlanRequest。
#pragma once

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace space_sim_task_control
{

/// 一条规划管线 + 其中的 planner id（planner_id 为空 = 用管线默认插件）
struct PlannerSpec
{
  std::string pipeline;    ///< 请求里的 planning_pipeline
  std::string planner_id;  ///< 请求里的 planner_id
};

/// 规划器名 -> PlannerSpec 的对照表（顺序 = 报错时展示的顺序）
inline const std::vector<std::pair<std::string, PlannerSpec>> & plannerAliases()
{
  static const std::vector<std::pair<std::string, PlannerSpec>> kAliases{
    {"chomp", {"chomp", "chomp_interface/CHOMPPlanner"}},
    {"stomp", {"stomp", "space_sim_stomp/StompPlanner"}},
    {"pilz", {"pilz_industrial_motion_planner", "PTP"}},
    {"ptp", {"pilz_industrial_motion_planner", "PTP"}},
    {"lin", {"pilz_industrial_motion_planner", "LIN"}},
  };
  return kAliases;
}

inline std::string toString(const PlannerSpec & spec)
{
  if (spec.pipeline.empty()) {
    return spec.planner_id.empty() ? "(move_group 默认)" : spec.planner_id;
  }
  return spec.pipeline + "/" + (spec.planner_id.empty() ? "(默认插件)" : spec.planner_id);
}

inline std::string describePlannerChain(const std::vector<PlannerSpec> & chain)
{
  if (chain.empty()) {
    return "(move_group 默认)";
  }
  std::ostringstream os;
  for (size_t i = 0; i < chain.size(); ++i) {
    os << (i == 0 ? "" : " -> ") << toString(chain[i]);
  }
  return os.str();
}

inline std::string supportedPlannerNames()
{
  std::ostringstream os;
  for (const auto & kv : plannerAliases()) {
    os << kv.first << " ";
  }
  os << "(也支持原生的 pipeline:planner_id 写法，如 chomp:chomp_interface/CHOMPPlanner；"
        "none/default/空 = 不指定，用 move_group 默认)";
  return os.str();
}

/// 解析规划器名（大小写不敏感）。空串 / none / default -> 空 spec（= 不设置，用 move_group 默认）。
inline bool resolvePlannerName(const std::string & raw, PlannerSpec & out)
{
  std::string name = raw;
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  // 去掉首尾空白
  const auto first = name.find_first_not_of(" \t");
  const auto last = name.find_last_not_of(" \t");
  name = (first == std::string::npos) ? "" : name.substr(first, last - first + 1);

  if (name.empty() || name == "none" || name == "default") {
    out = PlannerSpec{};
    return true;
  }
  for (const auto & kv : plannerAliases()) {
    if (kv.first == name) {
      out = kv.second;
      return true;
    }
  }
  const auto colon = name.find(':');
  if (colon != std::string::npos && colon > 0) {
    out.pipeline = name.substr(0, colon);
    out.planner_id = name.substr(colon + 1);
    return out.pipeline == "chomp" || out.pipeline == "stomp" ||
           out.pipeline == "pilz_industrial_motion_planner";
  }
  return false;
}

/// 该规划器是不是**只接受关节空间目标**（位姿约束会被直接拒掉）。
///
/// 实测（2026-09-23，MoveIt 2.12）：`chomp` 收到位姿约束时 move_group 直接报
/// `[moveit.planners.chomp.planner]: Only joint-space goals are supported` +
/// `INVALID_GOAL_CONSTRAINTS`；Pilz PTP 也要求关节目标。
/// 因此 `ArmPlanner::planPose` 在链里出现这些规划器时先用 `/compute_ik` 解出关节目标。
inline bool needsJointGoal(const PlannerSpec & spec)
{
  if (spec.pipeline == "chomp") {
    return true;
  }
  // Pilz 的 PTP 是关节空间规划；LIN / CIRC 是笛卡尔（接受位姿约束）
  return spec.pipeline == "pilz_industrial_motion_planner" && spec.planner_id == "PTP";
}

/// 主规划器 + 兜底规划器 -> 依次尝试的规划器链（去重、保序）。
///
/// 规则：
///   * primary 必须能解析，否则返回 false（err 里列出可选值）；
///   * fallback 为空 = 只试 primary（不回退）；非空时同样必须能解析；
///   * 允许 CHOMP、STOMP 与 Pilz；不追加 RRT 等随机搜索规划器。
inline bool buildPlannerChain(
  const std::string & primary, const std::string & fallback, std::vector<PlannerSpec> & out,
  std::string & err)
{
  out.clear();
  PlannerSpec p;
  if (!resolvePlannerName(primary, p)) {
    err = "未知规划器 '" + primary + "'，可选：" + supportedPlannerNames();
    return false;
  }
  if (!(p.pipeline.empty() && p.planner_id.empty())) {
    out.push_back(p);
  }

  if (!fallback.empty()) {
    PlannerSpec f;
    if (!resolvePlannerName(fallback, f)) {
      err = "未知兜底规划器 '" + fallback + "'，可选：" + supportedPlannerNames();
      return false;
    }
    if (!(f.pipeline.empty() && f.planner_id.empty())) {
      out.push_back(f);
    }
  }

  std::vector<PlannerSpec> unique;
  for (const auto & spec : out) {
    const bool duplicate = std::any_of(
      unique.begin(), unique.end(), [&spec](const PlannerSpec & seen) {
        return seen.pipeline == spec.pipeline && seen.planner_id == spec.planner_id;
      });
    if (!duplicate) {
      unique.push_back(spec);
    }
  }
  out.swap(unique);
  return true;  // out 为空 = 用 move_group 默认（由 ArmPlanner 的默认链兜底）
}

}  // namespace space_sim_task_control
