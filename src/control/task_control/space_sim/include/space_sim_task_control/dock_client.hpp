// mujoco_dock_sim_plugin 的 dock / release 服务封装。
//
// 插件负责"容差判定 + weld 焊合"：本任务只负责把 TCP/镜片规划到对合位姿，
// 然后调用 dock 让插件在 0.01 m / 0.15 rad 容差内自动完成焊合。
// 服务是**阻塞式**的（等到焊合/超时），默认超时 = dock_time*3+5 = 6.5 s。
#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mujoco_dock_sim_plugin/srv/dock.hpp"
#include "mujoco_dock_sim_plugin/srv/release.hpp"

namespace space_sim_task_control
{

/// dock/release 的调用结果
struct DockResult
{
  bool success = false;
  std::string state;    ///< docked / already_docked / released / not_docked / timeout / rejected
  std::string message;
  double relpose_pos_err = 0.0;   ///< 实测相对位置误差（dock 成功时插件回传 relpose）
  double relpose_rot_err = 0.0;
};

class DockClient
{
public:
  using Dock = mujoco_dock_sim_plugin::srv::Dock;
  using Release = mujoco_dock_sim_plugin::srv::Release;

  explicit DockClient(const rclcpp::Node::SharedPtr & node, double call_timeout_s = 15.0);

  bool waitForServices(double timeout_s);

  /// group 为空时插件按字母序自动选组（tcp 相关会命中 grasp，镜片<->插口/中心命中 default）
  DockResult dock(
    const std::string & site1, const std::string & site2, const std::string & group = "",
    double timeout_s = 0.0);

  DockResult release(
    const std::string & site1, const std::string & site2, double timeout_s = 0.0);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Dock>::SharedPtr dock_client_;
  rclcpp::Client<Release>::SharedPtr release_client_;
  double call_timeout_s_;
};

}  // namespace space_sim_task_control
