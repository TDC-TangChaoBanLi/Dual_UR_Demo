// 通过 mujoco_god_plugin 读取 MuJoCo 场景中对象的位姿。
//
// 用途：镜片是 MuJoCo 里的浮动体（freejoint），**不在 TF 树里**，只能由此查询。
// 坐标系：frame_id 为空 = MuJoCo 世界系（与 URDF 的 world/base_link 重合）；
//         frame_id 给某个 body 名时返回该 body 系下的位姿（例如查 site 在镜片体坐标系下的偏置）。
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "space_sim_task_control/types.hpp"
#include "mujoco_god_plugin/srv/get_object_pose.hpp"

namespace space_sim_task_control
{

class SitePoseProvider
{
public:
  using GetObjectPose = mujoco_god_plugin::srv::GetObjectPose;

  explicit SitePoseProvider(const rclcpp::Node::SharedPtr & node, double timeout_s = 5.0);

  bool waitForService(double timeout_s);
  std::optional<Eigen::Isometry3d> get(
    const std::string & type, const std::string & name, const std::string & frame_id = "");

  std::optional<Eigen::Isometry3d> lensBody(int lens);
  std::optional<Eigen::Isometry3d> lensFace(int lens, const std::string & face);
  std::optional<Eigen::Isometry3d> centreFace(const std::string & face);
  std::optional<Eigen::Isometry3d> tcpSite(const std::string & arm);

  /// 校验两个 site 是否已对合：返回 (位置误差, 姿态误差rad)，失败返回 nullopt
  std::optional<std::pair<double, double>> mateError(
    const std::string & site1, const std::string & site2,
    const Eigen::Isometry3d & dock_target = Eigen::Isometry3d(Eigen::Isometry3d::Identity()));

  int failures() const { return failures_; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<GetObjectPose>::SharedPtr client_;
  double timeout_s_;
  int failures_ = 0;
};

}  // namespace space_sim_task_control
