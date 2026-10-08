// 目标位姿构造：把"抓哪片镜片 / 用哪个侧面抓 / 什么相位 / 装到哪个中心面"翻译成
// 一条条 TCP 目标位姿。**全部从 live site 位姿解析**，不依赖任何离线能力矩阵。
//
// 公式（与 dock 插件 grasp/default 组的 dock_target = Q_DOCK 语义配套）：
//   T_grasp    = T_lens_Sg · Q_DOCK · Rz(-phase)        # TCP 的 +z 指向镜片内部
//   T_pregrasp = T_grasp · offsetZ(-approach)           # 沿对接面法向退出
//   T_tcp<-lens= inv(T_grasp) · T_lensBody              # 抓稳后 TCP 与镜片的刚性关系
//   T_lensBody = T_centre_Sk · Q_DOCK · inv(T_S0_in_lensBody)
//   T_place    = T_lensBody · inv(T_tcp<-lens)
//   p_preplace = p_place + approach · z_centre_Sk (TCP 姿态保持 T_place)
//
// 退出存储区（一步笛卡尔：旋转 + 平移同步）：
//   位姿 = shortestRotationToDown(T_pregrasp) 的位置取 T_pregrasp 的位置，
//   再沿"外侧"（±x，符号取自臂基座 x 的符号）平移 storage_exit_dx。
#pragma once

#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "space_sim_task_control/site_pose_provider.hpp"
#include "space_sim_task_control/types.hpp"

namespace space_sim_task_control
{

/// 一次完整装配（抓取 → 抬起 → 移出 → 转运 → 插装 → 分离）的全部目标
struct AssemblyTask
{
  std::string arm;
  int lens = 0;
  std::string grasp_face;
  double phase_deg = 0.0;
  std::string centre_face;

  Eigen::Isometry3d pregrasp;      ///< 抓取前 0.02 m
  Eigen::Isometry3d grasp;         ///< 抓取对合位姿
  Eigen::Isometry3d preplace;      ///< 插装前 0.02 m
  Eigen::Isometry3d place;         ///< 插装对合位姿
  Eigen::Vector3d centre_axis_world = Eigen::Vector3d::UnitZ(); ///< 目标中心面的世界系 +z
  Eigen::Isometry3d exit_target;   ///< （单段模式）TCP +z 朝下 + 沿 ±x 平移，一步完成
  /// （分段模式）第一步：保持姿态，只沿外侧 ±x 平移。
  /// 必须先平移再旋转：镜片板绕 y 轴转会让远端上扫 ~0.26 m，直接顶进上方接口盘而卡死。
  Eigen::Isometry3d exit_translate;
  /// （分段模式）第二步：位置不动，只把 TCP 的 +z 转到世界 -z。
  Eigen::Isometry3d exit_rotate;
  Eigen::Isometry3d lens_in_tcp;   ///< 镜片体在 TCP link 系下的位姿（attached 碰撞体）
  double exit_dx = 0.0;

  std::string describe() const;
};

/// 沿目标中心面 site 的 +z 外退，TCP 姿态保持不变。
inline Eigen::Isometry3d placeStandoff(const AssemblyTask & task, double distance)
{
  Eigen::Isometry3d pose = task.place;
  pose.translation() += distance * task.centre_axis_world;
  return pose;
}

class TargetBuilder
{
public:
  TargetBuilder(SitePoseProvider * sites, double approach_m, double storage_exit_dx);

  /// 校验 god 插件对 frame_id 的处理是否可信：
  /// 用 T_world_body · T_body_site 与 T_world_site 比对（插件对未知 body 名会**静默返回 world 位姿**）
  bool verifyFrameConsistency(std::string & err);

  /// 某臂能抓该镜片的侧面（kSideFaces 里"法向指向本臂"的那些，按点积由大到小排序）
  bool graspFaces(
    const std::string & arm, int lens, std::vector<std::string> & out, std::string & err);

  /// 构造一条完整任务的目标位姿；失败时 err 说明是哪一次位姿查询失败
  bool build(
    const std::string & arm, int lens, const std::string & grasp_face, double phase_deg,
    const std::string & centre_face, AssemblyTask & out, std::string & err);

  double approach() const { return approach_m_; }
  double storageExitDx() const { return storage_exit_dx_; }

private:
  SitePoseProvider * sites_;
  double approach_m_;
  double storage_exit_dx_;
};

}  // namespace space_sim_task_control
