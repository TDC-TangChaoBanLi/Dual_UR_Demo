#include "space_sim_task_control/assembly_targets.hpp"

#include <algorithm>
#include <sstream>

namespace space_sim_task_control
{

std::string AssemblyTask::describe() const
{
  std::ostringstream os;
  os << "len_ring_" << lens << " 抓取面 " << grasp_face << " 相位 " << static_cast<int>(phase_deg)
     << "° -> " << centre_face;
  return os.str();
}

TargetBuilder::TargetBuilder(SitePoseProvider * sites, double approach_m, double storage_exit_dx)
: sites_(sites), approach_m_(approach_m), storage_exit_dx_(storage_exit_dx)
{
}

bool TargetBuilder::verifyFrameConsistency(std::string & err)
{
  const int lens = 1;
  const auto world_body = sites_->get("body", lensBodyName(lens));
  const auto world_site = sites_->get("site", lensFaceSite(lens, kStorageFace));
  const auto body_site = sites_->get("site", lensFaceSite(lens, kStorageFace), lensBodyName(lens));
  if (!world_body || !world_site || !body_site) {
    err = "god 位姿查询失败（仿真 / mujoco_god_plugin 没起？）";
    return false;
  }
  // T_body_site 在 body 系里应是固定的局部偏置；用它和世界位姿链式相乘应能复现 world_site。
  const Eigen::Isometry3d reconstructed = (*world_body) * (*body_site);
  const double pos_err = (reconstructed.translation() - world_site->translation()).norm();
  const double rot_err =
    Eigen::AngleAxisd(reconstructed.rotation() * world_site->rotation().transpose()).angle();
  if (pos_err > 1e-3 || std::abs(rot_err) > 1e-3) {
    std::ostringstream os;
    os << "god 插件的 frame_id 处理不可信：T_world_body·T_body_site 与 T_world_site 不一致"
       << "（位置误差 " << pos_err << " m / 姿态误差 " << rot_err
       << " rad）。常见原因：frame_id 传了 site 名或拼错的 body 名（插件会静默返回 world 位姿）";
    err = os.str();
    return false;
  }
  return true;
}

bool TargetBuilder::graspFaces(
  const std::string & arm, int lens, std::vector<std::string> & out, std::string & err)
{
  out.clear();
  const ArmSpec * spec = findArm(arm);
  if (spec == nullptr) {
    err = "未知机械臂: " + arm;
    return false;
  }
  std::vector<std::pair<double, std::string>> scored;
  for (const char * face : kSideFaces) {
    const auto pose = sites_->get("site", lensFaceSite(lens, face));
    if (!pose) {
      err = "查询位姿失败: " + lensFaceSite(lens, face);
      return false;
    }
    // "指向本臂" = 面外法向的**水平分量**与 (基座 - 该面) 的水平方向同向。
    // 只看水平分量：侧面分属两臂是绕竖直轴分左右半区的（S1/S2 vs S4/S5），
    // 而每个面本身还上下俯仰约 30°（下俯的 S2/S4 用 3D 点积会被误判为背对机械臂）。
    Eigen::Vector3d normal = normalOf(*pose);
    Eigen::Vector3d normal_xy(normal.x(), normal.y(), 0.0);
    const Eigen::Vector3d to_arm = spec->base_position - pose->translation();
    Eigen::Vector3d to_arm_xy(to_arm.x(), to_arm.y(), 0.0);
    if (to_arm_xy.norm() < 1e-6 || normal_xy.norm() < 1e-6) {
      continue;
    }
    const double dot = normal_xy.normalized().dot(to_arm_xy.normalized());
    if (dot > 0.0) {
      // 排序优先“退出旋转角小”的抓取面：TCP 的 +z 与面外法向反向，
      // 所以“把 TCP z 转到世界 -z”要转过的角度 = 面外法向与世界 +z 的夹角。
      // 上仰面优先尝试，但 S1/S5 装配需要的下俯面 S2/S4 也必须保留。
      scored.emplace_back(normal.z(), std::string(face));
    }
  }
  std::sort(
    scored.begin(), scored.end(),
    [](const auto & a, const auto & b) { return a.first > b.first; });
  for (const auto & s : scored) {
    out.push_back(s.second);
  }
  if (out.empty()) {
    err = "找不到朝向 " + arm + " 的抓取面（len_ring_" + std::to_string(lens) + "）";
    return false;
  }
  return true;
}

bool TargetBuilder::build(
  const std::string & arm, int lens, const std::string & grasp_face, double phase_deg,
  const std::string & centre_face, AssemblyTask & out, std::string & err)
{
  const ArmSpec * spec = findArm(arm);
  if (spec == nullptr) {
    err = "未知机械臂: " + arm;
    return false;
  }
  const auto lens_body = sites_->get("body", lensBodyName(lens));
  const auto s0_in_body = sites_->get("site", lensFaceSite(lens, kStorageFace), lensBodyName(lens));
  const auto grasp_site = sites_->get("site", lensFaceSite(lens, grasp_face));
  const auto centre_face_pose = sites_->get("site", centreSite(centre_face));
  if (!lens_body) {
    err = "查询位姿失败: body " + lensBodyName(lens);
    return false;
  }
  // 储存位合理性检查：镜片是被搬运的浮动体，一旦掉了/被撞飞，后续所有目标位姿都会
  // 跟着变成垃圾值（规划报一堆看不懂的失败）。这里先快速拦住。
  const Eigen::Vector3d p = lens_body->translation();
  if (std::abs(p.x()) > 0.2 || std::abs(p.y()) > 0.45 || p.z() < -0.95 || p.z() > -0.25) {
    std::ostringstream os;
    os << "镜片 " << lensBodyName(lens) << " 不在储存位（实测 " << fmt(*lens_body)
       << "）—— 很可能已经掉出或被撞飞，请重启仿真";
    err = os.str();
    return false;
  }
  if (!s0_in_body) {
    err = "查询位姿失败: site " + lensFaceSite(lens, kStorageFace) + " in " + lensBodyName(lens);
    return false;
  }
  if (!grasp_site) {
    err = "查询位姿失败: site " + lensFaceSite(lens, grasp_face);
    return false;
  }
  if (!centre_face_pose) {
    err = "查询位姿失败: site " + centreSite(centre_face);
    return false;
  }

  // ---- 抓取 ----
  const Eigen::Isometry3d phase = rotation(axisAngleZ(-phase_deg * M_PI / 180.0));
  out.grasp = (*grasp_site) * rotation(kQDock) * phase;
  out.pregrasp = out.grasp * offsetZ(-approach_m_);
  out.lens_in_tcp = out.grasp.inverse() * (*lens_body);

  // ---- 插装：让镜片的 S0 与中心面以 Q_DOCK 对合 ----
  const Eigen::Isometry3d lens_body_goal =
    (*centre_face_pose) * rotation(kQDock) * s0_in_body->inverse();
  out.place = lens_body_goal * out.lens_in_tcp.inverse();
  out.centre_axis_world = normalOf(*centre_face_pose).normalized();
  out.preplace = placeStandoff(out, approach_m_);

  // ---- 移出存储区 ----
  // 单段：旋转 + 平移同步。分段：先平移（保持姿态）再旋转 ——
  // 后者才是物理上可行的：镜片板绕 y 轴转动时远端会向上扫 ~0.26 m，
  // 在储存格上方就是接口盘，带姿态变化一起走必然顶住（实测 JTC 报
  // "State tolerances failed" → PATH_TOLERANCE_VIOLATED）。
  const double sgn = spec->base_position.x() >= 0.0 ? 1.0 : -1.0;
  out.exit_dx = sgn * storage_exit_dx_;
  out.exit_target = out.pregrasp;
  out.exit_target.linear() = shortestRotationToDown(out.pregrasp).linear();
  out.exit_target.translation().x() += out.exit_dx;
  out.exit_translate = out.pregrasp;  // 姿态不变
  out.exit_translate.translation().x() += out.exit_dx;
  out.exit_rotate = out.exit_translate;
  out.exit_rotate.linear() = shortestRotationToDown(out.pregrasp).linear();

  out.arm = arm;
  out.lens = lens;
  out.grasp_face = grasp_face;
  out.phase_deg = phase_deg;
  out.centre_face = centre_face;
  return true;
}

}  // namespace space_sim_task_control
