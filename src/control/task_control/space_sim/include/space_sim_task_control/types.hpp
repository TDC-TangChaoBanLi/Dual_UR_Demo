// space_sim 装配任务：公共常量、位姿类型、site 命名与双臂描述。
//
// 坐标系/接口约定（几何源头见 src/envs/space_sim/README.md）：
//   * 所有对接口 site 都是"插合帧"：+z = 面外法向、+x = 0.1 m 长边、原点 = 对接参考点；
//   * 两个插合的矩形接口相对姿态恒为 Q_DOCK = 绕 y 180° = (w,x,y,z) = (0,0,1,0)；
//   * dock 插件的 grasp 组额外接受绕 z 的 180° 相位，default 组不接受；
//   * MoveIt 的位姿目标一律用模型系 `world`（用 base_link 需要 TF，无 RSP 时会 -21）。
//
// 镜片 6 个侧面的角色（用户确认 + 位姿实测一致）：
//   * S0 —— 与储存插口对接的面（镜片竖直悬挂时朝上），储存期间被插口占着，不可抓；
//     装配时又用它去和中心镜片的对应对接口插合；
//   * S3 —— 与 S0 相对的另一面（朝下），机械臂够不到；
//   * S1/S2/S4/S5 —— 四个侧面；对每一个镜片，每条臂恰有 2 个可用（法向指向该臂的那两个）。
#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace space_sim_task_control
{

/// MoveIt 位姿目标使用的参考系（= URDF 根 link，也是 MuJoCo 世界系）
inline constexpr const char * kPlanningFrame = "world";

/// 每个对接口的插合相对姿态：绕 y 180°
inline const Eigen::Quaterniond kQDock{0.0, 0.0, 1.0, 0.0};

/// 储存镜片数量（len_ring_1..6）与中心镜片对接口数量（S0..S5）
inline constexpr int kLensCount = 6;
inline constexpr int kCentreFaceCount = 6;

/// 镜片的储存面（与插口对接）与正下方（够不到）的侧面编号
inline constexpr const char * kStorageFace = "S0";
inline constexpr const char * kDownFace = "S3";

/// 侧面对接口编号（抓取候选 = 这四个里筛出"法向指向本臂"的两个）
inline const std::array<const char *, 4> kSideFaces{{"S1", "S2", "S4", "S5"}};

// --------------------------------------------------------------------------- //
// 被搬运镜片的碰撞近似（多处共用，改一处即可）
// --------------------------------------------------------------------------- //
/// 镜片 body 系下的实测 AABB（米）
inline constexpr double kLensAabbX = 0.5438;
inline constexpr double kLensAabbY = 0.5888;
inline constexpr double kLensAabbZ = 0.05;
/// 镜片体心相对 body 原点的 z 偏移
inline constexpr double kLensBoxOffsetZ = -0.025;
/// 卫星底部接口盘 / 卫星在 x、y 上的半宽（镜片要"升到卫星上方"必须先完全离开这个范围）
inline constexpr double kSatelliteHalfXY = 0.30;

// --------------------------------------------------------------------------- //
// site / link 命名（唯一出处，避免各处拼字符串漂移）
// --------------------------------------------------------------------------- //

/// 镜片 body 名，如 len_ring_1
inline std::string lensBodyName(int lens)
{
  return "len_ring_" + std::to_string(lens);
}

/// 镜片某个侧面的对接口 site，如 len_ring_1_S5（face = "S0".."S5"）
inline std::string lensFaceSite(int lens, const std::string & face)
{
  return lensBodyName(lens) + "_" + face;
}

/// 该镜片的储存插口 site，如 len_dock_1_site
inline std::string lensSocketSite(int lens)
{
  return "len_dock_" + std::to_string(lens) + "_site";
}

/// 中心镜片的对接口 site，如 len_center_S2（face = "S0".."S5"）
inline std::string centreSite(const std::string & face)
{
  return "len_center_" + face;
}

// --------------------------------------------------------------------------- //
// 单臂描述
// --------------------------------------------------------------------------- //

struct ArmSpec
{
  std::string name;                 ///< "arm_A"
  std::string group;                ///< MoveIt 规划组名（与 name 相同）
  std::string base_link;            ///< "arm_A_ur_base_link"
  std::string tcp_link;             ///< "arm_A__tcp"
  std::string tcp_site;             ///< "arm_A__tcp_site"
  Eigen::Vector3d base_position;    ///< 基座世界位置（判定"外侧"方向与"朝向本臂"的侧面）
  std::vector<std::string> joints;
};

inline const std::array<ArmSpec, 2> & arms()
{
  static const std::array<ArmSpec, 2> kArms = {{
    ArmSpec{
      "arm_A", "arm_A", "arm_A_ur_base_link", "arm_A__tcp", "arm_A__tcp_site",
      Eigen::Vector3d(0.425, 0.0, 0.0),
      {"arm_A_ur_shoulder_pan_joint", "arm_A_ur_shoulder_lift_joint", "arm_A_ur_elbow_joint",
       "arm_A_ur_wrist_1_joint", "arm_A_ur_wrist_2_joint", "arm_A_ur_wrist_3_joint"}},
    ArmSpec{
      "arm_B", "arm_B", "arm_B_ur_base_link", "arm_B__tcp", "arm_B__tcp_site",
      Eigen::Vector3d(-0.425, 0.0, 0.0),
      {"arm_B_ur_shoulder_pan_joint", "arm_B_ur_shoulder_lift_joint", "arm_B_ur_elbow_joint",
       "arm_B_ur_wrist_1_joint", "arm_B_ur_wrist_2_joint", "arm_B_ur_wrist_3_joint"}},
  }};
  return kArms;
}

inline const ArmSpec * findArm(const std::string & name)
{
  for (const auto & a : arms()) {
    if (a.name == name) {
      return &a;
    }
  }
  return nullptr;
}

/// 所有 12 个可动关节（顺序：arm_A 6 个 + arm_B 6 个，与 SRDF / arm_BOTH 组一致）
inline std::vector<std::string> allJoints()
{
  std::vector<std::string> out;
  for (const auto & arm : arms()) {
    out.insert(out.end(), arm.joints.begin(), arm.joints.end());
  }
  return out;
}

// --------------------------------------------------------------------------- //
// 位姿工具（统一用 Eigen::Isometry3d）
// --------------------------------------------------------------------------- //

/// 沿局部 z 平移
inline Eigen::Isometry3d offsetZ(double d)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation().z() = d;
  return t;
}

/// 纯旋转
inline Eigen::Isometry3d rotation(const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  return t;
}

inline Eigen::Quaterniond axisAngleZ(double rad)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()));
}

/// 位姿的"面外法向" = 局部 +z 轴在世界系下的方向（site 帧约定：+z = 对接面外法向）
inline Eigen::Vector3d normalOf(const Eigen::Isometry3d & t)
{
  return t.linear().col(2);
}

inline Eigen::Isometry3d fromMsg(const geometry_msgs::msg::Pose & m)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() << m.position.x, m.position.y, m.position.z;
  t.linear() = Eigen::Quaterniond(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
                 .normalized()
                 .toRotationMatrix();
  return t;
}

inline geometry_msgs::msg::Pose toMsg(const Eigen::Isometry3d & t)
{
  geometry_msgs::msg::Pose m;
  m.position.x = t.translation().x();
  m.position.y = t.translation().y();
  m.position.z = t.translation().z();
  const Eigen::Quaterniond q(t.linear());
  m.orientation.w = q.w();
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  return m;
}

inline geometry_msgs::msg::PoseStamped toStamped(const Eigen::Isometry3d & t)
{
  geometry_msgs::msg::PoseStamped m;
  m.header.frame_id = kPlanningFrame;
  m.pose = toMsg(t);
  return m;
}

inline std::string fmt(const Eigen::Isometry3d & t)
{
  const auto p = t.translation();
  char buf[128];
  std::snprintf(buf, sizeof(buf), "(%.4f, %.4f, %.4f)", p.x(), p.y(), p.z());
  return std::string(buf);
}

/// 「把 TCP 的 +z 转到世界 -z」的最短旋转（绕 z 的滚转改动最小）。
///
/// 机械臂是**斜着抓**的（抓取面的世界法向抬高约 30°），所以需要转过的角度并不是 90°：
/// 旋转轴 = normalize(z_tcp × (-z_world))、角度 = 两者夹角。该旋转同时把镜片表面法向留在
/// ±y 附近 —— 镜片保持竖直、在 x 向跨约 0.54 m，配合沿 ±x 平移 0.6 m 即可整体离开接口盘
/// （|x| < 0.3）与太阳能板（|x| < 0.24）的范围。
inline Eigen::Isometry3d shortestRotationToDown(const Eigen::Isometry3d & from)
{
  const Eigen::Vector3d z_from = normalOf(from);
  const Eigen::Vector3d z_to(0.0, 0.0, -1.0);
  Eigen::Isometry3d out = from;
  const Eigen::Vector3d axis = z_from.cross(z_to);
  const double s = axis.norm();
  const double c = z_from.dot(z_to);
  if (s < 1e-9) {
    if (c > 0.0) {
      return out;  // 已经朝下
    }
    Eigen::Vector3d any = (std::abs(z_from.x()) < 0.9) ? Eigen::Vector3d::UnitX()
                                                       : Eigen::Vector3d::UnitY();
    Eigen::Vector3d perp = any - z_from * any.dot(z_from);
    if (perp.norm() < 1e-9) {
      perp = Eigen::Vector3d::UnitZ();
    }
    out.linear() = Eigen::AngleAxisd(M_PI, perp.normalized()).toRotationMatrix() * from.linear();
    return out;
  }
  const double angle = std::atan2(s, c);
  out.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix() * from.linear();
  return out;
}

}  // namespace space_sim_task_control
