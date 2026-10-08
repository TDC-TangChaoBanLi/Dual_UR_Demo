#pragma once

#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <Eigen/Geometry>

namespace space_sim_task_control
{
inline std::string poseDescription(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond q(pose.rotation());
  std::ostringstream out;
  out << std::setprecision(17) << "world xyz(m)=[" << pose.translation().x() << ","
      << pose.translation().y() << "," << pose.translation().z() << "] quat(xyzw)=["
      << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "]";
  return out.str();
}

inline std::string jointDescription(const std::vector<std::string> & names,
                                    const std::vector<double> & positions)
{
  std::ostringstream out;
  out << std::setprecision(17) << "关节(rad)={";
  for (size_t i = 0; i < positions.size(); ++i) {
    if (i) out << ",";
    out << (i < names.size() ? names[i] : "未命名") << ":" << positions[i];
  }
  out << "}";
  return out.str();
}

inline std::string moveItFailure(int code)
{
  const char * reason = "未分类 MoveIt 错误，需结合 move_group 日志";
  switch (code) {
    case -1: reason = "规划失败，未找到有效轨迹"; break;
    case -2: reason = "规划结果无效"; break;
    case -3: reason = "环境变化导致轨迹失效"; break;
    case -4: reason = "控制器执行失败"; break;
    case -6: reason = "规划或求解超时"; break;
    case -7: reason = "请求被抢占"; break;
    case -10: reason = "起始状态碰撞"; break;
    case -11: reason = "起始状态违反路径约束"; break;
    case -12: reason = "目标状态碰撞"; break;
    case -13: reason = "目标状态违反路径约束"; break;
    case -14: reason = "目标约束无效"; break;
    case -15: reason = "规划组无效"; break;
    case -17: reason = "机器人状态无效"; break;
    case -21: reason = "坐标变换失败"; break;
    case -31: reason = "未找到 IK 解（不可达、奇异、超时或碰撞过滤，单凭此码不能区分）"; break;
  }
  return std::string(reason) + " code=" + std::to_string(code);
}
}  // namespace space_sim_task_control
