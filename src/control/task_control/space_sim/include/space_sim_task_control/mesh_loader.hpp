// 网格文件 → shape_msgs::Mesh
//
// 为什么需要：moveit_msgs/CollisionObject.meshes 要的是**网格数据**（顶点 + 三角面索引），
// 不是文件路径。所以 attached body / 已装配镜片都要先把 STL/OBJ 解析进消息里。
//
// 支持：二进制 STL（主流）、ASCII STL、OBJ（仅 v / f 行）。
// 顶点做去重（STL 每个三角形各存 3 个顶点，去重后规模可降到 1/3）。
#pragma once

#include <string>

#include "shape_msgs/msg/mesh.hpp"

namespace space_sim_task_control
{

/// 读取网格文件；失败时 err 说明原因
bool loadMeshFile(const std::string & path, shape_msgs::msg::Mesh & out, std::string & err);

/// 通过 ament_index 找 `包名/share/相对路径`，找不到返回空串
std::string resolvePackageFile(const std::string & package, const std::string & relative);

}  // namespace space_sim_task_control
