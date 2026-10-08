// 把 MoveIt 参数从运行中的 move_group 节点拷贝到本节点。
//
// 为什么需要：`MoveGroupInterface` 会在构造时从**本节点的参数**里读 robot_description /
// robot_description_semantic 来建立 robot model。任务节点自己是新进程，参数服务器上没有
// 这些内容，所以要主动拷贝一份（与 src/control/test_control/src/test_move_group.cpp 同一做法）。
// 好处是不必在多个 launch 文件里重复加载同一份 URDF/SRDF，且与正在运行的 move_group 完全一致。
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace space_sim_task_control
{

/// 成功时返回 true；`source_node` 为空用参数 `move_group_parameter_node`
/// （默认 `/arm_BOTH_move_group`）
bool copyMoveGroupParameters(
  const rclcpp::Node::SharedPtr & node, std::string source_node = "", double timeout_s = 10.0);

}  // namespace space_sim_task_control
