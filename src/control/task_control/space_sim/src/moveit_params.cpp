#include "space_sim_task_control/moveit_params.hpp"

#include <atomic>
#include <chrono>
#include <optional>
#include <vector>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/executors.hpp"
#include "space_sim_task_control/types.hpp"

namespace space_sim_task_control
{
namespace
{

std::optional<rclcpp::ParameterValue> toParameterValue(
  const rcl_interfaces::msg::ParameterValue & value)
{
  switch (value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      return rclcpp::ParameterValue(value.bool_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return rclcpp::ParameterValue(value.integer_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return rclcpp::ParameterValue(value.double_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
      return rclcpp::ParameterValue(value.string_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
      return rclcpp::ParameterValue(value.byte_array_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
      return rclcpp::ParameterValue(value.bool_array_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
      return rclcpp::ParameterValue(value.integer_array_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return rclcpp::ParameterValue(value.double_array_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
      return rclcpp::ParameterValue(value.string_array_value);
    default:
      return std::nullopt;
  }
}

}  // namespace

bool copyMoveGroupParameters(
  const rclcpp::Node::SharedPtr & node, std::string source_node, double timeout_s)
{
  const auto timeout = std::chrono::duration<double>(timeout_s);
  if (node->has_parameter("robot_description") &&
    node->has_parameter("robot_description_semantic"))
  {
    return true;  // 已经由 launch 传入
  }
  if (source_node.empty()) {
    source_node = node->declare_parameter<std::string>("move_group_parameter_node", "/arm_BOTH_move_group");
  }

  std::vector<std::string> names{
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics",
    "robot_description_planning",
    "planning_pipelines",
    "default_planning_pipeline",
    "chomp",
    "stomp",
    "pilz_industrial_motion_planner",
    "moveit_controller_manager",
    "moveit_simple_controller_manager",
    "trajectory_execution.allowed_execution_duration_scaling",
    "trajectory_execution.allowed_goal_duration_margin",
    "trajectory_execution.allowed_start_tolerance",
    "trajectory_execution.execution_duration_monitoring",
    "planning_scene_monitor_options",
  };

  // MoveIt 的参数服务按完整叶子名称公开关节限制；读取父级
  // robot_description_planning 不会把这些数值带到客户端 RobotModel。
  for (const auto & joint : allJoints()) {
    const std::string prefix = "robot_description_planning.joint_limits." + joint + ".";
    for (const char * key : {"has_velocity_limits", "max_velocity",
                             "has_acceleration_limits", "max_acceleration"})
      names.push_back(prefix + key);
  }

  const std::string service_name = source_node + "/get_parameters";
  // 用一个临时节点发起这次服务调用：目标节点可能已经被别的 executor 占用，
  // 而 spin_until_future_complete 需要一个"没有归属"的节点。
  static std::atomic<int> counter{0};
  auto fetcher = rclcpp::Node::make_shared(
    "space_sim_param_fetcher_" + std::to_string(counter.fetch_add(1)));
  auto client = fetcher->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  if (!client->wait_for_service(timeout)) {
    RCLCPP_ERROR(
      node->get_logger(), "参数服务不可用: %s（move_group 没起？）", service_name.c_str());
    return false;
  }
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
        fetcher, future,
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "读取 %s 的参数超时", source_node.c_str());
    return false;
  }
  const auto response = future.get();
  if (response->values.size() != names.size()) {
    RCLCPP_ERROR(node->get_logger(), "从 %s 读到的参数个数异常", source_node.c_str());
    return false;
  }

  bool has_description = false;
  bool has_semantic = false;
  std::vector<rclcpp::Parameter> copied;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto converted = toParameterValue(response->values[i]);
    if (!converted) {
      continue;
    }
    if (names[i] == "robot_description") {
      has_description = true;
    }
    if (names[i] == "robot_description_semantic") {
      has_semantic = true;
    }
    copied.emplace_back(names[i], *converted);
  }
  if (!has_description || !has_semantic) {
    RCLCPP_ERROR(
      node->get_logger(), "%s 上没有 robot_description / robot_description_semantic", source_node.c_str());
    return false;
  }
  for (const auto & parameter : copied) {
    if (node->has_parameter(parameter.get_name())) {
      node->set_parameter(parameter);
    } else {
      node->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "已从 %s 拷贝 %zu 个 MoveIt 参数", source_node.c_str(), copied.size());
  return true;
}

}  // namespace space_sim_task_control
