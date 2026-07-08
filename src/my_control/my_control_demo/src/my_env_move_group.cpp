#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
const char* ARM_A_GROUP = "arm_A";
const char* ARM_B_GROUP = "arm_B";
const char* ARM_BOTH_GROUP = "arm_BOTH";
const char* ARM_A_TCP = "arm_A__tcp";
const char* ARM_B_TCP = "arm_B__tcp";

std::string format_pose(const geometry_msgs::msg::Pose& pose)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4)
         << "pos=[" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "] "
         << "quat=[" << pose.orientation.x << ", " << pose.orientation.y << ", "
         << pose.orientation.z << ", " << pose.orientation.w << "]";
  return stream.str();
}

geometry_msgs::msg::Pose offset_pose(
  const geometry_msgs::msg::Pose& source, double dx, double dy, double dz, double droll, double dpitch, double dyaw)
{
  geometry_msgs::msg::Pose target = source;
  target.position.x += dx;
  target.position.y += dy;
  target.position.z += dz;

  tf2::Quaternion current;
  tf2::fromMsg(source.orientation, current);
  tf2::Quaternion delta;
  delta.setRPY(droll, dpitch, dyaw);
  tf2::Quaternion result = current * delta;
  result.normalize();
  target.orientation = tf2::toMsg(result);
  return target;
}

std::optional<rclcpp::ParameterValue> to_rclcpp_parameter_value(const rcl_interfaces::msg::ParameterValue& value)
{
  switch (value.type)
  {
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

bool copy_move_group_parameters(const rclcpp::Node::SharedPtr& node)
{
  if (node->has_parameter("robot_description") && node->has_parameter("robot_description_semantic"))
  {
    RCLCPP_INFO(node->get_logger(), "MoveIt robot description parameters are already set on this node.");
    return true;
  }

  const std::string source_node = node->declare_parameter<std::string>("move_group_parameter_node", "/arm_BOTH_move_group");
  const std::vector<std::string> names{
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics",
    "robot_description_planning",
    "planning_pipelines",
    "default_planning_pipeline",
    "ompl",
    "pilz_industrial_motion_planner",
    "moveit_controller_manager",
    "moveit_simple_controller_manager",
    "trajectory_execution.allowed_execution_duration_scaling",
    "trajectory_execution.allowed_goal_duration_margin",
    "trajectory_execution.allowed_start_tolerance",
    "trajectory_execution.execution_duration_monitoring",
    "planning_scene_monitor_options",
  };

  const std::string service_name = source_node + "/get_parameters";
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(service_name);

  RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt parameters from %s", service_name.c_str());
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter service is not available: %s", service_name.c_str());
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;
  auto future = client->async_send_request(request);
  if (future.wait_for(10s) != std::future_status::ready)
  {
    RCLCPP_ERROR(node->get_logger(), "Timed out while reading parameters from %s", source_node.c_str());
    return false;
  }

  const auto response = future.get();
  if (response->values.size() != names.size())
  {
    RCLCPP_ERROR(node->get_logger(), "Unexpected parameter response size from %s", source_node.c_str());
    return false;
  }

  bool found_robot_description = false;
  bool found_robot_description_semantic = false;
  std::vector<rclcpp::Parameter> copied_parameters;

  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const auto converted = to_rclcpp_parameter_value(response->values[i]);
    if (!converted)
    {
      RCLCPP_DEBUG(node->get_logger(), "Parameter %s is not set on %s", names[i].c_str(), source_node.c_str());
      continue;
    }

    if (names[i] == "robot_description")
    {
      found_robot_description = true;
    }
    if (names[i] == "robot_description_semantic")
    {
      found_robot_description_semantic = true;
    }
    copied_parameters.emplace_back(names[i], *converted);
  }

  if (!found_robot_description || !found_robot_description_semantic)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Could not read robot_description and robot_description_semantic from %s. "
                 "Make sure start_my_env_moveit.launch.py is running.",
                 source_node.c_str());
    return false;
  }

  for (const auto& parameter : copied_parameters)
  {
    if (node->has_parameter(parameter.get_name()))
    {
      node->set_parameter(parameter);
    }
    else
    {
      node->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
    }
  }

  RCLCPP_INFO(node->get_logger(), "Copied %zu MoveIt parameter(s) from %s", copied_parameters.size(), source_node.c_str());
  return true;
}

class MoveGroupTester
{
public:
  explicit MoveGroupTester(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    arm_a_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, ARM_A_GROUP);
    arm_b_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, ARM_B_GROUP);
    arm_both_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, ARM_BOTH_GROUP);

    configure_group(arm_a_group_);
    configure_group(arm_b_group_);
    configure_group(arm_both_group_);

    RCLCPP_INFO(node_->get_logger(), "my_env_move_group initialized");
  }

  void run()
  {
    RCLCPP_INFO(node_->get_logger(), "=== Starting MoveGroup planning/execution tests ===");
    bool arm_a_ok = test_single_arm(arm_a_group_, ARM_A_GROUP, ARM_A_TCP);
    bool arm_b_ok = test_single_arm(arm_b_group_, ARM_B_GROUP, ARM_B_TCP);
    bool arm_both_ok = test_both_arm();

    RCLCPP_INFO(node_->get_logger(), "=== MoveGroup test summary: arm_A=%s arm_B=%s arm_BOTH=%s ===",
                arm_a_ok ? "OK" : "FAILED", arm_b_ok ? "OK" : "FAILED", arm_both_ok ? "OK" : "FAILED");
  }

private:
  void configure_group(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group)
  {
    group->setPlanningTime(10.0);
    group->setNumPlanningAttempts(20);
    group->setGoalPositionTolerance(0.005);
    group->setGoalOrientationTolerance(0.01);
    group->setGoalJointTolerance(0.005);
    group->allowReplanning(true);
  }

  bool plan_and_execute(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group, const std::string& label)
  {
    group->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = group->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s planning failed", label.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s planning succeeded; executing", label.c_str());
    const auto execute_result = group->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s execution failed", label.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s execution succeeded", label.c_str());
    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool move_single_link_to_pose(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group,
    const std::string& group_name, const std::string& link, const geometry_msgs::msg::Pose& pose,
    const std::string& target_name)
  {
    group->clearPoseTargets();
    if (!group->setPoseTarget(pose, link))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to set pose target for %s", group_name.c_str(), link.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s %s target for %s: %s", group_name.c_str(), target_name.c_str(),
                link.c_str(), format_pose(pose).c_str());
    return plan_and_execute(group, group_name + " " + target_name);
  }

  bool test_single_arm(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group,
    const std::string& group_name, const std::string& tcp_link)
  {
    RCLCPP_INFO(node_->get_logger(), "=== Testing planning group %s ===", group_name.c_str());
    const auto initial = group->getCurrentPose(tcp_link);
    RCLCPP_INFO(node_->get_logger(), "%s current pose for %s: %s", group_name.c_str(), tcp_link.c_str(),
                format_pose(initial.pose).c_str());

    const std::vector<geometry_msgs::msg::Pose> targets{
      offset_pose(initial.pose, 0.02, 0.0, 0.0, 0.05, 0.0, 0.0),
      offset_pose(initial.pose, 0.0, 0.02, 0.0, 0.0, 0.05, 0.0),
      offset_pose(initial.pose, 0.0, 0.0, 0.02, 0.0, 0.0, 0.05),
    };

    bool ok = true;
    for (std::size_t i = 0; i < targets.size(); ++i)
    {
      ok = move_single_link_to_pose(group, group_name, tcp_link, targets[i], "target_" + std::to_string(i + 1)) && ok;
    }

    RCLCPP_INFO(node_->get_logger(), "%s returning to initial pose: %s", group_name.c_str(),
                format_pose(initial.pose).c_str());
    ok = move_single_link_to_pose(group, group_name, tcp_link, initial.pose, "return_initial") && ok;
    return ok;
  }

  bool move_both_to_poses(
    const geometry_msgs::msg::Pose& arm_a_pose, const geometry_msgs::msg::Pose& arm_b_pose,
    const std::string& target_name)
  {
    arm_both_group_->clearPoseTargets();
    const bool a_set = arm_both_group_->setPoseTarget(arm_a_pose, ARM_A_TCP);
    const bool b_set = arm_both_group_->setPoseTarget(arm_b_pose, ARM_B_TCP);
    if (!a_set || !b_set)
    {
      RCLCPP_ERROR(node_->get_logger(), "arm_BOTH failed to set pose targets for %s", target_name.c_str());
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "arm_BOTH %s target %s: %s", target_name.c_str(), ARM_A_TCP,
                format_pose(arm_a_pose).c_str());
    RCLCPP_INFO(node_->get_logger(), "arm_BOTH %s target %s: %s", target_name.c_str(), ARM_B_TCP,
                format_pose(arm_b_pose).c_str());
    return plan_and_execute(arm_both_group_, "arm_BOTH " + target_name);
  }

  bool test_both_arm()
  {
    RCLCPP_INFO(node_->get_logger(), "=== Testing planning group arm_BOTH ===");
    const auto initial_a = arm_both_group_->getCurrentPose(ARM_A_TCP);
    const auto initial_b = arm_both_group_->getCurrentPose(ARM_B_TCP);
    RCLCPP_INFO(node_->get_logger(), "arm_BOTH current pose %s: %s", ARM_A_TCP, format_pose(initial_a.pose).c_str());
    RCLCPP_INFO(node_->get_logger(), "arm_BOTH current pose %s: %s", ARM_B_TCP, format_pose(initial_b.pose).c_str());

    struct BothTarget
    {
      geometry_msgs::msg::Pose a;
      geometry_msgs::msg::Pose b;
    };

    const std::vector<BothTarget> targets{
      { offset_pose(initial_a.pose, 0.02, 0.0, 0.0, 0.05, 0.0, 0.0),
        offset_pose(initial_b.pose, -0.02, 0.0, 0.0, -0.05, 0.0, 0.0) },
      { offset_pose(initial_a.pose, 0.0, 0.02, 0.0, 0.0, 0.05, 0.0),
        offset_pose(initial_b.pose, 0.0, -0.02, 0.0, 0.0, -0.05, 0.0) },
      { offset_pose(initial_a.pose, 0.0, 0.0, 0.02, 0.0, 0.0, 0.05),
        offset_pose(initial_b.pose, 0.0, 0.0, 0.02, 0.0, 0.0, -0.05) },
    };

    bool ok = true;
    for (std::size_t i = 0; i < targets.size(); ++i)
    {
      ok = move_both_to_poses(targets[i].a, targets[i].b, "target_" + std::to_string(i + 1)) && ok;
    }

    RCLCPP_INFO(node_->get_logger(), "arm_BOTH returning to initial poses");
    ok = move_both_to_poses(initial_a.pose, initial_b.pose, "return_initial") && ok;
    return ok;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_a_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_b_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_both_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("my_env_move_group", node_options);

  std::thread worker([node]() {
    std::this_thread::sleep_for(2s);
    if (!copy_move_group_parameters(node))
    {
      rclcpp::shutdown();
      return;
    }
    MoveGroupTester tester(node);
    tester.run();
    rclcpp::shutdown();
  });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  if (worker.joinable())
  {
    worker.join();
  }
  return 0;
}
