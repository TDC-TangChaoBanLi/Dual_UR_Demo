#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

struct ArmConfig
{
  std::string id;
  std::string controller_prefix;
  std::vector<std::string> joints;
};

builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  const double clamped = std::max(0.0, seconds);
  duration.sec = static_cast<int32_t>(std::floor(clamped));
  duration.nanosec = static_cast<uint32_t>((clamped - static_cast<double>(duration.sec)) * 1e9);
  return duration;
}

bool starts_with(const std::string& value, const std::string& prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool ends_with(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
std::string format_positions(const std::vector<double>& values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << "[";
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i != 0)
    {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}
}  // namespace

class Ros2ControlTester : public rclcpp::Node
{
public:
  Ros2ControlTester() : Node("test_ros2_control")
  {
    controller_type_ = this->declare_parameter<std::string>("controller_type", "scaled_joint_trajectory_controller");
    arm_target_ = this->declare_parameter<std::string>("arm_target", "both");
    arm_a_prefix_ = this->declare_parameter<std::string>("arm_a_prefix", "arm_A_ur");
    arm_b_prefix_ = this->declare_parameter<std::string>("arm_b_prefix", "arm_B_ur");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    command_mode_ = this->declare_parameter<std::string>("command_mode", "action");
    trajectory_time_sec_ = this->declare_parameter<double>("trajectory_time_sec", 2.0);
    small_delta_rad_ = this->declare_parameter<double>("small_delta_rad", 0.03);
    medium_delta_rad_ = this->declare_parameter<double>("medium_delta_rad", 0.08);
    large_delta_rad_ = this->declare_parameter<double>("large_delta_rad", 0.15);
    goal_position_tolerance_rad_ = this->declare_parameter<double>("goal_position_tolerance_rad", 0.01);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Ros2ControlTester::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "test_ros2_control initialized");
    RCLCPP_INFO(this->get_logger(), "controller_type=%s arm_target=%s command_mode=%s joint_state_topic=%s",
                controller_type_.c_str(), arm_target_.c_str(), command_mode_.c_str(), joint_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "arm_a_prefix=%s arm_b_prefix=%s",
                arm_a_prefix_.c_str(), arm_b_prefix_.c_str());
  }

  void run()
  {
    const auto arms = selected_arms();
    if (arms.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid arm_target '%s'. Use A, B, or both.", arm_target_.c_str());
      return;
    }

    if (!valid_controller_type())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid controller_type '%s'. Supported suffixes: scaled_joint_trajectory_controller, "
                   "joint_trajectory_controller, passthrough_trajectory_controller, forward_position_controller.",
                   controller_type_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for current joint states...");
    if (!wait_for_joint_states(arms, 10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for required joint states on %s", joint_state_topic_.c_str());
      return;
    }

    bool all_ok = true;
    for (const auto& arm : arms)
    {
      const std::string controller = resolve_controller_name(arm);
      const bool forward_position = ends_with(controller, "forward_position_controller");

      RCLCPP_INFO(this->get_logger(), "=== Testing arm %s with controller %s ===", arm.id.c_str(), controller.c_str());

      std::vector<double> current;
      if (!get_current_positions(arm, current))
      {
        RCLCPP_ERROR(this->get_logger(), "Missing current joint positions for arm %s", arm.id.c_str());
        all_ok = false;
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Current positions for arm %s: %s", arm.id.c_str(), format_positions(current).c_str());
      auto waypoints = generate_waypoints(current);
      for (std::size_t i = 0; i < waypoints.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "Waypoint %zu for arm %s: %s", i, arm.id.c_str(),
                    format_positions(waypoints[i]).c_str());
      }

      create_controller_state_subscription(controller);

      bool ok = false;
      if (forward_position)
      {
        ok = run_forward_position_test(controller, waypoints);
      }
      else if (command_mode_ == "topic")
      {
        ok = run_trajectory_topic_test(controller, arm, waypoints);
      }
      else
      {
        ok = run_trajectory_action_test(controller, arm, waypoints);
      }

      std::this_thread::sleep_for(500ms);
      print_final_delta(arm, waypoints.back());
      all_ok = all_ok && ok;
    }

    if (all_ok)
    {
      RCLCPP_INFO(this->get_logger(), "All requested ros2_control controller tests finished successfully.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "One or more ros2_control controller tests reported errors.");
    }
  }

private:
  std::vector<ArmConfig> selected_arms() const
  {
    const std::vector<std::string> joint_suffixes{
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    std::vector<std::string> arm_a_joints;
    std::vector<std::string> arm_b_joints;
    for (const auto& suffix : joint_suffixes)
    {
      arm_a_joints.push_back(arm_a_prefix_ + "_" + suffix);
      arm_b_joints.push_back(arm_b_prefix_ + "_" + suffix);
    }

    const ArmConfig arm_a{ "A", arm_a_prefix_, arm_a_joints };
    const ArmConfig arm_b{ "B", arm_b_prefix_, arm_b_joints };

    if (arm_target_ == "A" || arm_target_ == "a")
    {
      return { arm_a };
    }
    if (arm_target_ == "B" || arm_target_ == "b")
    {
      return { arm_b };
    }
    if (arm_target_ == "both" || arm_target_ == "Both" || arm_target_ == "BOTH")
    {
      return { arm_a, arm_b };
    }
    return {};
  }

  bool valid_controller_type() const
  {
    static const std::set<std::string> supported{
      "scaled_joint_trajectory_controller",
      "joint_trajectory_controller",
      "passthrough_trajectory_controller",
      "forward_position_controller",
    };

    if (supported.count(controller_type_) != 0)
    {
      return true;
    }

    return std::any_of(supported.begin(), supported.end(), [this](const std::string& suffix) {
      return ends_with(controller_type_, suffix);
    });
  }

  std::string resolve_controller_name(const ArmConfig& arm) const
  {
    if (starts_with(controller_type_, "arm_A_") || starts_with(controller_type_, "arm_B_"))
    {
      if (!starts_with(controller_type_, arm.controller_prefix))
      {
        RCLCPP_WARN(this->get_logger(),
                    "Full controller name '%s' does not match selected arm %s. Using it exactly as requested.",
                    controller_type_.c_str(), arm.id.c_str());
      }
      return controller_type_;
    }

    std::string controller = controller_type_;
    while (starts_with(controller, "/"))
    {
      controller.erase(controller.begin());
    }
    return arm.controller_prefix + "_" + controller;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
      latest_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  bool wait_for_joint_states(const std::vector<ArmConfig>& arms, std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      bool all_present = true;
      for (const auto& arm : arms)
      {
        std::vector<double> unused;
        all_present = all_present && get_current_positions(arm, unused);
      }
      if (all_present)
      {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool get_current_positions(const ArmConfig& arm, std::vector<double>& positions) const
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    positions.clear();
    for (const auto& joint : arm.joints)
    {
      const auto it = latest_joint_positions_.find(joint);
      if (it == latest_joint_positions_.end())
      {
        return false;
      }
      positions.push_back(it->second);
    }
    return true;
  }

  std::vector<std::vector<double>> generate_waypoints(const std::vector<double>& current) const
  {
    const std::vector<double> delta{
      small_delta_rad_, small_delta_rad_, medium_delta_rad_, large_delta_rad_, large_delta_rad_, large_delta_rad_
    };

    auto plus = current;
    auto minus = current;
    for (std::size_t i = 0; i < current.size() && i < delta.size(); ++i)
    {
      plus[i] += delta[i];
      minus[i] -= delta[i];
    }
    return { current, plus, minus, current };
  }

  trajectory_msgs::msg::JointTrajectory build_trajectory(
    const ArmConfig& arm, const std::vector<std::vector<double>>& waypoints) const
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.joint_names = arm.joints;

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = waypoints[i];
      point.time_from_start = duration_from_seconds((static_cast<double>(i) + 1.0) * trajectory_time_sec_);
      trajectory.points.push_back(point);
    }

    return trajectory;
  }

  void create_controller_state_subscription(const std::string& controller)
  {
    const std::string topic = "/" + controller + "/controller_state";
    auto sub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      topic, 10,
      [this, controller](const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
        if (!msg->feedback.positions.empty() && !msg->reference.positions.empty())
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "State %s feedback[0]=%.4f reference[0]=%.4f speed_scaling=%.3f",
                               controller.c_str(), msg->feedback.positions.front(), msg->reference.positions.front(),
                               msg->speed_scaling_factor);
        }
      });
    controller_state_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed controller state: %s", topic.c_str());
  }

  bool run_trajectory_action_test(
    const std::string& controller, const ArmConfig& arm, const std::vector<std::vector<double>>& waypoints)
  {
    const std::string action_name = "/" + controller + "/follow_joint_trajectory";
    auto client = rclcpp_action::create_client<FollowJointTrajectory>(this->shared_from_this(), action_name);

    RCLCPP_INFO(this->get_logger(), "Waiting for action server: %s", action_name.c_str());
    if (!client->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server is not available: %s", action_name.c_str());
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = build_trajectory(arm, waypoints);
    goal.goal_time_tolerance = duration_from_seconds(2.0);

    // Per-joint goal tolerance so the controller actually verifies that the
    // arm reached the target.  Without this, JTC only checks that the
    // trajectory time elapsed and reports success even if the arm never moved.
    for (const auto& joint : arm.joints)
    {
      control_msgs::msg::JointTolerance tol;
      tol.name = joint;
      tol.position = goal_position_tolerance_rad_;
      goal.goal_tolerance.push_back(tol);
    }

    auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    options.goal_response_callback = [this, controller](GoalHandleFollowJointTrajectory::SharedPtr goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by %s", controller.c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by %s", controller.c_str());
      }
    };
    options.feedback_callback = [this, controller](
                                GoalHandleFollowJointTrajectory::SharedPtr,
                                const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
      if (!feedback->actual.positions.empty() && !feedback->desired.positions.empty())
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Feedback %s actual[0]=%.4f desired[0]=%.4f error_points=%zu",
                             controller.c_str(), feedback->actual.positions.front(), feedback->desired.positions.front(),
                             feedback->error.positions.size());
      }
    };

    RCLCPP_INFO(this->get_logger(), "Sending trajectory action goal to %s", action_name.c_str());
    auto goal_handle_future = client->async_send_goal(goal, options);
    if (goal_handle_future.wait_for(10s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for goal response from %s", controller.c_str());
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      return false;
    }

    const auto expected = std::chrono::duration<double>(trajectory_time_sec_ * static_cast<double>(waypoints.size()) + 10.0);
    auto result_future = client->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(expected)) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for result from %s", controller.c_str());
      return false;
    }

    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR(this->get_logger(), "Action %s finished with result code %d", controller.c_str(),
                   static_cast<int>(wrapped.code));
      return false;
    }

    const auto result = wrapped.result;
    if (result->error_code == FollowJointTrajectory::Result::SUCCESSFUL)
    {
      RCLCPP_INFO(this->get_logger(), "Action %s succeeded: %s", controller.c_str(), result->error_string.c_str());
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Action %s failed with error_code=%d error='%s'", controller.c_str(),
                 result->error_code, result->error_string.c_str());
    return false;
  }

  bool run_trajectory_topic_test(
    const std::string& controller, const ArmConfig& arm, const std::vector<std::vector<double>>& waypoints)
  {
    const std::string topic = "/" + controller + "/joint_trajectory";
    auto publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic, 10);
    wait_for_subscribers(topic, publisher);

    const auto trajectory = build_trajectory(arm, waypoints);
    RCLCPP_INFO(this->get_logger(), "Publishing JointTrajectory to %s", topic.c_str());
    publisher->publish(trajectory);

    const auto wait_time = std::chrono::duration<double>(trajectory_time_sec_ * static_cast<double>(waypoints.size()) + 1.0);
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time));
    return true;
  }

  bool run_forward_position_test(const std::string& controller, const std::vector<std::vector<double>>& waypoints)
  {
    const std::string topic = "/" + controller + "/commands";
    auto publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
    wait_for_subscribers(topic, publisher);

    RCLCPP_INFO(this->get_logger(), "Publishing forward position commands to %s", topic.c_str());
    for (std::size_t i = 0; i < waypoints.size() && rclcpp::ok(); ++i)
    {
      std_msgs::msg::Float64MultiArray command;
      command.data = waypoints[i];
      publisher->publish(command);
      RCLCPP_INFO(this->get_logger(), "Published forward position waypoint %zu: %s", i,
                  format_positions(waypoints[i]).c_str());
      std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(trajectory_time_sec_)));
    }
    return true;
  }

  template <typename PublisherT>
  void wait_for_subscribers(const std::string& topic, const PublisherT& publisher)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      if (publisher->get_subscription_count() > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Topic %s has %zu subscriber(s)", topic.c_str(), publisher->get_subscription_count());
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_WARN(this->get_logger(), "No subscribers detected on %s before publishing", topic.c_str());
  }

  void print_final_delta(const ArmConfig& arm, const std::vector<double>& target) const
  {
    std::vector<double> actual;
    if (!get_current_positions(arm, actual))
    {
      RCLCPP_WARN(this->get_logger(), "Cannot print final delta for arm %s; joint state is missing", arm.id.c_str());
      return;
    }

    std::vector<double> delta;
    for (std::size_t i = 0; i < actual.size() && i < target.size(); ++i)
    {
      delta.push_back(actual[i] - target[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Final positions for arm %s: %s", arm.id.c_str(), format_positions(actual).c_str());
    RCLCPP_INFO(this->get_logger(), "Final delta actual-target for arm %s: %s", arm.id.c_str(),
                format_positions(delta).c_str());
  }

  std::string controller_type_;
  std::string arm_target_;
  std::string arm_a_prefix_;
  std::string arm_b_prefix_;
  std::string joint_state_topic_;
  std::string command_mode_;
  double trajectory_time_sec_ = 2.0;
  double small_delta_rad_ = 0.03;
  double medium_delta_rad_ = 0.08;
  double large_delta_rad_ = 0.15;
  double goal_position_tolerance_rad_ = 0.01;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr> controller_state_subs_;
  mutable std::mutex joint_state_mutex_;
  std::map<std::string, double> latest_joint_positions_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ros2ControlTester>();

  std::thread worker([node]() {
    std::this_thread::sleep_for(500ms);
    node->run();
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
