#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
struct ServoArm
{
  std::string id;
  std::string pose_topic;
  std::string twist_topic;
  std::string joint_topic;
  std::string status_topic;
  std::string tcp_frame;
  std::vector<std::string> joint_names;
};

std::string status_to_string(const moveit_msgs::msg::ServoStatus& status)
{
  std::ostringstream stream;
  stream << "code=" << static_cast<int>(status.code) << " message='" << status.message << "'";
  return stream.str();
}

geometry_msgs::msg::Pose pose_from_transform(const geometry_msgs::msg::TransformStamped& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation = transform.transform.rotation;
  return pose;
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
}  // namespace

class MoveServoTester : public rclcpp::Node
{
public:
  MoveServoTester() : Node("my_env_move_servo"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    servo_target_ = this->declare_parameter<std::string>("servo_target", "both");
    command_type_ = this->declare_parameter<std::string>("command_type", "pose");
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "world");
    duration_sec_ = this->declare_parameter<double>("duration_sec", 30.0);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    linear_speed_ = this->declare_parameter<double>("linear_speed", 0.01);
    angular_speed_ = this->declare_parameter<double>("angular_speed", 0.05);
    position_delta_m_ = this->declare_parameter<double>("position_delta_m", 0.02);
    orientation_delta_rad_ = this->declare_parameter<double>("orientation_delta_rad", 0.05);
    joint_delta_rad_ = this->declare_parameter<double>("joint_delta_rad", 0.03);
    joint_velocity_rad_s_ = this->declare_parameter<double>("joint_velocity_rad_s", 0.05);

    RCLCPP_INFO(this->get_logger(), "my_env_move_servo initialized");
    RCLCPP_INFO(
      this->get_logger(),
      "servo_target=%s command_type=%s planning_frame=%s duration_sec=%.2f publish_rate_hz=%.2f",
      servo_target_.c_str(), command_type_.c_str(), planning_frame_.c_str(), duration_sec_, publish_rate_hz_);
    RCLCPP_INFO(
      this->get_logger(),
      "twist speeds: linear=%.4f m/s angular=%.4f rad/s; pose deltas: position=%.4f m orientation=%.4f rad; "
      "joint delta=%.4f rad joint velocity=%.4f rad/s",
      linear_speed_, angular_speed_, position_delta_m_, orientation_delta_rad_, joint_delta_rad_, joint_velocity_rad_s_);
  }

  void run()
  {
    if (!valid_command_type())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid command_type '%s'. Use pose, twist, or joint.", command_type_.c_str());
      return;
    }

    const auto arms = selected_arms();
    if (arms.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid servo_target '%s'. Use A, B, or both.", servo_target_.c_str());
      return;
    }

    create_publishers_and_subscribers(arms);
    wait_for_servo_subscribers(arms);

    if (command_type_ == "pose" && !capture_start_poses(arms))
    {
      return;
    }

    const double safe_duration = std::max(0.1, duration_sec_);
    const double phase_duration = safe_duration / 6.0;
    const auto start = std::chrono::steady_clock::now();
    auto next_log = start;
    rclcpp::Rate rate(std::max(1.0, publish_rate_hz_));

    RCLCPP_INFO(this->get_logger(), "Starting %s Servo command test for %.2f seconds", command_type_.c_str(), safe_duration);
    while (rclcpp::ok())
    {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      if (elapsed >= safe_duration)
      {
        break;
      }

      const int phase = std::min(5, static_cast<int>(elapsed / phase_duration));
      const double phase_elapsed = std::fmod(elapsed, phase_duration);
      const double sign = phase_elapsed < (phase_duration / 2.0) ? 1.0 : -1.0;

      publish_command(arms, phase, sign);

      if (now >= next_log)
      {
        print_progress(arms, phase, safe_duration - elapsed);
        next_log = now + 1s;
      }

      rate.sleep();
    }

    publish_stop_commands(arms);
    print_summary(arms);
  }

private:
  bool valid_command_type() const
  {
    return command_type_ == "pose" || command_type_ == "twist" || command_type_ == "joint";
  }

  std::vector<ServoArm> selected_arms() const
  {
    const ServoArm arm_a{
      "A", "/arm_A_servo_node/pose_target_cmds", "/arm_A_servo_node/delta_twist_cmds",
      "/arm_A_servo_node/delta_joint_cmds", "/arm_A_servo_node/status", "arm_A__tcp",
      { "arm_A_ur_shoulder_pan_joint", "arm_A_ur_shoulder_lift_joint", "arm_A_ur_elbow_joint",
        "arm_A_ur_wrist_1_joint", "arm_A_ur_wrist_2_joint", "arm_A_ur_wrist_3_joint" }
    };
    const ServoArm arm_b{
      "B", "/arm_B_servo_node/pose_target_cmds", "/arm_B_servo_node/delta_twist_cmds",
      "/arm_B_servo_node/delta_joint_cmds", "/arm_B_servo_node/status", "arm_B__tcp",
      { "arm_B_ur_shoulder_pan_joint", "arm_B_ur_shoulder_lift_joint", "arm_B_ur_elbow_joint",
        "arm_B_ur_wrist_1_joint", "arm_B_ur_wrist_2_joint", "arm_B_ur_wrist_3_joint" }
    };

    if (servo_target_ == "A" || servo_target_ == "a")
    {
      return { arm_a };
    }
    if (servo_target_ == "B" || servo_target_ == "b")
    {
      return { arm_b };
    }
    if (servo_target_ == "both" || servo_target_ == "Both" || servo_target_ == "BOTH")
    {
      return { arm_a, arm_b };
    }
    return {};
  }

  void create_publishers_and_subscribers(const std::vector<ServoArm>& arms)
  {
    for (const auto& arm : arms)
    {
      if (command_type_ == "pose")
      {
        pose_publishers_[arm.id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(arm.pose_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing arm_%s PoseStamped to %s", arm.id.c_str(), arm.pose_topic.c_str());
      }
      else if (command_type_ == "twist")
      {
        twist_publishers_[arm.id] = this->create_publisher<geometry_msgs::msg::TwistStamped>(arm.twist_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing arm_%s TwistStamped to %s", arm.id.c_str(), arm.twist_topic.c_str());
      }
      else
      {
        joint_publishers_[arm.id] = this->create_publisher<control_msgs::msg::JointJog>(arm.joint_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing arm_%s JointJog to %s", arm.id.c_str(), arm.joint_topic.c_str());
      }

      status_subs_.push_back(this->create_subscription<moveit_msgs::msg::ServoStatus>(
        arm.status_topic, 10,
        [this, id = arm.id](const moveit_msgs::msg::ServoStatus::SharedPtr msg) {
          latest_status_[id] = *msg;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Servo status arm_%s: %s", id.c_str(), status_to_string(*msg).c_str());
        }));
      RCLCPP_INFO(this->get_logger(), "Subscribed arm_%s ServoStatus from %s", arm.id.c_str(), arm.status_topic.c_str());
    }
  }

  bool capture_start_poses(const std::vector<ServoArm>& arms)
  {
    RCLCPP_INFO(this->get_logger(), "Capturing start poses from TF frame %s", planning_frame_.c_str());
    bool ok = true;
    for (const auto& arm : arms)
    {
      try
      {
        const auto transform = tf_buffer_.lookupTransform(planning_frame_, arm.tcp_frame, tf2::TimePointZero, 5s);
        start_poses_[arm.id] = pose_from_transform(transform);
        RCLCPP_INFO(this->get_logger(), "arm_%s start pose captured from %s -> %s", arm.id.c_str(),
                    planning_frame_.c_str(), arm.tcp_frame.c_str());
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup TF %s -> %s: %s", planning_frame_.c_str(),
                     arm.tcp_frame.c_str(), ex.what());
        ok = false;
      }
    }
    return ok;
  }

  void publish_command(const std::vector<ServoArm>& arms, int phase, double sign)
  {
    for (const auto& arm : arms)
    {
      if (command_type_ == "pose")
      {
        pose_publishers_[arm.id]->publish(make_pose_command(arm, phase, sign));
      }
      else if (command_type_ == "twist")
      {
        twist_publishers_[arm.id]->publish(make_twist_command(arm, phase, sign));
      }
      else
      {
        joint_publishers_[arm.id]->publish(make_joint_command(arm, phase, sign));
      }
    }
  }

  geometry_msgs::msg::PoseStamped make_pose_command(const ServoArm& arm, int phase, double sign) const
  {
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    double droll = 0.0;
    double dpitch = 0.0;
    double dyaw = 0.0;

    switch (phase)
    {
      case 0:
        dx = sign * position_delta_m_;
        break;
      case 1:
        dy = sign * position_delta_m_;
        break;
      case 2:
        dz = sign * position_delta_m_;
        break;
      case 3:
        droll = sign * orientation_delta_rad_;
        break;
      case 4:
        dpitch = sign * orientation_delta_rad_;
        break;
      default:
        dyaw = sign * orientation_delta_rad_;
        break;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = planning_frame_;
    pose.pose = offset_pose(start_poses_.at(arm.id), dx, dy, dz, droll, dpitch, dyaw);
    return pose;
  }

  geometry_msgs::msg::TwistStamped make_twist_command(const ServoArm& arm, int phase, double sign) const
  {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = arm.tcp_frame;

    switch (phase)
    {
      case 0:
        twist.twist.linear.x = sign * linear_speed_;
        break;
      case 1:
        twist.twist.linear.y = sign * linear_speed_;
        break;
      case 2:
        twist.twist.linear.z = sign * linear_speed_;
        break;
      case 3:
        twist.twist.angular.x = sign * angular_speed_;
        break;
      case 4:
        twist.twist.angular.y = sign * angular_speed_;
        break;
      default:
        twist.twist.angular.z = sign * angular_speed_;
        break;
    }

    return twist;
  }

  control_msgs::msg::JointJog make_joint_command(const ServoArm& arm, int phase, double sign) const
  {
    control_msgs::msg::JointJog jog;
    jog.header.stamp = this->now();
    jog.header.frame_id = arm.tcp_frame;
    const std::size_t index = static_cast<std::size_t>(std::clamp(phase, 0, 5));
    jog.joint_names = { arm.joint_names[index] };
    jog.displacements = { sign * joint_delta_rad_ };
    jog.velocities = { sign * joint_velocity_rad_s_ };
    jog.duration = 1.0 / std::max(1.0, publish_rate_hz_);
    return jog;
  }

  std::string phase_name(int phase) const
  {
    switch (phase)
    {
      case 0:
        return command_type_ == "joint" ? "joint_1" : "x_or_roll_stage_1";
      case 1:
        return command_type_ == "joint" ? "joint_2" : "y_or_pitch_stage_2";
      case 2:
        return command_type_ == "joint" ? "joint_3" : "z_or_yaw_stage_3";
      case 3:
        return command_type_ == "joint" ? "joint_4" : "roll";
      case 4:
        return command_type_ == "joint" ? "joint_5" : "pitch";
      default:
        return command_type_ == "joint" ? "joint_6" : "yaw";
    }
  }

  bool publisher_has_subscribers(const ServoArm& arm) const
  {
    if (command_type_ == "pose")
    {
      return pose_publishers_.at(arm.id)->get_subscription_count() > 0;
    }
    if (command_type_ == "twist")
    {
      return twist_publishers_.at(arm.id)->get_subscription_count() > 0;
    }
    return joint_publishers_.at(arm.id)->get_subscription_count() > 0;
  }

  std::string command_topic(const ServoArm& arm) const
  {
    if (command_type_ == "pose")
    {
      return arm.pose_topic;
    }
    if (command_type_ == "twist")
    {
      return arm.twist_topic;
    }
    return arm.joint_topic;
  }

  void wait_for_servo_subscribers(const std::vector<ServoArm>& arms)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      bool all_ready = true;
      for (const auto& arm : arms)
      {
        all_ready = all_ready && publisher_has_subscribers(arm);
      }
      if (all_ready)
      {
        RCLCPP_INFO(this->get_logger(), "All selected Servo command topics have subscribers.");
        return;
      }
      std::this_thread::sleep_for(100ms);
    }

    for (const auto& arm : arms)
    {
      if (!publisher_has_subscribers(arm))
      {
        RCLCPP_WARN(this->get_logger(), "No subscriber detected on %s before streaming", command_topic(arm).c_str());
      }
    }
  }

  void print_progress(const std::vector<ServoArm>& arms, int phase, double remaining_sec) const
  {
    RCLCPP_INFO(this->get_logger(), "Servo command_type=%s phase=%s remaining=%.1fs", command_type_.c_str(),
                phase_name(phase).c_str(), remaining_sec);
    for (const auto& arm : arms)
    {
      const auto it = latest_status_.find(arm.id);
      const std::string status = it == latest_status_.end() ? "no status received yet" : status_to_string(it->second);
      RCLCPP_INFO(this->get_logger(), "arm_%s topic=%s last_status=%s", arm.id.c_str(), command_topic(arm).c_str(),
                  status.c_str());
    }
  }

  void publish_stop_commands(const std::vector<ServoArm>& arms)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing stop commands for command_type=%s", command_type_.c_str());
    rclcpp::Rate rate(std::max(1.0, publish_rate_hz_));
    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      for (const auto& arm : arms)
      {
        if (command_type_ == "pose")
        {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = this->now();
          pose.header.frame_id = planning_frame_;
          pose.pose = start_poses_.at(arm.id);
          pose_publishers_[arm.id]->publish(pose);
        }
        else if (command_type_ == "twist")
        {
          geometry_msgs::msg::TwistStamped twist;
          twist.header.stamp = this->now();
          twist.header.frame_id = arm.tcp_frame;
          twist_publishers_[arm.id]->publish(twist);
        }
        else
        {
          control_msgs::msg::JointJog jog;
          jog.header.stamp = this->now();
          jog.header.frame_id = arm.tcp_frame;
          jog.joint_names = arm.joint_names;
          jog.displacements.assign(arm.joint_names.size(), 0.0);
          jog.velocities.assign(arm.joint_names.size(), 0.0);
          jog.duration = 1.0 / std::max(1.0, publish_rate_hz_);
          joint_publishers_[arm.id]->publish(jog);
        }
      }
      rate.sleep();
    }
  }

  void print_summary(const std::vector<ServoArm>& arms) const
  {
    RCLCPP_INFO(this->get_logger(), "=== MoveIt Servo test summary ===");
    for (const auto& arm : arms)
    {
      const auto it = latest_status_.find(arm.id);
      const std::string status = it == latest_status_.end() ? "no status received" : status_to_string(it->second);
      RCLCPP_INFO(this->get_logger(), "arm_%s final_status=%s", arm.id.c_str(), status.c_str());
    }
  }

  std::string servo_target_;
  std::string command_type_;
  std::string planning_frame_;
  double duration_sec_ = 30.0;
  double publish_rate_hz_ = 50.0;
  double linear_speed_ = 0.01;
  double angular_speed_ = 0.05;
  double position_delta_m_ = 0.02;
  double orientation_delta_rad_ = 0.05;
  double joint_delta_rad_ = 0.03;
  double joint_velocity_rad_s_ = 0.05;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, geometry_msgs::msg::Pose> start_poses_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_publishers_;
  std::map<std::string, rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr> joint_publishers_;
  std::vector<rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr> status_subs_;
  std::map<std::string, moveit_msgs::msg::ServoStatus> latest_status_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveServoTester>();

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
