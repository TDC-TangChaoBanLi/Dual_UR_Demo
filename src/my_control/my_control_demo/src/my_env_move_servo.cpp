#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace
{
struct ServoArm
{
  std::string id;              // "A" / "B"
  std::string pose_topic;      // Servo 位姿指令话题
  std::string status_topic;    // Servo 状态话题
  std::string planning_frame;  // 位姿目标和误差计算的参考系
  std::string tcp_frame;       // 实际 TCP 的 TF frame
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
    arm_a_ = declare_arm_parameters(
      "A", "world", "arm_A__tcp", "/arm_A_servo_node/pose_target_cmds", "/arm_A_servo_node/status");
    arm_b_ = declare_arm_parameters(
      "B", "world", "arm_B__tcp", "/arm_B_servo_node/pose_target_cmds", "/arm_B_servo_node/status");

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);

    // 正弦轨迹参数
    sine_period_sec_ = this->declare_parameter<double>("sine_period_sec", 6.0);
    num_periods_ = this->declare_parameter<double>("num_periods", 3.0);
    position_amplitude_m_ = this->declare_parameter<double>("position_amplitude_m", 0.05);
    orientation_amplitude_rad_ = this->declare_parameter<double>("orientation_amplitude_rad", 0.1);
    settle_time_sec_ = this->declare_parameter<double>("settle_time_sec", 1.0);

    RCLCPP_INFO(this->get_logger(), "my_env_move_servo initialized");
    RCLCPP_INFO(
      this->get_logger(),
      "arm_A: planning_frame=%s tcp_frame=%s pose_topic=%s status_topic=%s",
      arm_a_.planning_frame.c_str(), arm_a_.tcp_frame.c_str(), arm_a_.pose_topic.c_str(), arm_a_.status_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "arm_B: planning_frame=%s tcp_frame=%s pose_topic=%s status_topic=%s",
      arm_b_.planning_frame.c_str(), arm_b_.tcp_frame.c_str(), arm_b_.pose_topic.c_str(), arm_b_.status_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "sine trajectory: publish_rate=%.2f Hz period=%.2fs num_periods=%.2f position_amplitude=%.4f m "
      "orientation_amplitude=%.4f rad settle_time=%.2fs",
      publish_rate_hz_, sine_period_sec_, num_periods_, position_amplitude_m_, orientation_amplitude_rad_,
      settle_time_sec_);
  }

  void run()
  {
    const std::vector<ServoArm> arms{ arm_a_, arm_b_ };

    create_publishers_and_subscribers(arms);
    wait_for_servo_subscribers(arms);

    const auto stages = build_stages(arms);
    RCLCPP_INFO(this->get_logger(), "Prepared %zu sine test stage(s).", stages.size());

    for (std::size_t i = 0; i < stages.size() && rclcpp::ok(); ++i)
    {
      const auto& stage = stages[i];
      RCLCPP_INFO(this->get_logger(),
                  "===== Stage %zu/%zu '%s': arms=%zu position=%s orientation=%s =====",
                  i + 1, stages.size(), stage.name.c_str(), stage.arms.size(),
                  stage.use_position ? "yes" : "no", stage.use_orientation ? "yes" : "no");

      if (!capture_start_poses(stage.arms))
      {
        RCLCPP_ERROR(this->get_logger(), "Skipping stage '%s' due to missing start pose.", stage.name.c_str());
        continue;
      }

      run_sine_stage(stage);
      publish_hold(stage.arms, settle_time_sec_);
    }

    RCLCPP_INFO(this->get_logger(), "All sine test stages finished.");
    print_summary(arms);
  }

private:
  struct SineStage
  {
    std::string name;
    std::vector<ServoArm> arms;
    bool use_position;
    bool use_orientation;
  };

  struct TrackingErrorSample
  {
    double elapsed_sec;
    double position_error_m;
    double orientation_error_rad;
  };

  struct ArmStageErrorSamples
  {
    std::vector<TrackingErrorSample> samples;
    std::size_t unavailable_samples = 0;
  };

  struct StageErrorRecord
  {
    std::string stage_name;
    std::map<std::string, ArmStageErrorSamples> arm_samples;
  };

  struct ErrorStatistics
  {
    std::size_t count = 0;
    double maximum = 0.0;
    double mean = 0.0;
    double standard_deviation = 0.0;
  };

  ServoArm declare_arm_parameters(
    const std::string& id, const std::string& default_planning_frame, const std::string& default_tcp_frame,
    const std::string& default_pose_topic, const std::string& default_status_topic)
  {
    const std::string prefix = "arm_" + std::string(1, static_cast<char>(std::tolower(id.front()))) + "_";
    return ServoArm{
      id,
      this->declare_parameter<std::string>(prefix + "pose_topic", default_pose_topic),
      this->declare_parameter<std::string>(prefix + "status_topic", default_status_topic),
      this->declare_parameter<std::string>(prefix + "planning_frame", default_planning_frame),
      this->declare_parameter<std::string>(prefix + "tcp_frame", default_tcp_frame),
    };
  }

  // 构建测试阶段列表：
  // 单臂阶段依次对 A、B 执行位置 / 姿态 / 位姿三项，最后追加双臂同步位姿阶段。
  std::vector<SineStage> build_stages(const std::vector<ServoArm>& arms) const
  {
    std::vector<SineStage> stages;
    for (const auto& arm : arms)
    {
      stages.push_back({ "arm_" + arm.id + "_position", { arm }, true, false });
      stages.push_back({ "arm_" + arm.id + "_orientation", { arm }, false, true });
      stages.push_back({ "arm_" + arm.id + "_pose", { arm }, true, true });
    }
    if (arms.size() > 1)
    {
      stages.push_back({ "arm_BOTH_pose", arms, true, true });
    }
    return stages;
  }

  // 运行单个正弦阶段：X/Y/Z（或 roll/pitch/yaw）依次相差 120°。
  void run_sine_stage(const SineStage& stage)
  {
    const double stage_duration = std::max(0.0, num_periods_) * std::max(0.1, sine_period_sec_);
    const double omega = 2.0 * M_PI / std::max(0.1, sine_period_sec_);
    const auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(std::max(1.0, publish_rate_hz_));
    StageErrorRecord error_record;
    error_record.stage_name = stage.name;
    for (const auto& arm : stage.arms)
    {
      error_record.arm_samples.emplace(arm.id, ArmStageErrorSamples{});
    }

    RCLCPP_INFO(this->get_logger(), "Running sine stage '%s' for %.2fs", stage.name.c_str(), stage_duration);
    while (rclcpp::ok())
    {
      const auto now = std::chrono::steady_clock::now();
      const double t = std::chrono::duration<double>(now - start).count();
      if (t >= stage_duration)
      {
        break;
      }

      for (const auto& arm : stage.arms)
      {
        const auto pose_cmd = make_sine_pose(arm, omega * t, stage.use_position, stage.use_orientation);
        last_target_poses_[arm.id] = pose_cmd.pose;
        pose_publishers_[arm.id]->publish(pose_cmd);
      }

      // 每个目标发布周期都采集一次“最新目标 vs 最新 TCP TF”的误差。
      for (const auto& arm : stage.arms)
      {
        double position_error_m = 0.0;
        double orientation_error_rad = 0.0;
        auto& arm_error_samples = error_record.arm_samples.at(arm.id);
        if (compute_pose_tracking_error(arm, position_error_m, orientation_error_rad))
        {
          arm_error_samples.samples.push_back({ t, position_error_m, orientation_error_rad });
        }
        else
        {
          ++arm_error_samples.unavailable_samples;
        }
      }

      rate.sleep();
    }

    print_stage_error_summary(error_record);
    stage_error_history_.push_back(std::move(error_record));
  }

  // 三轴相位依次为 0、2π/3、4π/3。减去各轴在 phase=0 时的值，
  // 使目标在阶段起点以及完整周期结束时都严格等于捕获的起始位姿，避免指令跳变。
  geometry_msgs::msg::PoseStamped make_sine_pose(
    const ServoArm& arm, double phase, bool use_position, bool use_orientation) const
  {
    constexpr double phase_x = 4.0 * M_PI / 3.0;
    constexpr double phase_y = 0.0;
    constexpr double phase_z = 2.0 * M_PI / 3.0;
    const auto phase_shifted_sine = [phase](double axis_phase) {
      return std::sin(phase + axis_phase) - std::sin(axis_phase);
    };

    const double sx = phase_shifted_sine(phase_x);
    const double sy = phase_shifted_sine(phase_y);
    const double sz = phase_shifted_sine(phase_z);

    const double dx = use_position ? sx * position_amplitude_m_ : 0.0;
    const double dy = use_position ? sy * position_amplitude_m_ : 0.0;
    const double dz = use_position ? sz * position_amplitude_m_ : 0.0;
    const double droll = use_orientation ? sx * orientation_amplitude_rad_ : 0.0;
    const double dpitch = use_orientation ? sy * orientation_amplitude_rad_ : 0.0;
    const double dyaw = use_orientation ? sz * orientation_amplitude_rad_ : 0.0;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = arm.planning_frame;
    pose.pose = offset_pose(start_poses_.at(arm.id), dx, dy, dz, droll, dpitch, dyaw);
    return pose;
  }

  // 在阶段结束后，持续发布起点位姿，让机械臂回到起点并稳定。
  void publish_hold(const std::vector<ServoArm>& arms, double hold_sec)
  {
    if (hold_sec <= 0.0)
    {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(std::max(1.0, publish_rate_hz_));
    while (rclcpp::ok())
    {
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (t >= hold_sec)
      {
        break;
      }
      for (const auto& arm : arms)
      {
        const auto it = start_poses_.find(arm.id);
        if (it == start_poses_.end())
        {
          continue;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = arm.planning_frame;
        pose.pose = it->second;
        pose_publishers_[arm.id]->publish(pose);
      }
      rate.sleep();
    }
  }

  void create_publishers_and_subscribers(const std::vector<ServoArm>& arms)
  {
    for (const auto& arm : arms)
    {
      pose_publishers_[arm.id] = this->create_publisher<geometry_msgs::msg::PoseStamped>(arm.pose_topic, 10);
      RCLCPP_INFO(this->get_logger(), "Publishing arm_%s PoseStamped to %s", arm.id.c_str(), arm.pose_topic.c_str());

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
    bool ok = true;
    for (const auto& arm : arms)
    {
      try
      {
        const auto transform = tf_buffer_.lookupTransform(arm.planning_frame, arm.tcp_frame, tf2::TimePointZero, 5s);
        start_poses_[arm.id] = pose_from_transform(transform);
        RCLCPP_INFO(this->get_logger(), "arm_%s start pose captured from %s -> %s", arm.id.c_str(),
                    arm.planning_frame.c_str(), arm.tcp_frame.c_str());
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup TF %s -> %s: %s", arm.planning_frame.c_str(),
                     arm.tcp_frame.c_str(), ex.what());
        ok = false;
      }
    }
    return ok;
  }

  void wait_for_servo_subscribers(const std::vector<ServoArm>& arms)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      bool all_ready = true;
      for (const auto& arm : arms)
      {
        all_ready = all_ready && (pose_publishers_.at(arm.id)->get_subscription_count() > 0);
      }
      if (all_ready)
      {
        RCLCPP_INFO(this->get_logger(), "All selected Servo pose command topics have subscribers.");
        return;
      }
      std::this_thread::sleep_for(100ms);
    }

    for (const auto& arm : arms)
    {
      if (pose_publishers_.at(arm.id)->get_subscription_count() == 0)
      {
        RCLCPP_WARN(this->get_logger(), "No subscriber detected on %s before streaming", arm.pose_topic.c_str());
      }
    }
  }

  // Compute tracking error between the last commanded target pose and the
  // current actual TCP pose read from TF.  Returns false if unavailable
  // (e.g. no target sent yet, or TF lookup failed).
  bool compute_pose_tracking_error(const ServoArm& arm, double& position_error_m, double& orientation_error_rad) const
  {
    const auto target_it = last_target_poses_.find(arm.id);
    if (target_it == last_target_poses_.end())
    {
      return false;
    }

    geometry_msgs::msg::Pose actual;
    try
    {
      const auto transform = tf_buffer_.lookupTransform(arm.planning_frame, arm.tcp_frame, tf2::TimePointZero);
      actual = pose_from_transform(transform);
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }

    const auto& target = target_it->second;
    const double dx = target.position.x - actual.position.x;
    const double dy = target.position.y - actual.position.y;
    const double dz = target.position.z - actual.position.z;
    position_error_m = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion q_target;
    tf2::Quaternion q_actual;
    tf2::fromMsg(target.orientation, q_target);
    tf2::fromMsg(actual.orientation, q_actual);
    // Relative rotation from actual to target; its angle is the orientation error.
    const tf2::Quaternion q_error = q_target * q_actual.inverse();
    orientation_error_rad = std::abs(q_error.normalized().getAngle());
    if (orientation_error_rad > M_PI)
    {
      orientation_error_rad = 2.0 * M_PI - orientation_error_rad;
    }
    return true;
  }

  static ErrorStatistics calculate_statistics(
    const std::vector<TrackingErrorSample>& samples, bool use_position_error)
  {
    ErrorStatistics statistics;
    statistics.count = samples.size();
    if (samples.empty())
    {
      return statistics;
    }

    // Welford 在线算法的离线等价写法，避免大样本求和时的数值消减。
    double mean = 0.0;
    double squared_difference_sum = 0.0;
    std::size_t count = 0;
    for (const auto& sample : samples)
    {
      const double value = use_position_error ? sample.position_error_m : sample.orientation_error_rad;
      statistics.maximum = std::max(statistics.maximum, value);
      ++count;
      const double delta = value - mean;
      mean += delta / static_cast<double>(count);
      const double delta_after_mean_update = value - mean;
      squared_difference_sum += delta * delta_after_mean_update;
    }
    statistics.mean = mean;
    statistics.standard_deviation = std::sqrt(squared_difference_sum / static_cast<double>(count));
    return statistics;
  }

  void print_stage_error_summary(const StageErrorRecord& record) const
  {
    RCLCPP_INFO(this->get_logger(), "===== Stage '%s' tracking error summary =====", record.stage_name.c_str());
    for (const auto& [arm_id, arm_error_samples] : record.arm_samples)
    {
      const auto position = calculate_statistics(arm_error_samples.samples, true);
      const auto orientation = calculate_statistics(arm_error_samples.samples, false);
      if (position.count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "arm_%s: no valid tracking error samples; unavailable=%zu", arm_id.c_str(),
                    arm_error_samples.unavailable_samples);
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
                  "arm_%s samples=%zu unavailable=%zu position_error[m]: max=%.6f mean=%.6f stddev=%.6f",
                  arm_id.c_str(), position.count, arm_error_samples.unavailable_samples, position.maximum,
                  position.mean, position.standard_deviation);
      RCLCPP_INFO(this->get_logger(),
                  "arm_%s orientation_error[rad]: max=%.6f mean=%.6f stddev=%.6f "
                  "([deg]: max=%.3f mean=%.3f stddev=%.3f)",
                  arm_id.c_str(), orientation.maximum, orientation.mean, orientation.standard_deviation,
                  orientation.maximum * 180.0 / M_PI, orientation.mean * 180.0 / M_PI,
                  orientation.standard_deviation * 180.0 / M_PI);
    }
  }

  void print_summary(const std::vector<ServoArm>& arms) const
  {
    RCLCPP_INFO(this->get_logger(), "=== MoveIt Servo test summary ===");
    RCLCPP_INFO(this->get_logger(), "Stored tracking error history for %zu completed stage(s).",
                stage_error_history_.size());
    for (const auto& arm : arms)
    {
      const auto it = latest_status_.find(arm.id);
      const std::string status = it == latest_status_.end() ? "no status received" : status_to_string(it->second);
      RCLCPP_INFO(this->get_logger(), "arm_%s final_status=%s", arm.id.c_str(), status.c_str());
    }
  }

  ServoArm arm_a_;
  ServoArm arm_b_;
  double publish_rate_hz_ = 100.0;
  double sine_period_sec_ = 6.0;
  double num_periods_ = 3.0;
  double position_amplitude_m_ = 0.05;
  double orientation_amplitude_rad_ = 0.1;
  double settle_time_sec_ = 1.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, geometry_msgs::msg::Pose> start_poses_;
  std::map<std::string, geometry_msgs::msg::Pose> last_target_poses_;
  std::vector<StageErrorRecord> stage_error_history_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;
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
