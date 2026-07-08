#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
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
  std::string id;                            // "A" / "B"
  std::string node_name;                     // Servo 节点名，如 "/arm_A_servo_node"
  std::string pose_topic;                    // 查询得到并解析后的位姿指令话题
  std::string status_topic;                  // 查询得到并解析后的状态话题
  std::string switch_command_type_service;   // <node_name>/switch_command_type
  std::string planning_frame;                // 查询得到的规划参考系
  std::string tcp_frame;                     // 查询得到的末端执行器坐标系 (ee_frame)
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
    // 两个 Servo 节点名通过参数给入（始终测试两个臂）。
    arm_a_servo_node_ = this->declare_parameter<std::string>("arm_a_servo_node", "/arm_A_servo_node");
    arm_b_servo_node_ = this->declare_parameter<std::string>("arm_b_servo_node", "/arm_B_servo_node");

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
      "arm_a_servo_node=%s arm_b_servo_node=%s publish_rate_hz=%.2f",
      arm_a_servo_node_.c_str(), arm_b_servo_node_.c_str(), publish_rate_hz_);
    RCLCPP_INFO(
      this->get_logger(),
      "sine trajectory: period=%.2fs num_periods=%.2f position_amplitude=%.4f m "
      "orientation_amplitude=%.4f rad settle_time=%.2fs",
      sine_period_sec_, num_periods_, position_amplitude_m_, orientation_amplitude_rad_, settle_time_sec_);
  }

  void run()
  {
    const std::vector<std::pair<std::string, std::string>> arm_nodes{
      { "A", arm_a_servo_node_ },
      { "B", arm_b_servo_node_ },
    };

    std::vector<ServoArm> arms;
    for (const auto& [id, node_name] : arm_nodes)
    {
      ServoArm arm;
      if (!build_arm_from_servo_params(id, node_name, arm))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to query parameters from Servo node %s; aborting.",
                     node_name.c_str());
        return;
      }
      arms.push_back(arm);
    }

    create_publishers_and_subscribers(arms);
    wait_for_servo_subscribers(arms);

    // 将每个臂切换到位置伺服模式 (POSE = 2)。
    for (const auto& arm : arms)
    {
      switch_to_pose_command_type(arm);
    }

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

  // 将 Servo 的相对话题名（如 "~/pose_target_cmds" 或 "~/status"）解析为
  // 全局话题名 "<node_name>/<topic>"。已是绝对路径的原样返回。
  static std::string resolve_servo_topic(const std::string& node_name, const std::string& topic)
  {
    if (!topic.empty() && topic.front() == '/')
    {
      return topic;  // 已是绝对话题名
    }
    std::string relative = topic;
    if (!relative.empty() && relative.front() == '~')
    {
      relative.erase(relative.begin());  // 去掉 '~'
    }
    if (!relative.empty() && relative.front() == '/')
    {
      relative.erase(relative.begin());  // 去掉 '/'
    }
    return node_name + "/" + relative;
  }

  // 通过 <node_name>/get_parameters 服务从 Servo 节点查询 planning_frame、
  // ee_frame、pose_command_in_topic、status_topic，避免在此处写死。
  bool build_arm_from_servo_params(const std::string& id, const std::string& node_name, ServoArm& arm)
  {
    const std::string service_name = node_name + "/get_parameters";
    auto client = this->create_client<rcl_interfaces::srv::GetParameters>(service_name);
    if (!client->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter service not available: %s", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = { "moveit_servo.planning_frame", "moveit_servo.ee_frame",
                       "moveit_servo.pose_command_in_topic", "moveit_servo.status_topic" };

    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timed out querying parameters from %s", service_name.c_str());
      return false;
    }

    const auto response = future.get();
    if (response->values.size() != request->names.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Unexpected parameter count from %s", service_name.c_str());
      return false;
    }

    const std::string planning_frame = response->values[0].string_value;
    const std::string ee_frame = response->values[1].string_value;
    const std::string pose_topic = response->values[2].string_value;
    const std::string status_topic = response->values[3].string_value;

    if (planning_frame.empty() || ee_frame.empty() || pose_topic.empty() || status_topic.empty())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Servo node %s returned empty frame/topic parameters (planning_frame='%s' ee_frame='%s' "
                   "pose_topic='%s' status_topic='%s')",
                   node_name.c_str(), planning_frame.c_str(), ee_frame.c_str(), pose_topic.c_str(),
                   status_topic.c_str());
      return false;
    }

    arm.id = id;
    arm.node_name = node_name;
    arm.planning_frame = planning_frame;
    arm.tcp_frame = ee_frame;
    arm.pose_topic = resolve_servo_topic(node_name, pose_topic);
    arm.status_topic = resolve_servo_topic(node_name, status_topic);
    arm.switch_command_type_service = node_name + "/switch_command_type";

    RCLCPP_INFO(this->get_logger(),
                "arm_%s from %s: planning_frame=%s ee_frame=%s pose_topic=%s status_topic=%s",
                arm.id.c_str(), node_name.c_str(), arm.planning_frame.c_str(), arm.tcp_frame.c_str(),
                arm.pose_topic.c_str(), arm.status_topic.c_str());
    return true;
  }

  // 构建测试阶段列表：
  // 单臂阶段依次对每个选中臂执行 位置 / 姿态 / 位姿 三项；
  // 若选中 both，最后追加一个双臂同步位姿阶段。
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
      stages.push_back({ "dual_arm_pose", arms, true, true });
    }
    return stages;
  }

  // 通过 ServoCommandType 服务把指定臂切换到 POSE (2) 模式。
  void switch_to_pose_command_type(const ServoArm& arm)
  {
    auto client = this->create_client<moveit_msgs::srv::ServoCommandType>(arm.switch_command_type_service);
    if (!client->wait_for_service(5s))
    {
      RCLCPP_WARN(this->get_logger(), "Service %s not available; skipping command_type switch for arm_%s",
                  arm.switch_command_type_service.c_str(), arm.id.c_str());
      return;
    }

    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;

    auto future = client->async_send_request(request);
    if (future.wait_for(3s) != std::future_status::ready)
    {
      RCLCPP_WARN(this->get_logger(), "Timed out switching arm_%s to POSE command type", arm.id.c_str());
      return;
    }

    if (future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "arm_%s switched to POSE command type", arm.id.c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "arm_%s reported failure switching to POSE command type", arm.id.c_str());
    }
  }

  // 运行单个正弦阶段：目标 = 起点 + 幅值·sin(2π t / period)，各轴同相位同幅值。
  void run_sine_stage(const SineStage& stage)
  {
    const double stage_duration = std::max(0.0, num_periods_) * std::max(0.1, sine_period_sec_);
    const double omega = 2.0 * M_PI / std::max(0.1, sine_period_sec_);
    const auto start = std::chrono::steady_clock::now();
    auto next_log = start;
    rclcpp::Rate rate(std::max(1.0, publish_rate_hz_));

    RCLCPP_INFO(this->get_logger(), "Running sine stage '%s' for %.2fs", stage.name.c_str(), stage_duration);
    while (rclcpp::ok())
    {
      const auto now = std::chrono::steady_clock::now();
      const double t = std::chrono::duration<double>(now - start).count();
      if (t >= stage_duration)
      {
        break;
      }

      const double s = std::sin(omega * t);
      for (const auto& arm : stage.arms)
      {
        const auto pose_cmd = make_sine_pose(arm, s, stage.use_position, stage.use_orientation);
        last_target_poses_[arm.id] = pose_cmd.pose;
        pose_publishers_[arm.id]->publish(pose_cmd);
      }

      if (now >= next_log)
      {
        print_progress(stage.arms, stage.name, stage_duration - t);
        next_log = now + 1s;
      }

      rate.sleep();
    }
  }

  // 目标位姿 = 起点位姿 + sin 系数 s * 幅值（position/orientation 由 stage 决定是否启用）。
  geometry_msgs::msg::PoseStamped make_sine_pose(
    const ServoArm& arm, double s, bool use_position, bool use_orientation) const
  {
    const double dpos = use_position ? s * position_amplitude_m_ : 0.0;
    const double dori = use_orientation ? s * orientation_amplitude_rad_ : 0.0;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = arm.planning_frame;
    pose.pose = offset_pose(start_poses_.at(arm.id), dpos, dpos, dpos, dori, dori, dori);
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

  void print_progress(const std::vector<ServoArm>& arms, const std::string& stage_name, double remaining_sec) const
  {
    RCLCPP_INFO(this->get_logger(), "Stage '%s' remaining=%.1fs", stage_name.c_str(), remaining_sec);
    for (const auto& arm : arms)
    {
      double position_error_m = 0.0;
      double orientation_error_rad = 0.0;
      if (compute_pose_tracking_error(arm, position_error_m, orientation_error_rad))
      {
        RCLCPP_INFO(this->get_logger(),
                    "arm_%s tracking error: position=%.4f m orientation=%.4f rad (%.2f deg)", arm.id.c_str(),
                    position_error_m, orientation_error_rad, orientation_error_rad * 180.0 / M_PI);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "arm_%s tracking error: unavailable (no target or TF lookup failed)",
                    arm.id.c_str());
      }
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

  std::string arm_a_servo_node_;
  std::string arm_b_servo_node_;
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
