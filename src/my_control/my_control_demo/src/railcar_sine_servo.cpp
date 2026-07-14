#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{
constexpr std::chrono::milliseconds kSendPeriod{ 50 };
constexpr std::chrono::seconds kReplyTimeout{ 1 };

// 与 railcar_sine_direct.py 相同的固定命令：0.1 Hz、20 mm、5 个周期。
constexpr std::uint8_t kSineCommand[] = {
  0xAA, 0x55, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A,
  0x00, 0xC8, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xAA
};
constexpr std::uint8_t kStopCommand[] = {
  0xAA, 0x55, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xAA
};
constexpr std::size_t kStatusSize = 18;

std::int16_t read_i16_le(const std::uint8_t* data)
{
  const auto value = static_cast<std::uint16_t>(data[0]) |
                     (static_cast<std::uint16_t>(data[1]) << 8U);
  return static_cast<std::int16_t>(value);
}

std::string socket_error(const std::string& operation)
{
  return operation + ": " + std::strerror(errno);
}
}  // namespace

class RailcarSineServo : public rclcpp::Node
{
public:
  RailcarSineServo()
  : Node("railcar_sine_servo"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/arm_A_servo_node/pose_target_cmds");
    railcar_ip_ = this->declare_parameter<std::string>("railcar_ip", "192.168.1.88");
    railcar_port_ = this->declare_parameter<int>("railcar_port", 6000);
    local_ip_ = this->declare_parameter<std::string>("local_ip", "");
    local_port_ = this->declare_parameter<int>("local_port", 6000);

    pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
  }

  ~RailcarSineServo() override
  {
    close_socket();
  }

  void run()
  {
    wait_for_pose_subscriber();
    if (!capture_initial_pose())
    {
      return;
    }

    try
    {
      open_socket();
      RCLCPP_INFO(
        this->get_logger(),
        "Railcar sine tracking started: %s:%d -> %s, target_y = initial_y + railcar_position",
        railcar_ip_.c_str(), railcar_port_, pose_topic_.c_str());

      auto next_cycle = std::chrono::steady_clock::now();
      while (rclcpp::ok())
      {
        const auto [position_mm, speed_mm_s] = send_sine_command();

        geometry_msgs::msg::PoseStamped target;
        target.header.stamp = this->now();
        target.header.frame_id = planning_frame_;
        target.pose = initial_pose_;
        target.pose.position.y = initial_pose_.position.y + position_mm / 1000.0;
        pose_publisher_->publish(target);

        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "railcar_position=%.1f mm speed=%.0f mm/s, target_y=%.6f m",
          position_mm, speed_mm_s, target.pose.position.y);

        next_cycle += kSendPeriod;
        const auto now = std::chrono::steady_clock::now();
        if (next_cycle > now)
        {
          std::this_thread::sleep_until(next_cycle);
        }
        else
        {
          // 通信耗时超过一个周期时不累积补发命令。
          next_cycle = now;
        }
      }
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(this->get_logger(), "Railcar communication failed: %s", error.what());
    }

    send_stop();
    close_socket();
  }

private:
  void wait_for_pose_subscriber()
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (
      rclcpp::ok() && pose_publisher_->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(100ms);
    }
    if (pose_publisher_->get_subscription_count() == 0)
    {
      RCLCPP_WARN(this->get_logger(), "No subscriber detected on %s", pose_topic_.c_str());
    }
  }

  bool capture_initial_pose()
  {
    try
    {
      const auto transform =
        tf_buffer_.lookupTransform(planning_frame_, tcp_frame_, tf2::TimePointZero, 5s);
      initial_pose_.position.x = transform.transform.translation.x;
      initial_pose_.position.y = transform.transform.translation.y;
      initial_pose_.position.z = transform.transform.translation.z;
      initial_pose_.orientation = transform.transform.rotation;
      RCLCPP_INFO(
        this->get_logger(), "Initial TCP pose: xyz=[%.6f, %.6f, %.6f]",
        initial_pose_.position.x, initial_pose_.position.y, initial_pose_.position.z);
      return true;
    }
    catch (const tf2::TransformException& error)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to read initial TCP TF %s -> %s: %s",
        planning_frame_.c_str(), tcp_frame_.c_str(), error.what());
      return false;
    }
  }

  void open_socket()
  {
    if (railcar_port_ < 1 || railcar_port_ > 65535 || local_port_ < 0 || local_port_ > 65535)
    {
      throw std::runtime_error("UDP port must be in the valid range");
    }

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      throw std::runtime_error(socket_error("socket"));
    }

    timeval timeout{};
    timeout.tv_sec = kReplyTimeout.count();
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
      throw std::runtime_error(socket_error("setsockopt(SO_RCVTIMEO)"));
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<std::uint16_t>(local_port_));
    if (local_ip_.empty())
    {
      local.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else if (::inet_pton(AF_INET, local_ip_.c_str(), &local.sin_addr) != 1)
    {
      throw std::runtime_error("Invalid local_ip: " + local_ip_);
    }
    if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) < 0)
    {
      throw std::runtime_error(socket_error("bind"));
    }

    railcar_address_ = {};
    railcar_address_.sin_family = AF_INET;
    railcar_address_.sin_port = htons(static_cast<std::uint16_t>(railcar_port_));
    if (::inet_pton(AF_INET, railcar_ip_.c_str(), &railcar_address_.sin_addr) != 1)
    {
      throw std::runtime_error("Invalid railcar_ip: " + railcar_ip_);
    }
  }

  std::pair<double, double> send_sine_command()
  {
    const auto sent = ::sendto(
      socket_fd_, kSineCommand, sizeof(kSineCommand), 0,
      reinterpret_cast<const sockaddr*>(&railcar_address_), sizeof(railcar_address_));
    if (sent != static_cast<ssize_t>(sizeof(kSineCommand)))
    {
      throw std::runtime_error(socket_error("sendto"));
    }

    const auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
    while (rclcpp::ok())
    {
      std::uint8_t data[65535];
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t received = ::recvfrom(
        socket_fd_, data, sizeof(data), 0,
        reinterpret_cast<sockaddr*>(&source), &source_size);
      if (received < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        throw std::runtime_error(
          errno == EAGAIN || errno == EWOULDBLOCK ? "waiting for railcar status timed out" :
                                                    socket_error("recvfrom"));
      }

      if (
        source.sin_addr.s_addr != railcar_address_.sin_addr.s_addr ||
        source.sin_port != railcar_address_.sin_port)
      {
        if (std::chrono::steady_clock::now() >= deadline)
        {
          throw std::runtime_error("waiting for railcar status timed out");
        }
        continue;
      }

      const std::uint8_t* frame = extract_status_frame(data, static_cast<std::size_t>(received));
      if (frame == nullptr)
      {
        throw std::runtime_error("railcar reply does not contain a complete status frame");
      }

      // 状态帧偏移：速度 4（1 mm/s），当前位置 6（0.1 mm）。
      return { static_cast<double>(read_i16_le(frame + 6)) * 0.1,
               static_cast<double>(read_i16_le(frame + 4)) };
    }
    throw std::runtime_error("ROS shutdown requested");
  }

  static const std::uint8_t* extract_status_frame(const std::uint8_t* data, std::size_t size)
  {
    for (std::size_t start = 0; start + kStatusSize <= size; ++start)
    {
      if (data[start] != 0x55 || data[start + 1] != 0xAA)
      {
        continue;
      }
      if (data[start + 16] == 0xAA && data[start + 17] == 0x55)
      {
        return data + start;
      }
      // 兼容状态字段和帧尾之间多两个 00 的 20 字节抓包形式。该形式中
      // 当前位置/速度偏移不变，因此可以直接返回起始地址。
      if (
        start + 20 <= size && data[start + 16] == 0x00 && data[start + 17] == 0x00 &&
        data[start + 18] == 0xAA && data[start + 19] == 0x55)
      {
        return data + start;
      }
    }
    return nullptr;
  }

  void send_stop() noexcept
  {
    if (socket_fd_ < 0)
    {
      return;
    }
    (void)::sendto(
      socket_fd_, kStopCommand, sizeof(kStopCommand), 0,
      reinterpret_cast<const sockaddr*>(&railcar_address_), sizeof(railcar_address_));
    RCLCPP_INFO(this->get_logger(), "Stop command sent to railcar");
  }

  void close_socket() noexcept
  {
    if (socket_fd_ >= 0)
    {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  std::string pose_topic_;
  std::string railcar_ip_;
  int railcar_port_ = 6000;
  std::string local_ip_;
  int local_port_ = 6000;
  const std::string planning_frame_{ "world" };
  const std::string tcp_frame_{ "arm_A__tcp" };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::Pose initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  int socket_fd_ = -1;
  sockaddr_in railcar_address_{};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RailcarSineServo>();

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
