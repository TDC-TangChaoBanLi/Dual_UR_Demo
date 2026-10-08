#include "space_sim_task_control/dock_client.hpp"

#include <cmath>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/quaternion.hpp"

namespace space_sim_task_control
{

DockClient::DockClient(const rclcpp::Node::SharedPtr & node, double call_timeout_s)
: node_(node), call_timeout_s_(call_timeout_s)
{
  dock_client_ = node_->create_client<Dock>("/mujoco_dock_sim_plugin/dock");
  release_client_ = node_->create_client<Release>("/mujoco_dock_sim_plugin/release");
}

bool DockClient::waitForServices(double timeout_s)
{
  const auto dur = std::chrono::duration<double>(timeout_s);
  return dock_client_->wait_for_service(dur) && release_client_->wait_for_service(dur);
}

DockResult DockClient::dock(
  const std::string & site1, const std::string & site2, const std::string & group,
  double timeout_s)
{
  DockResult out;
  if (!dock_client_->service_is_ready()) {
    out.state = "unavailable";
    out.message = "dock 服务不可用";
    return out;
  }
  auto req = std::make_shared<Dock::Request>();
  req->site1 = site1;
  req->site2 = site2;
  req->group = group;
  req->timeout = static_cast<float>(timeout_s);

  auto future = dock_client_->async_send_request(req);
  // 服务是阻塞式的：本地等待给足余量（默认插件超时 6.5 s + 余量）
  const double wait_s = (timeout_s > 0.0 ? timeout_s : 6.5) + call_timeout_s_;
  if (future.wait_for(std::chrono::duration<double>(wait_s)) != std::future_status::ready) {
    out.state = "client_timeout";
    out.message = "等待 dock 响应超时";
    return out;
  }
  const auto res = future.get();
  out.success = res->success;
  out.state = res->state;
  out.message = res->message;

  const auto & rel = res->relpose;
  // relpose 是“对接成功时的实测相对位姿”，其目标值是 Q_DOCK（绕 y 180°）：
  // 所以误差应该相对 Q_DOCK 算，而不是相对单位旋转。
  out.relpose_pos_err = std::sqrt(
    rel.position.x * rel.position.x + rel.position.y * rel.position.y + rel.position.z * rel.position.z);
  const Eigen::Quaterniond q(rel.orientation.w, rel.orientation.x, rel.orientation.y, rel.orientation.z);
  const Eigen::Quaterniond q_dock(0.0, 0.0, 1.0, 0.0);
  const double dot = std::min(1.0, std::abs(q.dot(q_dock)));
  out.relpose_rot_err = 2.0 * std::acos(dot);
  return out;
}

DockResult DockClient::release(
  const std::string & site1, const std::string & site2, double timeout_s)
{
  DockResult out;
  if (!release_client_->service_is_ready()) {
    out.state = "unavailable";
    out.message = "release 服务不可用";
    return out;
  }
  auto req = std::make_shared<Release::Request>();
  req->site1 = site1;
  req->site2 = site2;
  req->timeout = static_cast<float>(timeout_s);

  auto future = release_client_->async_send_request(req);
  const double wait_s = (timeout_s > 0.0 ? timeout_s : 5.0) + call_timeout_s_;
  if (future.wait_for(std::chrono::duration<double>(wait_s)) != std::future_status::ready) {
    out.state = "client_timeout";
    out.message = "等待 release 响应超时";
    return out;
  }
  const auto res = future.get();
  out.success = res->success;
  out.state = res->state;
  out.message = res->message;
  return out;
}

}  // namespace space_sim_task_control
