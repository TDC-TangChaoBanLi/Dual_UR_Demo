#include "space_sim_task_control/site_pose_provider.hpp"

#include <utility>

namespace space_sim_task_control
{

SitePoseProvider::SitePoseProvider(const rclcpp::Node::SharedPtr & node, double timeout_s)
: node_(node), timeout_s_(timeout_s)
{
  client_ = node_->create_client<GetObjectPose>("/mujoco_god_plugin/get_object_pose");
}

bool SitePoseProvider::waitForService(double timeout_s)
{
  return client_->wait_for_service(std::chrono::duration<double>(timeout_s));
}

std::optional<Eigen::Isometry3d> SitePoseProvider::get(
  const std::string & type, const std::string & name, const std::string & frame_id)
{
  if (!client_->service_is_ready()) {
    ++failures_;
    return std::nullopt;
  }
  auto req = std::make_shared<GetObjectPose::Request>();
  req->type = type;
  req->name = name;
  req->frame_id = frame_id;
  auto future = client_->async_send_request(req);
  if (future.wait_for(std::chrono::duration<double>(timeout_s_)) != std::future_status::ready) {
    ++failures_;
    return std::nullopt;
  }
  const auto res = future.get();
  if (!res->success) {
    ++failures_;
    return std::nullopt;
  }
  // 注意：响应是 PoseStamped，不是裸 Pose
  return fromMsg(res->pose.pose);
}

std::optional<Eigen::Isometry3d> SitePoseProvider::lensBody(int lens)
{
  return get("body", "len_ring_" + std::to_string(lens));
}

std::optional<Eigen::Isometry3d> SitePoseProvider::lensFace(int lens, const std::string & face)
{
  return get("site", "len_ring_" + std::to_string(lens) + "_" + face);
}

std::optional<Eigen::Isometry3d> SitePoseProvider::centreFace(const std::string & face)
{
  return get("site", "len_center_" + face);
}

std::optional<Eigen::Isometry3d> SitePoseProvider::tcpSite(const std::string & arm)
{
  return get("site", arm + "__tcp_site");
}

std::optional<std::pair<double, double>> SitePoseProvider::mateError(
  const std::string & site1, const std::string & site2, const Eigen::Isometry3d & dock_target)
{
  const auto a = get("site", site1);
  const auto b = get("site", site2);
  if (!a || !b) {
    return std::nullopt;
  }
  const Eigen::Isometry3d rel = a->inverse() * (*b);  // site2 在 site1 系下
  const double pos_err = (rel.translation() - dock_target.translation()).norm();
  const Eigen::AngleAxisd aa(rel.rotation() * dock_target.rotation().transpose());
  return std::make_pair(pos_err, std::abs(aa.angle()));
}

}  // namespace space_sim_task_control
