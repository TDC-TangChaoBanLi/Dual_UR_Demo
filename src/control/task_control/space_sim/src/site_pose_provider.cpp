#include "space_sim_task_control/site_pose_provider.hpp"

#include <utility>

namespace space_sim_task_control
{

SitePoseProvider::SitePoseProvider(const rclcpp::Node::SharedPtr & node, double timeout_s)
: node_(node), timeout_s_(timeout_s)
{
  client_ = node_->create_client<GetObjectPose>("/mujoco_god_plugin/get_object_pose");
  cache_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

void SitePoseProvider::watch(const std::string & type, const std::string & name)
{
  const std::string key = type + "/" + name;
  rclcpp::SubscriptionOptions options;
  options.callback_group = cache_group_;
  subscriptions_.push_back(node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mujoco_god_plugin/pose/" + key, rclcpp::SensorDataQoS(),
    [this, key](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      auto & history = cache_[key];
      history.push_back({*msg, std::chrono::steady_clock::now()});
      while (history.size() > 4) history.pop_front();
    }, options));
}

std::optional<Eigen::Isometry3d> SitePoseProvider::cachedRelative(
  const std::string & object_key, const std::string & reference_key, double max_age_s)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  const auto object = cache_.find(object_key), reference = cache_.find(reference_key);
  if (object == cache_.end() || reference == cache_.end()) return std::nullopt;
  const auto now = std::chrono::steady_clock::now();
  for (auto a = object->second.rbegin(); a != object->second.rend(); ++a)
    for (auto b = reference->second.rbegin(); b != reference->second.rend(); ++b) {
      // Use one physics snapshot, never two independent moving-body samples.
      if (a->pose.header.stamp != b->pose.header.stamp ||
          a->pose.header.frame_id != b->pose.header.frame_id) continue;
      const double simulation_age = (node_->now() - rclcpp::Time(a->pose.header.stamp, RCL_ROS_TIME)).seconds();
      if (simulation_age < -0.01 || simulation_age > max_age_s ||
          std::chrono::duration<double>(now-a->received).count() > max_age_s ||
          std::chrono::duration<double>(now-b->received).count() > max_age_s) continue;
      const auto body = fromMsg(a->pose.pose), frame = fromMsg(b->pose.pose);
      if (!body.matrix().allFinite() || !frame.matrix().allFinite()) return std::nullopt;
      return frame.inverse() * body;
    }
  return std::nullopt;
}

bool SitePoseProvider::waitForService(double timeout_s)
{
  return client_->wait_for_service(std::chrono::duration<double>(timeout_s));
}

std::optional<Eigen::Isometry3d> SitePoseProvider::get(
  const std::string & type, const std::string & name, const std::string & frame_id,
  double timeout_s)
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
  if (future.wait_for(std::chrono::duration<double>(timeout_s > 0.0 ? timeout_s : timeout_s_)) != std::future_status::ready) {
    client_->remove_pending_request(future);
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
