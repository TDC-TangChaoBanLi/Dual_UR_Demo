#include "space_sim_task_control/scene_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>

#include "Eigen/Geometry"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "space_sim_task_control/types.hpp"

namespace space_sim_task_control
{
namespace
{

std::mutex scene_mutation_mutex;

std::string lensObjectId(int lens)
{
  return "space_sim_carried_lens_" + std::to_string(lens);
}

/// 该镜片的储存插口 link（与 space_sim_description/xacro/len_macro.xacro 一致）
std::string storageSocketLink(int lens)
{
  return "len_dock_" + std::to_string(lens);
}

/// 储存区底板 link（镜片就夹在它与插口之间）
constexpr const char * kStoragePanelLink = "len_dock_pannel";

}  // namespace

SceneManager::SceneManager(const rclcpp::Node::SharedPtr & node, double timeout_s)
: node_(node), timeout_s_(timeout_s)
{
  static std::atomic_uint next_id{0};
  client_node_ = rclcpp::Node::make_shared(
    "space_sim_scene_client_" + std::to_string(next_id.fetch_add(1)));
  get_scene_ = client_node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  apply_scene_ = client_node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  client_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  client_executor_->add_node(client_node_);
  client_thread_ = std::thread([this]() { client_executor_->spin(); });
}

SceneManager::~SceneManager()
{
  client_executor_->cancel();
  if (client_thread_.joinable()) client_thread_.join();
  client_executor_->remove_node(client_node_);
}

bool SceneManager::waitForServices(double timeout_s)
{
  const auto dur = std::chrono::duration<double>(timeout_s);
  return get_scene_->wait_for_service(dur) && apply_scene_->wait_for_service(dur);
}

bool SceneManager::fetchAcm(moveit_msgs::msg::AllowedCollisionMatrix & out, std::string & err)
{
  if (!get_scene_->service_is_ready()) {
    err = "get_planning_scene 服务不可用";
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_scene_->async_send_request(req);
  if (future.wait_for(std::chrono::duration<double>(timeout_s_)) != std::future_status::ready) {
    err = "get_planning_scene 超时";
    return false;
  }
  out = future.get()->scene.allowed_collision_matrix;
  if (out.entry_names.empty()) {
    err = "ACM 为空（move_group 未就绪？）";
    return false;
  }
  return true;
}

size_t SceneManager::ensureAcmEntry(
  moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name)
{
  const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
  if (it != acm.entry_names.end()) {
    return static_cast<size_t>(std::distance(acm.entry_names.begin(), it));
  }
  const size_t idx = acm.entry_names.size();
  acm.entry_names.push_back(name);
  // 现有每一行都要扩展到 n+1 列
  for (auto & entry : acm.entry_values) {
    entry.enabled.resize(idx + 1, false);
  }
  // 新的一行（全 false = 默认都检查碰撞）
  moveit_msgs::msg::AllowedCollisionEntry row;
  row.enabled.assign(idx + 1, false);
  acm.entry_values.push_back(row);
  return idx;
}

void SceneManager::setAcmAllowed(
  moveit_msgs::msg::AllowedCollisionMatrix & acm, size_t i, size_t j, bool allowed)
{
  if (i >= acm.entry_values.size() || j >= acm.entry_values.size()) {
    return;
  }
  if (j < acm.entry_values[i].enabled.size()) {
    acm.entry_values[i].enabled[j] = allowed;
  }
  if (i < acm.entry_values[j].enabled.size()) {
    acm.entry_values[j].enabled[i] = allowed;
  }
}

void SceneManager::allowLensStorageContacts(
  moveit_msgs::msg::AllowedCollisionMatrix & acm, int lens)
{
  const size_t obj_idx = ensureAcmEntry(acm, lensObjectId(lens));
  for (const std::string & other : {storageSocketLink(lens), std::string(kStoragePanelLink)}) {
    const size_t idx = ensureAcmEntry(acm, other);
    setAcmAllowed(acm, obj_idx, idx, true);
  }
}

bool SceneManager::applyScene(
  const moveit_msgs::msg::PlanningScene & scene, std::string & err, double timeout_s)
{
  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene = scene;
  auto future = apply_scene_->async_send_request(req);
  const double wait_s = timeout_s > 0.0 ? timeout_s : timeout_s_;
  if (future.wait_for(std::chrono::duration<double>(wait_s)) != std::future_status::ready) {
    err = "apply_planning_scene 超时（" + std::to_string(static_cast<int>(wait_s)) + " s）";
    return false;
  }
  if (!future.get()->success) {
    err = "apply_planning_scene 返回失败";
    return false;
  }
  return true;
}

bool SceneManager::clearTaskObjects(std::string & err)
{
  std::lock_guard<std::mutex> lock(scene_mutation_mutex);
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  auto future = get_scene_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout_s_)) != std::future_status::ready) {
    err = "重置时读取碰撞矩阵/附着对象超时"; return false;
  }
  const auto previous = future.get()->scene;
  const auto & old = previous.allowed_collision_matrix;
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  std::vector<size_t> keep;
  for (size_t i = 0; i < old.entry_names.size(); ++i) {
    if (old.entry_names[i].rfind("space_sim_carried_lens_", 0) != 0) keep.push_back(i);
  }
  auto & acm = scene.allowed_collision_matrix;
  acm.default_entry_names = old.default_entry_names;
  acm.default_entry_values = old.default_entry_values;
  for (size_t i : keep) {
    acm.entry_names.push_back(old.entry_names[i]);
    moveit_msgs::msg::AllowedCollisionEntry row;
    for (size_t j : keep) row.enabled.push_back(old.entry_values[i].enabled[j]);
    acm.entry_values.push_back(row);
  }
  for (const auto & previous_object : previous.world.collision_objects) {
    if (previous_object.id.rfind("space_sim_carried_lens_", 0) != 0) continue;
    moveit_msgs::msg::CollisionObject object;
    object.id = previous_object.id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.world.collision_objects.push_back(object);
  }
  for (const auto & previous_attachment : previous.robot_state.attached_collision_objects) {
    if (previous_attachment.object.id.rfind("space_sim_carried_lens_", 0) != 0) continue;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = previous_attachment.link_name;
    attached.object.id = previous_attachment.object.id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene.robot_state.attached_collision_objects.push_back(attached);
  }
  return applyScene(scene, err);
}

bool SceneManager::addLensObjects(
  const std::vector<std::pair<int, Eigen::Isometry3d>> & lens_poses,
  const shape_msgs::msg::Mesh & mesh, std::string & err)
{
  if (!apply_scene_->service_is_ready()) {
    err = "apply_planning_scene 服务不可用";
    return false;
  }
  if (mesh.triangles.empty() || mesh.vertices.empty()) {
    err = "镜片网格为空（加载失败？）";
    return false;
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  // 逐片写入 + 重试：启动时 move_group 可能刚起来还在建场景，一次发 6 个网格容易超时
  // （实测报过 “apply_planning_scene 超时” 导致整个节点初始化失败）。
  size_t ok_count = 0;
  for (const auto & lp : lens_poses) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = lensObjectId(lp.first);
    obj.header.frame_id = kPlanningFrame;
    obj.pose = toMsg(lp.second);
    obj.meshes.push_back(mesh);
    obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene.world.collision_objects.clear();
    scene.world.collision_objects.push_back(obj);
    bool single_ok = false;
    std::string last_err;
    for (int attempt = 0; attempt < 3 && !single_ok; ++attempt) {
      single_ok = applyScene(scene, last_err, 8.0);
      if (!single_ok) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    if (single_ok) {
      ++ok_count;
    } else {
      err = "把 " + obj.id + " 加入规划场景失败（重试 3 次）: " + last_err;
      return false;
    }
  }
  if (ok_count != lens_poses.size() || ok_count != kLensCount) {
    err = "规划场景缺少镜片碰撞体";
    return false;
  }

  // 储存态的镜片同样插在插口里（对象与插口 link 重叠），先把这些固有接触在 ACM 里放行，
  // 否则“抓取后”的起点碰撞检查会被这 6 对接触判失败（详见 allowLensStorageContacts 的说明）。
  {
    moveit_msgs::msg::AllowedCollisionMatrix acm;
    std::string acm_err;
    if (fetchAcm(acm, acm_err)) {
      for (const auto & lp : lens_poses) {
        allowLensStorageContacts(acm, lp.first);
      }
      moveit_msgs::msg::PlanningScene scene;
      scene.is_diff = true;
      scene.allowed_collision_matrix = acm;
      std::string apply_err;
      if (!applyScene(scene, apply_err)) {
        RCLCPP_WARN(
          node_->get_logger(), "放行“镜片 vs 储存插口”的 ACM 条目失败（不致命）: %s",
          apply_err.c_str());
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "读取 ACM 失败（不致命）: %s", acm_err.c_str());
    }
  }

  RCLCPP_INFO(
    node_->get_logger(), "已把 %zu/%zu 片待搬运镜片作为世界碰撞体加入规划场景（网格 %zu 顶点 / %zu 面）",
    ok_count, lens_poses.size(), mesh.vertices.size(), mesh.triangles.size());
  return true;
}

bool SceneManager::setLensCentreContact(int lens, bool allowed, std::string & err)
{
  return setLensTouch(lens, {"len_center"}, allowed, err);
}

bool SceneManager::setLensTouch(
  int lens, const std::vector<std::string> & links, bool allowed, std::string & err)
{
  std::lock_guard<std::mutex> scene_lock(scene_mutation_mutex);
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  if (!fetchAcm(acm, err)) return false;
  const size_t object = ensureAcmEntry(acm, lensObjectId(lens));
  for (const auto & link : links) {
    const size_t idx = ensureAcmEntry(acm, link);
    setAcmAllowed(acm, object, idx, allowed);
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix = acm;
  return applyScene(scene, err);
}

bool SceneManager::allowLinkWithAll(const std::string & link, std::string & err)
{
  if (!get_scene_->service_is_ready() || !apply_scene_->service_is_ready()) {
    err = "规划场景服务不可用";
    return false;
  }
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  if (!fetchAcm(acm, err)) {
    return false;
  }
  const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
  if (it == acm.entry_names.end()) {
    err = "ACM 中找不到 link: " + link;
    return false;
  }
  const size_t idx = static_cast<size_t>(std::distance(acm.entry_names.begin(), it));
  for (size_t j = 0; j < acm.entry_names.size(); ++j) {
    setAcmAllowed(acm, idx, j, true);
  }
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix = acm;
  if (!applyScene(scene, err)) {
    return false;
  }
  if (std::find(allowed_links_.begin(), allowed_links_.end(), link) == allowed_links_.end()) {
    allowed_links_.push_back(link);
  }
  return true;
}

bool SceneManager::attachLens(
  int lens, const std::string & tcp_link, const Eigen::Isometry3d & lens_in_tcp,
  const std::vector<std::string> & touch_links, const shape_msgs::msg::Mesh & mesh,
  std::string & err)
{
  std::lock_guard<std::mutex> scene_lock(scene_mutation_mutex);
  if (!apply_scene_->service_is_ready()) {
    err = "apply_planning_scene 服务不可用";
    return false;
  }
  if (mesh.triangles.empty() || mesh.vertices.empty()) {
    err = "镜片网格为空（加载失败？）";
    return false;
  }
  const std::string id = lensObjectId(lens);

  // 1) 先把"该 attached 网格 vs 允许接触的 link"在 ACM 里显式放开。
  //    这里必须包含目标镜片自己在 URDF 里的 link（"幽灵镜片"），否则两者完全重合、
  //    永远处于碰撞状态 —— 表现为规划/执行卡住。
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  if (!fetchAcm(acm, err)) {
    return false;
  }
  const size_t obj_idx = ensureAcmEntry(acm, id);
  for (const auto & link : touch_links) {
    const size_t link_idx = ensureAcmEntry(acm, link);
    setAcmAllowed(acm, obj_idx, link_idx, true);
  }
  // 1b) 还要放行"该镜片 vs 它自己的储存插口/底板" —— 镜片**本来就插在插口里**（抓取后
  //     刚退出十几毫米时仍然贴着），这是**对接接触**而不是碰撞（MuJoCo 里由物理解接触）。
  //     不放行的话 move_group 的 CheckStartStateCollision 会报
  //     “6 contact(s) detected : len_dock_N - space_sim_carried_lens_N” 并把整条流水线判失败
  //     （实测 2026-09-27：表现是每次规划 2~3 ms 就返回 code 99999，coupled 卡在
  //     「沿存储面 z 轴退出」/「移出存储区」；手工在 ACM 里放行后该步立刻通过）。
  allowLensStorageContacts(acm, lens);

  // 2) 一个 diff 里同时下发 ACM 与 attached object（原子，避免中间态被判碰撞）
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = tcp_link;
  aco.touch_links = touch_links;
  auto & obj = aco.object;
  obj.id = id;
  obj.header.frame_id = tcp_link;
  obj.pose = toMsg(lens_in_tcp);
  obj.meshes.push_back(mesh);
  obj.mesh_poses.push_back(geometry_msgs::msg::Pose());  // 网格就在 body 系里
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  scene.robot_state.attached_collision_objects.push_back(aco);
  scene.allowed_collision_matrix = acm;
  if (!applyScene(scene, err)) {
    err = "attach 镜片失败: " + err;
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "已挂载 len_ring_%d 的网格（%zu 顶点 / %zu 面），并放开与 %zu 个 link 的碰撞",
    lens, mesh.vertices.size(), mesh.triangles.size(), touch_links.size());
  return true;
}

bool SceneManager::detachLens(int lens, const std::string & link_name, std::string & err)
{
  if (!apply_scene_->service_is_ready()) {
    err = "apply_planning_scene 服务不可用";
    return false;
  }
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link_name;
  aco.object.id = lensObjectId(lens);
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  scene.robot_state.attached_collision_objects.push_back(aco);
  return applyScene(scene, err);
}

bool SceneManager::addPlacedLens(
  int lens, const Eigen::Isometry3d & world_pose, const shape_msgs::msg::Mesh & mesh,
  const std::vector<std::string> & allow_touch_links, std::string & err)
{
  std::lock_guard<std::mutex> scene_lock(scene_mutation_mutex);
  if (!apply_scene_->service_is_ready()) {
    err = "apply_planning_scene 服务不可用";
    return false;
  }
  if (mesh.triangles.empty() || mesh.vertices.empty()) {
    err = "镜片网格为空（加载失败？）";
    return false;
  }
  const std::string id = lensObjectId(lens);

  // 刚刚放下它的那条臂还在旁边（夹爪可能仍与镜片几何重叠），先放开这些 link 与它的碰撞，
  // 否则收尾的"退出/回 home"规划会被判碰撞而失败。
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  if (!fetchAcm(acm, err)) {
    return false;
  }
  const size_t obj_idx = ensureAcmEntry(acm, id);
  for (const auto & link : allow_touch_links) {
    const size_t link_idx = ensureAcmEntry(acm, link);
    setAcmAllowed(acm, obj_idx, link_idx, true);
  }

  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = kPlanningFrame;
  obj.pose = toMsg(world_pose);
  obj.meshes.push_back(mesh);
  obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.world.collision_objects.push_back(obj);
  scene.allowed_collision_matrix = acm;
  if (!applyScene(scene, err)) {
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "已在安装位置留下 len_ring_%d 的网格（后续规划会避让它）", lens);
  return true;
}

}  // namespace space_sim_task_control
