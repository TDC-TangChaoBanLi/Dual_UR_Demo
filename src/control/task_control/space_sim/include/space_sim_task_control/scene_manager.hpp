// 规划场景管理：被搬运 / 已装配镜片的碰撞几何（真实网格）与 ACM。
//
// 为什么需要：
//   * URDF 里镜片是**固定在储存位**的 link，MuJoCo 里却是浮动体 —— 抓取之后 MoveIt 的
//     模型里那一片仍然留在储存位（"幽灵镜片"），继续检查它会挡住后续动作；装配之后同理。
//   * 被搬运的镜片用 **真实网格**（不是 BOX 近似）挂成 attached collision object。
//   * 挂上网格后**必须显式取消它与目标镜片（URDF 幽灵 link）的碰撞检查**，否则两者
//     完全重合、永远处于碰撞中（表现为规划失败或执行卡住）。
//   * 装配完成后要在安装位置**留下一个同样的网格**（世界对象），这样装下一片时不会撞到
//     已经装好的镜片；同时要允许"刚放下它的那条臂"的夹爪与它接触（否则收尾回 home 会失败）。
//
// ACM 一律"读-改-写"：先用 get_planning_scene 取回当前 ACM（含 SRDF 的 disable_collisions
// 与已挂对象的条目），改完再 apply_planning_scene 回写；直接发一份新 ACM 会把基线抹掉。
#pragma once

#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/mesh.hpp"

namespace space_sim_task_control
{

class SceneManager
{
public:
  bool clearTaskObjects(std::string & err);
  explicit SceneManager(const rclcpp::Node::SharedPtr & node, double timeout_s = 10.0);
  ~SceneManager();

  bool waitForServices(double timeout_s);

  /// 把所有"待搬运镜片"的真实网格作为**世界碰撞体**加入规划场景。
  /// URDF 里已经不生成这些 link（`include_movable_lenses:=false`），所以储存态的镜片
  /// 必须靠这里进场景，机械臂才会绕开它们。
  bool addLensObjects(
    const std::vector<std::pair<int, Eigen::Isometry3d>> & lens_poses,
    const shape_msgs::msg::Mesh & mesh, std::string & err);

  /// 允许 `link` 与所有其它 link / 对象碰撞（幂等）
  bool setLensCentreContact(int lens, bool allowed, std::string & err);
  bool setLensStorageContact(int lens, bool allowed, std::string & err);
  bool setLensTouch(int lens, const std::vector<std::string> & links, bool allowed,
                    std::string & err);

  bool allowLinkWithAll(const std::string & link, std::string & err);

  /// 把镜片（真实网格）作为 attached collision object 挂到 `tcp_link` 上。
  /// `touch_links` 是允许接触的 link 白名单，**必须包含该镜片在 URDF 里的 link**
  /// （即"幽灵镜片"），否则挂上去就和自己重合、一直互撞。
  bool attachLens(
    int lens, const std::string & tcp_link, const Eigen::Isometry3d & lens_in_tcp,
    const std::vector<std::string> & touch_links, const shape_msgs::msg::Mesh & mesh,
    std::string & err);

  /// 卸载 attached 镜片。**必须给出当初挂载的 link_name**：MoveIt 对 link_name 为空的
  /// REMOVE 只会把对象从 world 里删掉，attached body 仍在。
  bool detachLens(int lens, const std::string & link_name, std::string & err);

  /// 把已装配的镜片（真实网格）按实测位姿加回场景，供后续规划避让；
  /// `allow_touch_links` 里给出"刚刚放下它的那条臂"的 link（允许接触，否则收尾规划会被挡）。
  bool addPlacedLens(
    int lens, const Eigen::Isometry3d & world_pose, const shape_msgs::msg::Mesh & mesh,
    const std::vector<std::string> & allow_touch_links, std::string & err);

private:
  bool fetchAcm(moveit_msgs::msg::AllowedCollisionMatrix & out, std::string & err);
  /// 在 ACM 里确保 `name` 有一行/一列（不存在则追加空行），返回其索引
  static size_t ensureAcmEntry(
    moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name);
  /// 放行“该镜片的碰撞体 vs 它自己的储存插口 / 储存区底板”：镜片本来就插在插口里，
  /// 那是**对接接触**而不是碰撞（MuJoCo 里由物理解接触）。不放行时 move_group 的
  /// CheckStartStateCollision 会在“抓取后”直接报
  /// “6 contact(s) detected : len_dock_N - space_sim_carried_lens_N” 把整轮规划判死
  /// （实测 2026-09-27：每次规划 2~3 ms 返回 code 99999，coupled 卡在取出存储区那一段）。
  static void allowLensStorageContacts(
    moveit_msgs::msg::AllowedCollisionMatrix & acm, int lens);
  static void setAcmAllowed(
    moveit_msgs::msg::AllowedCollisionMatrix & acm, size_t i, size_t j, bool allowed);
  /// 把 ACM + 规划场景 diff 一起下发（timeout_s<=0 用默认超时）
  bool applyScene(
    const moveit_msgs::msg::PlanningScene & scene, std::string & err, double timeout_s = 0.0);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::thread client_thread_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
  double timeout_s_;
  std::vector<std::string> allowed_links_;
};

}  // namespace space_sim_task_control
