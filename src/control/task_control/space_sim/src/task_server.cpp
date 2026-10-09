#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "space_sim_task_control/action/dock_storage.hpp"
#include "space_sim_task_control/action/exit_storage.hpp"
#include "space_sim_task_control/action/move_lens.hpp"
#include "space_sim_task_control/action/dock_centre.hpp"
#include "space_sim_task_control/srv/reset_planning_environment.hpp"
#include "mujoco_ros2_control_msgs/srv/reset_scene.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "mujoco_god_plugin/srv/get_joint_states.hpp"
#include "space_sim_task_control/arm_planner.hpp"
#include "space_sim_task_control/assembly_targets.hpp"
#include "space_sim_task_control/dock_client.hpp"
#include "space_sim_task_control/diagnostics.hpp"
#include "space_sim_task_control/mesh_loader.hpp"
#include "space_sim_task_control/motion_bridge.hpp"
#include "space_sim_task_control/moveit_params.hpp"
#include "space_sim_task_control/scene_manager.hpp"
#include "space_sim_task_control/site_pose_provider.hpp"
#include "space_sim_task_control/types.hpp"

namespace st = space_sim_task_control;
using DockStorage = st::action::DockStorage;
using ExitStorage = st::action::ExitStorage;
using MoveLens = st::action::MoveLens;
using DockCentre = st::action::DockCentre;
using ResetEnvironment = st::srv::ResetPlanningEnvironment;
using LensInitialState = st::msg::LensInitialState;
using ResetScene = mujoco_ros2_control_msgs::srv::ResetScene;
using SwitchController = controller_manager_msgs::srv::SwitchController;

namespace
{
struct ArmContext
{
  const st::ArmSpec * spec = nullptr;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;
  std::unique_ptr<st::ArmPlanner> planner;
  std::atomic<bool> busy{false};
  int lens = 0;
  std::string grasp_face;
  std::string centre_face;
  int log_lens = 0;
  std::string log_target = "目标尚未选择";
  std::string log_stage = "动作开始";
  double phase_deg = 0.0;
  Eigen::Isometry3d lens_in_tcp{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d storage_lowered_tip{Eigen::Isometry3d::Identity()};
  enum class State { empty, storage_docked, storage_released, carried, centre_docked,
                     installed_uncleared, error } state = State::empty;
};

std::vector<std::string> touchLinks(const st::ArmSpec & spec)
{
  const std::string p = spec.name + "_";
  return {spec.tcp_link, p + "gripper_base", p + "gripper_dock", p + "ur_tool0", p + "ur_flange"};
}

bool validFace(const std::string & face)
{
  return face.size() == 2 && face[0] == 'S' && face[1] >= '0' && face[1] <= '5';
}

std::string stageName(const std::string & stage)
{
  static const std::map<std::string, std::string> names{
    {"pregrasp", "取片/到预抓取位"}, {"grasp_axis", "取片/沿接口轴接近2cm"},
    {"magnetic_dock", "取片/夹爪磁吸镜片"}, {"release_storage", "取片/释放存储接口"},
    {"lower_from_storage_2cm", "取片/原姿态尽可能下降2cm"},
    {"exit_storage_vertical_mirror", "转运/镜面竖直移出存储区"},
    {"move_lens_body_pose", "转运/到指定镜片位姿"},
    {"centre_preplace", "装配/到中心接口+z外2cm"}, {"centre_insert", "装配/沿接口轴插入2cm"},
    {"centre_dock", "装配/镜片与中心接口磁吸"}, {"release_tool", "装配/释放夹爪接口"},
    {"retreat", "装配/沿抓取接口+z撤退2cm"}, {"return_home", "装配后/返回home"}
  };
  const auto it = names.find(stage);
  return it == names.end() ? stage : it->second + "(" + stage + ")";
}

struct ScopeExit
{
  std::function<void()> fn;
  ~ScopeExit() { if (fn) fn(); }
};
}  // namespace

class TaskServer
{
public:
  TaskServer()
  : node_(rclcpp::Node::make_shared("space_sim_task_server")),
    sites_(node_), targets_(&sites_, node_->declare_parameter<double>("approach_m", 0.02),
                            node_->declare_parameter<double>("storage_exit_dx", 0.6)),
    dock_(node_), scene_(node_), rng_(node_->declare_parameter<int>("seed", 7))
  {
    for (int lens = 1; lens <= 6; ++lens) sites_.watch("body", st::lensBodyName(lens));
    for (const auto & spec : st::arms()) sites_.watch("site", spec.tcp_site);
    approach_ = node_->get_parameter("approach_m").as_double();
    exit_dx_ = node_->get_parameter("storage_exit_dx").as_double();
    axis_tolerance_ = node_->declare_parameter<double>("axis_tolerance_m", 0.008);
    vertical_tolerance_ = node_->declare_parameter<double>("vertical_tolerance", 0.10);
    planning_time_ = node_->declare_parameter<double>("planning_time_s", 12.0);
    centre_planning_time_ = node_->declare_parameter<double>("centre_planning_time_s", 30.0);
    if(!std::isfinite(centre_planning_time_)||centre_planning_time_<=0||centre_planning_time_>30)
      throw std::invalid_argument("centre_planning_time_s must be in (0,30]");
    home_planning_time_ = node_->declare_parameter<double>("home_planning_time_s", 30.0);
    if(!std::isfinite(home_planning_time_)||home_planning_time_<=0||home_planning_time_>30)
      throw std::invalid_argument("home_planning_time_s must be in (0,30]");
    velocity_scale_ = node_->declare_parameter<double>("velocity_scale", 0.1);
    acceleration_scale_ = node_->declare_parameter<double>("acceleration_scale", 0.1);
    motion_.setNode(node_);
    motion_.setControllerTimeout(node_->declare_parameter<double>("controller_timeout_s", 120.0));
    init_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reset_scene_client_ = node_->create_client<ResetScene>(node_->declare_parameter<std::string>(
      "mujoco_reset_scene_service", "/mujoco_ros2_control_node/reset_scene"));
    switch_client_ = node_->create_client<SwitchController>("/controller_manager/switch_controller");
    joints_client_ = node_->create_client<mujoco_god_plugin::srv::GetJointStates>(
      "/mujoco_god_plugin/get_joint_states");
    init_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "initialize_planning_environment",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> guard(init_mutex_);
        std::string err;
        ResetEnvironment::Response reset_response;
        res->success = resetEnvironment(ResetEnvironment::Request{}, reset_response, err);
        res->message = res->success ? "MuJoCo已重置为home/全镜片在存储区，MoveIt与任务状态已同步" : err;
      }, rclcpp::ServicesQoS(), init_group_);
    reset_service_ = node_->create_service<ResetEnvironment>(
      "reset_planning_environment",
      [this](std::shared_ptr<ResetEnvironment::Request> req, std::shared_ptr<ResetEnvironment::Response> res) {
        std::lock_guard<std::mutex> guard(init_mutex_);
        std::string err;
        res->success = resetEnvironment(*req, *res, err);
        res->message = res->success ? "MuJoCo、MoveIt和任务状态已按指定初始状态重置" : err;
      }, rclcpp::ServicesQoS(), init_group_);
    storage_server_ = makeServer<DockStorage>(
      "dock_storage_lens", [this](ArmContext & ctx, const DockStorage::Goal & goal,
          DockStorage::Result & result, auto feedback) {
        return dockStorage(ctx, goal, result, feedback);
      });
    exit_server_ = makeServer<ExitStorage>(
      "exit_storage", [this](ArmContext & ctx, const ExitStorage::Goal & goal,
          ExitStorage::Result & result, auto feedback) {
        return exitStorage(ctx, goal, result, feedback);
      });
    move_server_ = makeServer<MoveLens>(
      "move_lens", [this](ArmContext & ctx, const MoveLens::Goal & goal,
          MoveLens::Result & result, auto feedback) {
        return moveLens(ctx, goal, result, feedback);
      });
    centre_server_ = makeServer<DockCentre>(
      "dock_centre", [this](ArmContext & ctx, const DockCentre::Goal & goal,
          DockCentre::Result & result, auto feedback) {
        return dockCentre(ctx, goal, result, feedback);
      });
  }

  ~TaskServer()
  {
    for (auto & kv : arms_) {
      if (kv.second->executor) kv.second->executor->cancel();
    }
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      for (auto & worker : workers_) if (worker.joinable()) worker.join();
    }
    for (auto & kv : arms_)
      if (kv.second->spin_thread.joinable()) kv.second->spin_thread.join();
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  template<class Action, class Work>
  typename rclcpp_action::Server<Action>::SharedPtr makeServer(
    const std::string & name, Work work)
  {
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    return rclcpp_action::create_server<Action>(
      node_, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename Action::Goal> goal) {
        std::unique_lock<std::mutex> guard(init_mutex_, std::try_to_lock);
        if (!guard.owns_lock()) return rclcpp_action::GoalResponse::REJECT;
        if (!initialized_) return rclcpp_action::GoalResponse::REJECT;
        const auto it = arms_.find(goal->arm);
        if (it == arms_.end() || it->second->busy.exchange(true))
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::REJECT; },
      [this, work, name](const std::shared_ptr<Handle> handle) {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_.emplace_back([this, handle, work, name]() {
          ArmContext & ctx = *arms_.at(handle->get_goal()->arm);
          ctx.log_lens = ctx.lens;
          ctx.log_target = "目标尚未选择";
          ctx.log_stage = name;
          if constexpr (std::is_same_v<Action, DockStorage>) {
            ctx.log_lens = handle->get_goal()->lens;
            ctx.log_target = "请求抓取接口=" + handle->get_goal()->grasp_face + "（空值按候选选择）";
          } else if constexpr (std::is_same_v<Action, DockCentre>) {
            ctx.log_target = "请求中心接口=" + handle->get_goal()->centre_face + "（空值按候选选择）";
          }
          RCLCPP_INFO(node_->get_logger(), "[动作开始] %s action=%s", context(ctx).c_str(), name.c_str());
          auto result = std::make_shared<typename Action::Result>();
          auto feedback = [this, handle, &ctx](const std::string & stage) {
            ctx.log_stage = stage;
            ctx.planner->setDiagnosticContext(context(ctx));
            RCLCPP_INFO(node_->get_logger(), "[阶段] %s", context(ctx).c_str());
            auto msg = std::make_shared<typename Action::Feedback>();
            msg->stage = stage;
            handle->publish_feedback(msg);
          };
          bool ok = false;
          try { ok = work(ctx, *handle->get_goal(), *result, feedback); }
          catch (const std::exception & e) { result->message = e.what(); }
          if (ok) RCLCPP_INFO(node_->get_logger(), "[动作完成] %s action=%s 结果=%s",
            context(ctx).c_str(), name.c_str(), result->message.c_str());
          else RCLCPP_ERROR(node_->get_logger(), "[动作失败] %s action=%s 原因=%s；本次 action 停止",
            context(ctx).c_str(), name.c_str(), result->message.c_str());
          if (ok) handle->succeed(result); else handle->abort(result);
          ctx.busy = false;
        });
      });
  }

  bool initialize(std::string & err)
  {
    if (infrastructure_ready_) return true;
    if (!sites_.waitForService(30.0) || !dock_.waitForServices(30.0) ||
        !scene_.waitForServices(30.0) || !targets_.verifyFrameConsistency(err)) {
      if (err.empty()) err = "MuJoCo/MoveIt 接口未就绪";
      return false;
    }
    const std::string file = st::resolvePackageFile("space_sim_description", "meshes/len_ring1.stl");
    if (file.empty() || !st::loadMeshFile(file, mesh_, err)) return false;
    for (const auto & spec : st::arms()) {
      if (arms_.count(spec.name) != 0) continue;
      auto ctx = std::make_unique<ArmContext>();
      ctx->spec = &spec;
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
      ctx->node = rclcpp::Node::make_shared(spec.name + "_task_planner", options);
      if (!st::copyMoveGroupParameters(ctx->node)) {
        err = spec.name + " MoveIt 参数读取失败"; return false;
      }
      ctx->planner = std::make_unique<st::ArmPlanner>(ctx->node, spec.group, spec.tcp_link);
      ctx->planner->configure(planning_time_, 3, velocity_scale_, acceleration_scale_);
      ctx->planner->setPlanners({{"stomp", "space_sim_stomp/StompPlanner"}});
      ctx->planner->setExecuteTimeout(120.0);
      ctx->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      ctx->executor->add_node(ctx->node);
      ctx->spin_thread = std::thread([executor = ctx->executor]() { executor->spin(); });
      motion_.setPlanner(spec.name, ctx->planner.get());
      arms_[spec.name] = std::move(ctx);
    }
    infrastructure_ready_ = true;
    return true;
  }

  bool switchTrajectoryControllers(bool activate, std::string & err)
  {
    auto request = std::make_shared<SwitchController::Request>();
    for (const auto & spec : st::arms()) {
      const auto name = spec.name + "_ur_joint_trajectory_controller";
      (activate ? request->activate_controllers : request->deactivate_controllers).push_back(name);
    }
    request->strictness = SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec = 10;
    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready || !future.get()->ok) {
      err = activate ? "重置后轨迹控制器启动失败" : "重置前轨迹控制器停止失败";
      return false;
    }
    return true;
  }

  bool resetEnvironment(const ResetEnvironment::Request & input, ResetEnvironment::Response & output,
                        std::string & err)
  {
    for (const auto & kv : arms_) {
      if (kv.second->busy) { err = "机械臂动作仍在运行，拒绝重置: " + kv.first; return false; }
    }
    if (!initialize(err)) return false;
    if (!reset_scene_client_->wait_for_service(std::chrono::seconds(10)) ||
        !switch_client_->wait_for_service(std::chrono::seconds(10)) ||
        !joints_client_->wait_for_service(std::chrono::seconds(10))) {
      err = "MuJoCo reset_scene/关节状态或控制器切换服务未就绪，请重启更新后的仿真";
      return false;
    }
    std::map<std::string, double> joints;
    for (const auto & spec : st::arms()) {
      const auto home = arms_.at(spec.name)->planner->namedStateValues(spec.name + "_home");
      for (const auto & name : spec.joints) {
        if (!home.count(name)) { err = "SRDF home缺少关节: " + name; return false; }
        joints[name] = home.at(name);
      }
    }
    const auto & supplied = input.joint_states;
    if (supplied.name.size() != supplied.position.size() || !supplied.velocity.empty() || !supplied.effort.empty()) {
      err = "joint_states只接受长度一致的name/position，velocity/effort必须为空"; return false;
    }
    std::set<std::string> seen_joints;
    for (size_t i = 0; i < supplied.name.size(); ++i) {
      if (!joints.count(supplied.name[i]) || !std::isfinite(supplied.position[i]) ||
          !seen_joints.insert(supplied.name[i]).second) {
        err = "未知、重复或非有限关节: " + supplied.name[i]; return false;
      }
      joints[supplied.name[i]] = supplied.position[i];
    }
    auto robot = arms_.at("arm_A")->planner->mg().getCurrentState(2.0);
    if (!robot) { err = "无法读取机器人模型状态"; return false; }
    for (const auto & kv : joints) robot->setVariablePosition(kv.first, kv.second);
    if (!robot->satisfiesBounds()) { err = "指定关节角超出模型限位"; return false; }
    std::map<int, LensInitialState> states;
    for (int lens = 1; lens <= st::kLensCount; ++lens) {
      LensInitialState state; state.lens = lens; state.location = LensInitialState::STORAGE;
      state.lens_face = "S0";
      states[lens] = state;
    }
    std::set<int> seen_lenses;
    std::set<std::string> occupied_faces, occupied_arms;
    for (auto state : input.lenses) {
      if (state.lens < 1 || state.lens > st::kLensCount || !seen_lenses.insert(state.lens).second ||
          !std::isfinite(state.phase_deg)) { err = "镜片编号重复/无效或相位非有限"; return false; }
      if (state.location == LensInitialState::CENTRE) {
        if (state.lens_face.empty()) state.lens_face = "S0";
        if (!validFace(state.centre_face) || !validFace(state.lens_face) || !state.arm.empty() ||
            !occupied_faces.insert(state.centre_face).second) {
          err = "中心接口重复/无效，或CENTRE状态指定了arm"; return false;
        }
      } else if (state.location == LensInitialState::ARM) {
        if (!arms_.count(state.arm) || !validFace(state.lens_face) || !state.centre_face.empty() ||
            !occupied_arms.insert(state.arm).second) {
          err = "机械臂重复/无效、抓取接口无效，或ARM状态指定了centre_face"; return false;
        }
      } else {
        if (state.location != LensInitialState::STORAGE || !state.arm.empty() || !state.centre_face.empty() ||
            (!state.lens_face.empty() && state.lens_face != "S0") || state.phase_deg != 0.0) {
          err = "镜片location无效，或存储状态指定了不兼容的接口"; return false;
        }
        state.lens_face = "S0";
      }
      states[state.lens] = state;
    }
    auto reset = std::make_shared<ResetScene::Request>();
    for (const auto & spec : st::arms()) for (const auto & name : spec.joints) {
      reset->state_overrides.joint_states.name.push_back(name);
      reset->state_overrides.joint_states.position.push_back(joints.at(name));
      reset->state_overrides.joint_states.velocity.push_back(0.0);
    }
    auto weld = [&](const std::string & name, bool active, const std::string & a = "", const std::string & b = "") {
      mujoco_ros2_control_msgs::msg::InitialWeld binding;
      binding.name = name; binding.active = active; binding.site1 = a; binding.site2 = b;
      reset->welds.push_back(binding);
    };
    for (const auto & kv : states) {
      const auto & state = kv.second;
      // Storage welds retain the restored MJCF binding and relative pose.
      if (state.location != LensInitialState::STORAGE) weld("dock_weld_" + std::to_string(state.lens), false);
      weld("center_weld_" + std::to_string(state.lens), state.location == LensInitialState::CENTRE,
           st::lensFaceSite(state.lens, state.lens_face), st::centreSite(state.centre_face));
      if (state.location == LensInitialState::STORAGE) continue;
      const auto local = sites_.get("site", st::lensFaceSite(state.lens, state.lens_face), st::lensBodyName(state.lens));
      if (!local) { err = "镜片接口局部位姿读取失败"; return false; }
      mujoco_ros2_control_msgs::msg::FreeJointState body;
      body.name = st::lensBodyName(state.lens);
      Eigen::Isometry3d pose;
      const Eigen::Isometry3d phase = st::rotation(st::axisAngleZ(-state.phase_deg * M_PI / 180.0));
      if (state.location == LensInitialState::CENTRE) {
        const auto centre = sites_.get("site", st::centreSite(state.centre_face), "len_center");
        if (!centre) { err = "中心接口局部位姿读取失败"; return false; }
        body.pose.header.frame_id = "len_center";
        pose = *centre * st::rotation(st::kQDock) * phase * local->inverse();
      } else {
        const auto & spec = *arms_.at(state.arm)->spec;
        const auto tcp = sites_.get("site", spec.tcp_site, spec.tcp_link);
        if (!tcp) { err = "TCP局部位姿读取失败"; return false; }
        body.pose.header.frame_id = spec.tcp_link;
        pose = *tcp * (st::rotation(st::kQDock) * phase).inverse() * local->inverse();
      }
      body.pose.pose = st::toMsg(pose);
      reset->state_overrides.free_joints.push_back(body);
    }
    for (const auto & spec : st::arms()) {
      bool held = false;
      for (const auto & kv : states) if (kv.second.location == LensInitialState::ARM && kv.second.arm == spec.name) {
        weld(spec.name == "arm_A" ? "tcp_weld_A" : "tcp_weld_B", true,
          spec.tcp_site, st::lensFaceSite(kv.first, kv.second.lens_face));
        held = true;
      }
      if (!held) weld(spec.name == "arm_A" ? "tcp_weld_A" : "tcp_weld_B", false);
    }
    RCLCPP_INFO(node_->get_logger(), "[环境重置] 请求校验通过；重置MuJoCo、控制器、MoveIt与任务记录");
    if (!motion_.resetExecutor(false, err)) return false;
    initialized_ = false;
    if (!switchTrajectoryControllers(false, err)) return false;
    auto response = reset_scene_client_->async_send_request(reset);
    if (response.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
      err = "MuJoCo场景复位超时；任务接口保持禁用，请重试重置";
      return false;
    }
    const auto reset_response = response.get();
    if (!reset_response->success) {
      err = "MuJoCo场景复位失败: " + reset_response->message + "；任务接口保持禁用，请重试重置";
      return false;
    }
    if (!switchTrajectoryControllers(true, err)) return false;
    for (auto & kv : arms_) {
      auto & ctx = *kv.second;
      ctx.lens = 0; ctx.grasp_face.clear(); ctx.centre_face.clear(); ctx.phase_deg = 0.0;
      ctx.lens_in_tcp.setIdentity(); ctx.storage_lowered_tip.setIdentity();
      ctx.state = ArmContext::State::empty; ctx.log_lens = 0;
      ctx.planner->resetStartStateToCurrent();
    }
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      used_.clear(); used_faces_.clear(); reserved_.clear(); reserved_faces_.clear();
      rng_.seed(node_->get_parameter("seed").as_int());
    }
    std::vector<std::pair<int, Eigen::Isometry3d>> poses;
    for (const auto & kv : states) {
      const auto body = sites_.lensBody(kv.first);
      if (!body) { err = "重置后镜片位姿读取失败"; return false; }
      poses.emplace_back(kv.first, *body);
    }
    if (!scene_.clearTaskObjects(err) || !scene_.addLensObjects(poses, mesh_, err)) return false;
    for (const auto & kv : states) {
      const auto & state = kv.second;
      if (state.location == LensInitialState::CENTRE) {
        used_.insert(state.lens); used_faces_.insert(state.centre_face);
        if (!scene_.setLensCentreContact(state.lens, true, err)) return false;
      } else if (state.location == LensInitialState::ARM) {
        auto & ctx = *arms_.at(state.arm);
        const auto body = sites_.lensBody(state.lens);
        const auto tcp = sites_.get("body", ctx.spec->tcp_link);
        if (!body || !tcp) { err = "重置后抓取位姿读取失败"; return false; }
        ctx.lens = state.lens; ctx.grasp_face = state.lens_face; ctx.phase_deg = state.phase_deg;
        ctx.lens_in_tcp = tcp->inverse() * *body; ctx.state = ArmContext::State::carried;
        reserved_.insert(state.lens);
        if (!scene_.attachLens(state.lens, ctx.spec->tcp_link, ctx.lens_in_tcp, touchLinks(*ctx.spec), mesh_, err)) return false;
      }
      output.lenses.push_back(state);
      RCLCPP_INFO(node_->get_logger(), "[初始化镜片] len_ring_%d location=%u 中心接口=%s arm=%s 镜片接口=%s",
        state.lens, state.location, state.centre_face.c_str(), state.arm.c_str(), state.lens_face.c_str());
    }
    auto read = std::make_shared<mujoco_god_plugin::srv::GetJointStates::Request>();
    read->names = reset->state_overrides.joint_states.name;
    auto actual = joints_client_->async_send_request(read);
    if (actual.wait_for(std::chrono::seconds(5)) != std::future_status::ready) { err = "重置后关节状态验证超时"; return false; }
    const auto snapshot = actual.get();
    if (!snapshot->success) { err = snapshot->message; return false; }
    output.joint_states = snapshot->state;
    for (size_t i = 0; i < snapshot->state.name.size(); ++i) {
      if (std::abs(snapshot->state.position[i] - joints.at(snapshot->state.name[i])) > 0.02) {
        err = "重置后关节角与目标不一致: " + snapshot->state.name[i] +
          " 目标=" + std::to_string(joints.at(snapshot->state.name[i])) +
          " 实测=" + std::to_string(snapshot->state.position[i]) +
          " 偏差=" + std::to_string(snapshot->state.position[i] - joints.at(snapshot->state.name[i])) +
          " rad；检查指定初始状态的物理接触，任务接口保持禁用，请重新重置";
        return false;
      }
    }
    for (const auto & kv : states) {
      const auto & state = kv.second;
      if (state.location == LensInitialState::STORAGE) continue;
      const std::string target_site = state.location == LensInitialState::CENTRE ?
        st::centreSite(state.centre_face) : arms_.at(state.arm)->spec->tcp_site;
      const auto target = st::rotation(st::kQDock) *
        st::rotation(st::axisAngleZ(-state.phase_deg * M_PI / 180.0));
      const auto error = sites_.mateError(st::lensFaceSite(state.lens, state.lens_face), target_site, target);
      if (!error || error->first > 0.006 || error->second > 0.06) {
        err = "重置后接口对合验证失败: " + st::lensFaceSite(state.lens, state.lens_face) +
          " -> " + target_site + (error ? " 位置误差=" + std::to_string(error->first) +
          " m 姿态误差=" + std::to_string(error->second) + " rad" : " 位姿读取失败") +
          "；任务接口保持禁用，请重新重置";
        return false;
      }
    }
    if (!motion_.resetExecutor(true, err)) return false;
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[环境重置完成] MuJoCo与MoveIt状态已同步");
    return true;
  }

  std::string context(const ArmContext & ctx) const
  {
    return "arm=" + ctx.spec->name + " 镜片=" +
      (ctx.log_lens > 0 ? "len_ring_" + std::to_string(ctx.log_lens) : "未选择/未持有") +
      " 阶段=" + stageName(ctx.log_stage) + " 目标=" + ctx.log_target;
  }

  std::string poseText(const Eigen::Isometry3d & pose) const
  {
    return st::poseDescription(pose);
  }

  bool executeMotion(ArmContext & ctx, const st::MotionRequest & req, std::string & err)
  {
    if (ctx.state == ArmContext::State::error) {
      err = "机械臂任务已进入错误状态，禁止继续运动: " + context(ctx);
      return false;
    }
    ctx.log_stage = req.what;
    ctx.planner->setDiagnosticContext(context(ctx));
    const char * planner = req.kind == st::MotionRequest::Kind::kCartesian ?
      "computeCartesianPath（笛卡尔直线插值，无 STOMP 回退）" :
      "stomp/space_sim_stomp/StompPlanner";
    RCLCPP_INFO(node_->get_logger(), "[运动开始] %s 规划方式=%s",
      context(ctx).c_str(), planner);
    auto guarded = req;
    if (ctx.state == ArmContext::State::storage_docked ||
        ctx.state == ArmContext::State::storage_released ||
        ctx.state == ArmContext::State::carried) {
      guarded.execution_guard = [this, &ctx](std::string & reason) {
        const auto actual = sites_.cachedRelative("body/" + st::lensBodyName(ctx.lens),
          "site/" + ctx.spec->tcp_site, 0.5);
        const double position = actual ?
          (actual->translation() - ctx.lens_in_tcp.translation()).norm() : INFINITY;
        const double angle = actual ? std::abs(Eigen::AngleAxisd(
          actual->rotation() * ctx.lens_in_tcp.rotation().transpose()).angle()) : INFINITY;
        if (position <= 0.003 && angle <= 0.03) return true;
        reason = std::string(actual ? "持片检查失败/疑似脱落: " : "持片位姿快照不可用或过期: ") + context(ctx) +
          " TCP相对位置误差(m)=" + std::to_string(position) +
          " 姿态误差(rad)=" + std::to_string(angle);
        ctx.state = ArmContext::State::error;
        // The attached-object prediction is no longer trustworthy for peers.
        // Stop both arms without resetting physics or moving the payload.
        motion_.stopExecution();
        return false;
      };
    }
    if (guarded.execution_guard && !guarded.execution_guard(err)) throw std::runtime_error(err);
    const bool ok = motion_.move(ctx.spec->name, guarded, err);
    if (ok) RCLCPP_INFO(node_->get_logger(), "[运动完成] %s", context(ctx).c_str());
    else RCLCPP_WARN(node_->get_logger(), "[运动失败] %s 原因=%s（后续候选/回退另行记录）",
      context(ctx).c_str(), err.c_str());
    // Fatal payload faults must leave all candidate loops immediately. Preserve
    // the measured cause in the action result instead of overwriting it with
    // subsequent "already in error" attempts.
    if (!ok && ctx.state == ArmContext::State::error) throw std::runtime_error(err);
    return ok;
  }

  bool move(ArmContext & ctx, const Eigen::Isometry3d & target, const std::string & stage,
            std::string & err, double axis = 0.0, bool vertical = false,
            bool cartesian = false, bool optimizer_only = false, double planning_time_s = 0.0)
  {
    st::MotionRequest req;
    req.kind = cartesian ? st::MotionRequest::Kind::kCartesian : st::MotionRequest::Kind::kPose;
    req.target = target;
    req.what = stage;
    req.axis_tolerance_m = axis;
    req.mirror_vertical = vertical;
    req.optimizer_only = optimizer_only;
    req.planning_time_s = planning_time_s;
    req.lens_in_tcp = ctx.lens_in_tcp;
    req.vertical_tolerance = vertical_tolerance_;
    return executeMotion(ctx, req, err);
  }

  template<class Feedback>
  bool completeStorageDock(ArmContext & ctx, DockStorage::Result & result, Feedback feedback)
  {
    result.lens = ctx.lens;
    result.grasp_face = ctx.grasp_face;
    result.phase_deg = ctx.phase_deg;
    std::string err;
    if (ctx.state == ArmContext::State::storage_docked) {
      ctx.log_target = st::lensFaceSite(ctx.lens, "S0") + " -> " + st::lensSocketSite(ctx.lens);
      const auto socket = sites_.get("site", st::lensSocketSite(ctx.lens));
      if (socket) RCLCPP_DEBUG(node_->get_logger(), "[接口位姿] %s %s", context(ctx).c_str(), poseText(*socket).c_str());
      feedback("release_storage");
      const auto released = dock_.release(st::lensFaceSite(ctx.lens, "S0"),
                                          st::lensSocketSite(ctx.lens));
      if (!released.success) {
        result.message = "储存器接口释放失败: " + released.message;
        return false;
      }
      ctx.state = ArmContext::State::error;
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        if (!scene_.attachLens(ctx.lens, ctx.spec->tcp_link, ctx.lens_in_tcp,
                               touchLinks(*ctx.spec), mesh_, err)) {
          result.message = "储存接口已释放，但 MoveIt 镜片附着失败: " + err;
          return false;
        }
      }
      ctx.storage_lowered_tip = ctx.planner->currentTipPose();
      ctx.storage_lowered_tip.translation().z() -= approach_;
      ctx.state = ArmContext::State::storage_released;
    }
    ctx.log_target = st::lensSocketSite(ctx.lens);
    feedback("lower_from_storage_2cm");
    const auto before = sites_.lensBody(ctx.lens);
    if (!before) {
      result.message = "储存接口已释放，但无法读取镜片下降前位姿";
      return false;
    }
    st::MotionRequest descent;
    descent.kind = st::MotionRequest::Kind::kCartesian;
    descent.target = ctx.storage_lowered_tip;
    descent.what = "储存接口释放后保持原姿态尽可能下降 2 cm";
    descent.allow_partial_cartesian = true;
    descent.cartesian_step_m = 0.001;
    descent.axis_tolerance_m = axis_tolerance_;
    descent.mirror_vertical = true;
    descent.lens_in_tcp = ctx.lens_in_tcp;
    descent.vertical_tolerance = vertical_tolerance_;
    if (!executeMotion(ctx, descent, err)) {
      result.message = "储存接口已释放，但下降直线执行失败: " + err;
      return false;
    }
    const auto after = sites_.lensBody(ctx.lens);
    const double remaining = after ? after->translation().z() -
      (ctx.storage_lowered_tip * ctx.lens_in_tcp).translation().z() : 0.0;
    if (!after || remaining < -0.005 || remaining > approach_ + 0.005 ||
        (after->translation() - before->translation()).head<2>().norm() > 0.005 ||
        Eigen::AngleAxisd(before->rotation().transpose() * after->rotation()).angle() > 0.05) {
      result.message = "下降后实时镜片位姿不符合原姿态竖直下降要求";
      return false;
    }
    ctx.state = ArmContext::State::carried;
    result.success = true;
    const double lowered = before->translation().z() - after->translation().z();
    RCLCPP_INFO(node_->get_logger(), "[下降结果] %s 实际下降=%.2f mm，距下降目标还差=%.2f mm；下一步=ExitStorage移出",
      context(ctx).c_str(), lowered * 1000.0, std::max(0.0, remaining) * 1000.0);
    result.message = "镜片已吸附、储存器接口已释放，保持原姿态实际下降 " +
      std::to_string(lowered * 1000.0) + " mm；可调用 exit_storage";
    return true;
  }

  template<class Feedback>
  bool dockStorage(ArmContext & ctx, const DockStorage::Goal & goal,
                   DockStorage::Result & result, Feedback feedback)
  {
    if (ctx.state == ArmContext::State::storage_docked ||
        ctx.state == ArmContext::State::storage_released) {
      if (goal.lens > 0 && goal.lens != ctx.lens) {
        result.message = "本臂正在完成镜片 " + std::to_string(ctx.lens) + " 的储存接口释放";
        return false;
      }
      return completeStorageDock(ctx, result, feedback);
    }
    if (ctx.state != ArmContext::State::empty) {
      result.message = "本臂已有镜片，不能再次抓取"; return false;
    }
    if (goal.lens > 0) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (used_.count(goal.lens)) {
        result.message = "镜片 " + std::to_string(goal.lens) + " 已被取用";
        return false;
      }
      if (reserved_.count(goal.lens)) {
        result.message = "镜片 " + std::to_string(goal.lens) + " 正被另一机械臂处理";
        return false;
      }
    }
    std::vector<int> lenses;
    if (goal.lens > 0) lenses = {goal.lens};
    else if (!goal.allowed_lenses.empty()) lenses = goal.allowed_lenses;
    else lenses = {1, 2, 3, 4, 5, 6};
    if (goal.randomize) {
      std::lock_guard<std::mutex> lock(rng_mutex_);
      std::shuffle(lenses.begin(), lenses.end(), rng_);
    }
    std::string last = "没有可用的镜片/抓取接口";
    for (int lens : lenses) {
      if (lens < 1 || lens > 6) continue;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (used_.count(lens) || reserved_.count(lens)) continue;
        reserved_.insert(lens);
      }
      std::vector<std::string> faces;
      if (!targets_.graspFaces(ctx.spec->name, lens, faces, last)) {
        std::lock_guard<std::mutex> lock(state_mutex_); reserved_.erase(lens); continue;
      }
      faces.erase(std::remove_if(faces.begin(), faces.end(), [&](const std::string & face) {
        return (!goal.grasp_face.empty() && face != goal.grasp_face) ||
               (!goal.allowed_faces.empty() &&
                std::find(goal.allowed_faces.begin(), goal.allowed_faces.end(), face) ==
                goal.allowed_faces.end());
      }), faces.end());
      if (goal.randomize) {
        std::lock_guard<std::mutex> lock(rng_mutex_);
        std::shuffle(faces.begin(), faces.end(), rng_);
      }
      for (const auto & face : faces) {
        std::vector<double> phases = goal.allowed_phases_deg.empty() ?
          std::vector<double>{goal.phase_deg} : goal.allowed_phases_deg;
        if (goal.randomize) {
          std::lock_guard<std::mutex> lock(rng_mutex_);
          std::shuffle(phases.begin(), phases.end(), rng_);
        }
        for (double phase : phases) {
          ctx.log_lens = lens;
          ctx.log_stage = "取片候选/预抓取IK";
          ctx.log_target = st::lensFaceSite(lens, face);
          const auto interface_pose = sites_.lensFace(lens, face);
          if (interface_pose) RCLCPP_DEBUG(node_->get_logger(), "[抓取候选] %s phase_deg=%.17g 接口=%s",
            context(ctx).c_str(), phase, poseText(*interface_pose).c_str());
          ctx.planner->setDiagnosticContext(context(ctx));
          st::AssemblyTask task;
          if (!targets_.build(ctx.spec->name, lens, face, phase, "S0", task, last) ||
              !ctx.planner->hasIk(task.pregrasp, last)) {
            RCLCPP_WARN(node_->get_logger(), "[候选失败] %s 原因=%s；下一步=其他允许的相位/抓取接口/镜片",
              context(ctx).c_str(), last.c_str());
            continue;
          }
          feedback("pregrasp");
          if (motion_.hybridEnabled()) ctx.planner->checkFollowingApproach(task.grasp, lens, .005);
          const bool pregrasp_ok = move(ctx, task.pregrasp, "接近储存镜片", last);
          ctx.planner->clearFollowingApproach();
          if (!pregrasp_ok) {
            RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=其他允许的抓取候选",
              context(ctx).c_str(), last.c_str());
            continue;
          }
          feedback("grasp_axis");
          // The mating dock is an intentional contact surface. Authorize only
          // this arm's dock against its reserved lens for the contact segment;
          // the other arm, wrist and neighbouring lenses remain checked.
          const std::vector<std::string> contact_links{ctx.spec->name + "_gripper_dock"};
          if (!scene_.setLensTouch(lens, contact_links, true, last)) {
            result.message = last;
            return false;
          }
          if (!move(ctx, task.grasp, "沿接口轴线对合", last, axis_tolerance_, false, true)) {
            std::string cleanup;
            if (!scene_.setLensTouch(lens, contact_links, false, cleanup)) {
              last += "; 恢复接口碰撞检查失败: " + cleanup;
              result.message = last;
              return false;
            }
            RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=其他允许的抓取候选，无 STOMP 直线回退",
              context(ctx).c_str(), last.c_str());
            continue;
          }
          feedback("magnetic_dock");
          const auto docked = dock_.dock(ctx.spec->tcp_site, st::lensFaceSite(lens, face), "grasp");
          if (!docked.success) {
            last = docked.message;
            std::string cleanup;
            if (!scene_.setLensTouch(lens, contact_links, false, cleanup)) {
              last += "; 恢复接口碰撞检查失败: " + cleanup;
              result.message = last;
              return false;
            }
            RCLCPP_WARN(node_->get_logger(), "[磁吸失败] %s 原因=%s；下一步=其他允许的抓取候选",
              context(ctx).c_str(), last.c_str());
            continue;
          }
          ctx.lens = lens;
          ctx.grasp_face = face;
          ctx.phase_deg = phase;
          ctx.lens_in_tcp = task.lens_in_tcp;
          ctx.state = ArmContext::State::storage_docked;
          return completeStorageDock(ctx, result, feedback);
        }
      }
      std::lock_guard<std::mutex> lock(state_mutex_); reserved_.erase(lens);
    }
    result.message = last;
    return false;
  }

  template<class Feedback>
  bool exitStorage(ArmContext & ctx, const ExitStorage::Goal & goal,
                   ExitStorage::Result & result, Feedback feedback)
  {
    result.lens = ctx.lens;
    if (ctx.state == ArmContext::State::storage_released ||
        ctx.state == ArmContext::State::storage_docked) {
      result.message = "请先重试 dock_storage，完成储存接口释放和可达范围内的直线下降";
      return false;
    }
    if (ctx.state != ArmContext::State::carried) {
      result.message = "本臂未持有已取出的储存镜片"; return false;
    }
    std::string err;
    auto finish_exit = [&](const std::string & message) {
      if (!scene_.setLensStorageContact(ctx.lens, false, err)) {
        result.message = "镜片已移出，但恢复存储区碰撞检查失败: " + err;
        return false;
      }
      result.success = true;
      result.message = message;
      return true;
    };
    ctx.log_target = "存储区外侧悬停位；当前抓取接口=" + st::lensFaceSite(ctx.lens, ctx.grasp_face);
    // 移出路径只限制镜面法向的世界 z 分量；允许镜片绕其镜面法向旋转。
    const double sign = ctx.spec->base_position.x() >= 0 ? 1.0 : -1.0;
    const Eigen::Isometry3d start = ctx.planner->currentTipPose();
    const double final_x = start.translation().x() +
      sign * (goal.exit_distance > 0 ? goal.exit_distance : exit_dx_);
    feedback("exit_storage_vertical_mirror");
    // 首选绕镜面法向转腕：这种旋转不改变镜面是否竖直，同时释放
    // 原先固定 TCP 姿态造成的一个姿态自由度。
    std::string last_failure;
    int failed_attempts = 0;
    auto record_failure = [&]() { last_failure = err; ++failed_attempts; };
    auto try_final = [&](const Eigen::Isometry3d & origin) {
      std::vector<Eigen::Matrix3d> orientations{origin.rotation()};
      const Eigen::Vector3d normal =
        (origin.rotation() * ctx.lens_in_tcp.rotation()).col(2).normalized();
      for (double deg : {45.0, -45.0, 90.0, -90.0, 180.0}) {
        orientations.push_back(
          Eigen::AngleAxisd(deg * M_PI / 180.0, normal).toRotationMatrix() *
          origin.rotation());
      }
      Eigen::Matrix3d down = st::shortestRotationToDown(origin).rotation();
      const Eigen::Vector3d mirror_normal = (down * ctx.lens_in_tcp.rotation()).col(2);
      const Eigen::Vector3d horizontal(mirror_normal.x(), mirror_normal.y(), 0.0);
      if (horizontal.norm() > 1e-8) {
        const Eigen::Quaterniond correction = Eigen::Quaterniond::FromTwoVectors(
          mirror_normal, horizontal.normalized());
        down = correction.toRotationMatrix() * down;
        orientations.push_back(down);
      }
      for (double deg : {30.0, -30.0, 60.0, -60.0}) {
        orientations.push_back(
          Eigen::AngleAxisd(deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          origin.rotation());
      }
      for (const auto & orientation : orientations) {
        Eigen::Isometry3d outside = ctx.planner->currentTipPose();
        outside.translation().x() = final_x;
        outside.linear() = orientation;
        if (std::isfinite(goal.hover_z) && goal.hover_z != 0.0) {
          // 显式悬停高度属于镜片 body 原点；TCP 与镜片并不重合。
          outside.translation().z() = goal.hover_z -
            (orientation * ctx.lens_in_tcp.translation()).z();
        }
        if (move(ctx, outside, "STOMP 镜面竖直移出储存区", err,
                 0.0, true, false, true)) return true;
        RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=下一移出终点姿态，仍使用 STOMP",
          context(ctx).c_str(), err.c_str());
        record_failure();
      }
      return false;
    };
    if (try_final(start)) {
      return finish_exit("镜片已移出储存区");
    }
    // 邻近储存格中的镜片可能挡住向 X 方向的直达路径。先沿储存格
    // 排列方向尝试一小段避让，再重新规划向外移出；每段仍要求镜面竖直。
    // 只在真实规划和碰撞检查通过后执行，不改变任何镜片的仿真位姿。
    for (double dy : {-0.06, 0.06, -0.10, 0.10}) {
      RCLCPP_INFO(node_->get_logger(), "[回退] %s 直接移出候选均失败；方案=STOMP侧向避让 dy=%.3f m 后重试",
        context(ctx).c_str(), dy);
      Eigen::Isometry3d aisle = ctx.planner->currentTipPose();
      aisle.translation().y() = start.translation().y() + dy;
      if (!move(ctx, aisle, "STOMP 镜面竖直侧向避让", err,
                0.0, true, false, true)) {
        record_failure();
        continue;
      }
      if (try_final(ctx.planner->currentTipPose())) {
        return finish_exit("镜片经空储存格移出");
      }
    }
    // 直达 0.6 m 时受腕部关节限位；先以原镜面姿态向外走一段，
    // 在远离储存格的位置重新求腕部姿态。每段仍检查镜面竖直和碰撞。
    for (double fraction : {0.40, 0.65}) {
      RCLCPP_INFO(node_->get_logger(), "[回退] %s 方案=STOMP分段移出，先到总距离的 %.0f%% 后重试",
        context(ctx).c_str(), fraction * 100.0);
      Eigen::Isometry3d intermediate = ctx.planner->currentTipPose();
      intermediate.translation().x() = start.translation().x() +
        fraction * (final_x - start.translation().x());
      if (!move(ctx, intermediate, "STOMP 镜面竖直分段移出", err,
                0.0, true, false, true)) {
        record_failure();
        continue;
      }
      if (try_final(ctx.planner->currentTipPose())) {
        return finish_exit("镜片已分段移出储存区");
      }
    }
    result.message = "镜面竖直移出失败（尝试 " + std::to_string(failed_attempts) +
      " 条候选路径），最后错误: " + last_failure;
    return false;
  }

  template<class Feedback>
  bool moveLens(ArmContext & ctx, const MoveLens::Goal & goal,
                MoveLens::Result & result, Feedback feedback)
  {
    result.lens = ctx.lens;
    if (ctx.state != ArmContext::State::carried) {
      result.message = "本臂未持有已移出的镜片"; return false;
    }
    if (goal.target.header.frame_id != st::kPlanningFrame) {
      result.message = "镜片目标位姿必须位于 world 坐标系"; return false;
    }
    const Eigen::Isometry3d body_target = st::fromMsg(goal.target.pose);
    if (!body_target.matrix().allFinite()) {
      result.message = "目标位姿包含非有限值"; return false;
    }
    feedback("move_lens_body_pose");
    ctx.log_target = "镜片body目标";
    RCLCPP_DEBUG(node_->get_logger(), "[镜片目标] %s %s", context(ctx).c_str(), poseText(body_target).c_str());
    result.success = move(ctx, body_target * ctx.lens_in_tcp.inverse(),
                          "镜片移动到世界目标位姿", result.message);
    if (result.success) result.message = "镜片已到达目标位姿";
    return result.success;
  }

  template<class Feedback>
  bool dockCentre(ArmContext & ctx, const DockCentre::Goal & goal,
                  DockCentre::Result & result, Feedback feedback)
  {
    result.lens = ctx.lens;
    const int placed_lens = ctx.lens;
    if (ctx.state != ArmContext::State::carried &&
        ctx.state != ArmContext::State::centre_docked) {
      result.message = "本臂未持有可装配镜片"; return false;
    }
    std::vector<std::string> faces;
    if (ctx.state == ArmContext::State::centre_docked) {
      if (!goal.centre_face.empty() && goal.centre_face != ctx.centre_face) {
        result.message = "镜片已与另一中心接口对接，只能重试原接口的释放";
        return false;
      }
      faces = {ctx.centre_face};
    } else if (!goal.centre_face.empty()) faces = {goal.centre_face};
    else if (!goal.allowed_faces.empty()) faces = goal.allowed_faces;
    else faces = {"S0", "S1", "S2", "S3", "S4", "S5"};
    faces.erase(std::remove_if(faces.begin(), faces.end(), [](const std::string & face) {
      return !validFace(face);
    }), faces.end());
    if (goal.randomize) {
      std::lock_guard<std::mutex> lock(rng_mutex_);
      std::shuffle(faces.begin(), faces.end(), rng_);
    }
    std::string err = "没有可用中心接口";
    for (const auto & face : faces) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (used_faces_.count(face) ||
            (reserved_faces_.count(face) && reserved_faces_.at(face) != ctx.spec->name)) continue;
        reserved_faces_[face] = ctx.spec->name;
      }
      ScopeExit lease{[this, &ctx, &face]() {
        if (ctx.state != ArmContext::State::centre_docked) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          reserved_faces_.erase(face);
        }
      }};
      bool contact_active = false;
      const auto release_contact = [&]() {
        if (!contact_active) return;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          centre_contacts_.erase(face);
          contact_active = false;
        }
        centre_contact_cv_.notify_all();
      };
      ScopeExit contact_guard{release_contact};
      const auto acquire_contact = [&]() {
        std::unique_lock<std::mutex> lock(state_mutex_);
        const auto available = [&]() {
          for (const auto & entry : centre_contacts_) {
            const int gap = std::abs(int(face[1])-int(entry.first[1]));
            if (entry.second != ctx.spec->name && (gap == 0 || gap == 1 || gap == 5)) return false;
          }
          return true;
        };
        if (!available()) {
          RCLCPP_INFO(node_->get_logger(), "[共享区协调] %s 等待相邻中心面的插入、释放及撤退完成；对侧动作继续执行",
            context(ctx).c_str());
          if (!centre_contact_cv_.wait_for(lock, std::chrono::seconds(120), available)) {
            err = "等待相邻中心面对接共享区超时"; return false;
          }
        }
        centre_contacts_[face] = ctx.spec->name;
        contact_active = true;
        return true;
      };
      const auto centre = sites_.centreFace(face);
      const auto body = sites_.lensBody(ctx.lens);
      const auto s0_body = sites_.get("site", st::lensFaceSite(ctx.lens, "S0"),
                                      st::lensBodyName(ctx.lens));
      if (!centre || !body || !s0_body) { err = "实时中心/镜片位姿读取失败"; continue; }
      ctx.log_target = st::centreSite(face) +
        "；镜片对接接口=" + st::lensFaceSite(ctx.lens, "S0") +
        "；抓取接口=" + st::lensFaceSite(ctx.lens, ctx.grasp_face);
      RCLCPP_DEBUG(node_->get_logger(), "[中心接口位姿] %s %s", context(ctx).c_str(), poseText(*centre).c_str());
      // Preserve the verified grasp transform; refreshing it here could hide a
      // dropped or shifted mirror and produce a false successful assembly.
      const Eigen::Isometry3d lens_goal = (*centre) * st::rotation(st::kQDock) * s0_body->inverse();
      const Eigen::Isometry3d place = lens_goal * ctx.lens_in_tcp.inverse();
      const Eigen::Vector3d axis = st::normalOf(*centre).normalized();
      Eigen::Isometry3d preplace = place;
      preplace.translation() += approach_ * axis;
      if (ctx.state == ArmContext::State::carried) {
        {
          std::lock_guard<std::mutex> lock(scene_mutex_);
          if (!scene_.setLensCentreContact(ctx.lens, false, err)) break;
        }
        feedback("centre_preplace");
        if (!move(ctx, preplace, "直接避障到中心预装配位（接口+z外2cm）", err,
                  0.0, false, false, false, centre_planning_time_)) {
          RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=下一个允许的中心接口，无其他接口则结束",
            context(ctx).c_str(), err.c_str());
          continue;
        }
        if (!acquire_contact()) break;
        std::vector<std::string> placed;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          for (int id : used_) placed.push_back("space_sim_carried_lens_" + std::to_string(id));
        }
        {
          std::lock_guard<std::mutex> lock(scene_mutex_);
          if (!scene_.setLensCentreContact(ctx.lens, true, err)) break;
          if (!placed.empty() && !scene_.setLensTouch(ctx.lens, placed, true, err)) break;
        }
        feedback("centre_insert");
        if (!move(ctx, place, "沿中心接口插入2cm", err, axis_tolerance_, false, true)) {
          RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=下一个允许的中心接口，无 STOMP 插入回退",
            context(ctx).c_str(), err.c_str());
          continue;
        }
        feedback("centre_dock");
        const auto docked = dock_.dock(st::lensFaceSite(ctx.lens, "S0"),
                                       st::centreSite(face), "default");
        if (!docked.success) {
          err = docked.message;
          RCLCPP_WARN(node_->get_logger(), "[磁吸失败] %s 原因=%s；下一步=下一个允许的中心接口",
            context(ctx).c_str(), err.c_str());
          continue;
        }
        ctx.state = ArmContext::State::centre_docked;
        ctx.centre_face = face;
      }
      if (!contact_active && !acquire_contact()) break;
      const auto mate = sites_.mateError(st::lensFaceSite(ctx.lens, "S0"),
                                         st::centreSite(face), st::rotation(st::kQDock));
      if (!mate || mate->first >= 0.01 || mate->second >= 0.05) {
        result.message = "中心接口对合误差超限，镜片仍由机械臂持有"; return false;
      }
      const auto tool_interface = sites_.lensFace(ctx.lens, ctx.grasp_face);
      ctx.log_target = st::lensFaceSite(ctx.lens, ctx.grasp_face) +
        "；已安装中心接口=" + st::centreSite(face);
      if (tool_interface) RCLCPP_DEBUG(node_->get_logger(), "[释放接口位姿] %s %s",
        context(ctx).c_str(), poseText(*tool_interface).c_str());
      feedback("release_tool");
      const auto released = dock_.release(ctx.spec->tcp_site,
                                          st::lensFaceSite(ctx.lens, ctx.grasp_face));
      if (!released.success) { result.message = released.message; return false; }
      ctx.state = ArmContext::State::error;
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        const auto actual_body = sites_.lensBody(ctx.lens);
        if (!actual_body || !scene_.detachLens(ctx.lens, ctx.spec->tcp_link, err) ||
            !scene_.addPlacedLens(ctx.lens, *actual_body, mesh_, touchLinks(*ctx.spec), err)) {
          result.message = "中心已对接且末端已释放，但规划场景更新失败: " + err;
          return false;
        }
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        used_.insert(ctx.lens); used_faces_.insert(face); reserved_.erase(ctx.lens);
        reserved_faces_.erase(face);
      }
      ctx.state = ArmContext::State::installed_uncleared;
      ctx.lens = 0;
      const auto grasp_site = sites_.lensFace(placed_lens, ctx.grasp_face);
      if (!grasp_site) {
        result.message = "镜片已装配，但无法读取实时抓取接口轴线，禁止盲目撤退";
        return false;
      }
      const Eigen::Vector3d grasp_axis = st::normalOf(*grasp_site).normalized();
      ctx.log_target = st::lensFaceSite(placed_lens, ctx.grasp_face) +
        "；已安装中心接口=" + st::centreSite(face);
      RCLCPP_DEBUG(node_->get_logger(), "[撤退接口位姿] %s %s", context(ctx).c_str(), poseText(*grasp_site).c_str());
      const Eigen::Isometry3d released_tip = ctx.planner->currentTipPose();
      Eigen::Isometry3d retreat = released_tip;
      retreat.translation() += approach_ * grasp_axis;
      feedback("retreat");
      if (!move(ctx, retreat, "沿夹爪与镜片抓取接口的 +z 后退 2 cm",
                err, axis_tolerance_, false, true)) {
        result.message = "镜片已装配，但沿抓取接口后退 2 cm 失败: " + err; return false;
      }
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        if (!scene_.setLensTouch(placed_lens, touchLinks(*ctx.spec), false, err)) {
          result.message = "已后退 2 cm，但恢复夹爪与镜片碰撞检查失败: " + err;
          return false;
        }
      }
      if (!ctx.planner->currentStateValid(err)) {
        result.message = "已后退 2 cm，但夹爪仍与场景碰撞: " + err;
        return false;
      }
      release_contact();
      if (goal.return_home) {
        feedback("return_home");
        std::map<std::string, double> home;
        for (const auto & name : {ctx.spec->name + "_home", std::string("home"),
                                  std::string("ready")}) {
          home = ctx.planner->namedStateValues(name);
          if (!home.empty()) break;
        }
        if (home.empty()) {
          result.message = "镜片已装配，但找不到本臂 home 命名状态"; return false;
        }
        st::MotionRequest request;
        request.kind = st::MotionRequest::Kind::kJoints;
        request.what = "装配后回 home";
        request.planning_time_s = home_planning_time_;
        ctx.log_target = ctx.spec->name + "_home；已安装中心接口=" + st::centreSite(face);
        for (const auto & joint : ctx.spec->joints) request.joints.push_back(home.at(joint));
        if (!executeMotion(ctx, request, err)) {
          RCLCPP_WARN(node_->get_logger(), "[回退] %s 原因=%s；下一步=安全中继位后再用STOMP返回home",
            context(ctx).c_str(), err.c_str());
          std::string home_error = err;
          const Eigen::Isometry3d start_clear = ctx.planner->currentTipPose();
          const double outward = ctx.spec->base_position.x() >= 0.0 ? 1.0 : -1.0;
          const std::vector<Eigen::Vector3d> offsets{
            0.04 * grasp_axis,
            0.08 * grasp_axis,
            Eigen::Vector3d(0.08 * outward, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.08)};
          bool reached_home = false;
          for (const auto & offset : offsets) {
            Eigen::Isometry3d via = start_clear;
            via.translation() += offset;
            feedback("return_home_via");
            if (!move(ctx, via, "装配后安全中继位", err)) continue;
            if (executeMotion(ctx, request, err)) {
              reached_home = true;
              break;
            }
            home_error = err;
          }
          if (!reached_home) {
            result.message = "镜片已装配并撤退 2 cm，但回 home 没有安全路径: " + home_error;
            return false;
          }
        }
      }
      ctx.state = ArmContext::State::empty;
      result.success = true;
      result.centre_face = face;
      result.message = "中心镜片对接、释放并沿抓取接口撤退 2 cm 完成";
      return true;
    }
    result.message = err;
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  st::SitePoseProvider sites_;
  st::TargetBuilder targets_;
  st::DockClient dock_;
  st::SceneManager scene_;
  st::DecoupledMotionBridge motion_;
  shape_msgs::msg::Mesh mesh_;
  std::map<std::string, std::unique_ptr<ArmContext>> arms_;
  bool infrastructure_ready_{false};  // Protected by init_mutex_; does not enable actions.
  std::atomic<bool> initialized_{false};
  std::mutex init_mutex_, state_mutex_, scene_mutex_, rng_mutex_, worker_mutex_;
  std::set<int> used_, reserved_;
  std::set<std::string> used_faces_;
  std::map<std::string, std::string> reserved_faces_;
  std::map<std::string, std::string> centre_contacts_;
  std::condition_variable centre_contact_cv_;
  std::mt19937 rng_;
  std::vector<std::thread> workers_;
  double approach_ = 0.02, exit_dx_ = 0.6, axis_tolerance_ = 0.008;
  double vertical_tolerance_ = 0.10, planning_time_ = 12.0;
  double home_planning_time_ = 30.0;
  double centre_planning_time_ = 30.0;
  double velocity_scale_ = 0.1, acceleration_scale_ = 0.1;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
  rclcpp::Service<ResetEnvironment>::SharedPtr reset_service_;
  rclcpp::Client<ResetScene>::SharedPtr reset_scene_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<mujoco_god_plugin::srv::GetJointStates>::SharedPtr joints_client_;
  rclcpp::CallbackGroup::SharedPtr init_group_;
  rclcpp_action::Server<DockStorage>::SharedPtr storage_server_;
  rclcpp_action::Server<ExitStorage>::SharedPtr exit_server_;
  rclcpp_action::Server<MoveLens>::SharedPtr move_server_;
  rclcpp_action::Server<DockCentre>::SharedPtr centre_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto server = std::make_shared<TaskServer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(server->node());
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
