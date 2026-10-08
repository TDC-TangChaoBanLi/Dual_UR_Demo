// Diagnostic service: apply the task's TOTG/Ruckig processing without execution.
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit/trajectory_processing/ruckig_traj_smoothing.hpp"
#include "space_sim_task_control/moveit_params.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("space_sim_audit_time_parameterize");
  if (!space_sim_task_control::copyMoveGroupParameters(node)) return 2;
  robot_model_loader::RobotModelLoader loader(node);
  auto model = loader.getModel();
  if (!model) return 2;
  using Service = moveit_msgs::srv::GetMotionPlan;
  auto service = node->create_service<Service>("audit_time_parameterize",
    [model](Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
      const auto & input = req->motion_plan_request;
      auto & output = res->motion_plan_response;
      output.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      if (input.trajectory_constraints.constraints.empty()) return;
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      moveit::core::robotStateMsgToRobotState(input.start_state, state);
      moveit_msgs::msg::RobotTrajectory trajectory;
      for (const auto & joint : input.trajectory_constraints.constraints.front().joint_constraints)
        trajectory.joint_trajectory.joint_names.push_back(joint.joint_name);
      for (const auto & waypoint : input.trajectory_constraints.constraints) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (const auto & joint : waypoint.joint_constraints) point.positions.push_back(joint.position);
        trajectory.joint_trajectory.points.push_back(point);
      }
      robot_trajectory::RobotTrajectory rt(model, input.group_name);
      rt.setRobotTrajectoryMsg(state, trajectory);
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      if (!totg.computeTimeStamps(rt, .1, .1)) return;
      if (!trajectory_processing::RuckigSmoothing::applySmoothing(rt, 1., 1., true)) return;
      rt.getRobotTrajectoryMsg(output.trajectory);
      output.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
