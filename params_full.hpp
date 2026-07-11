// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

// silence deprecation warnings for parameter_traits, needed for backwards compatibility
#define SILENCE_DEPRECATION_WARNINGS
#include <parameter_traits/parameter_traits.hpp>
#undef SILENCE_DEPRECATION_WARNINGS

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace servo {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        int64_t thread_priority = 40;
        double publish_period = 0.01;
        double max_expected_latency = 0.1;
        std::string move_group_name;
        std::string active_subgroup = "";
        std::string pose_command_in_topic = "~/pose_target_cmds";
        std::string cartesian_command_in_topic = "~/delta_twist_cmds";
        std::string joint_command_in_topic = "~/delta_joint_cmds";
        std::string command_in_type = "unitless";
        double incoming_command_timeout = 0.1;
        bool apply_twist_commands_about_ee_frame = true;
        std::string status_topic = "~/status";
        std::string command_out_topic = "/panda_arm_controller/joint_trajectory";
        std::string command_out_type = "trajectory_msgs/JointTrajectory";
        bool publish_joint_positions = true;
        bool publish_joint_velocities = true;
        bool publish_joint_accelerations = false;
        std::string monitored_planning_scene_topic = "/planning_scene";
        std::string joint_topic = "/joint_states";
        bool check_octomap_collisions = false;
        bool is_primary_planning_scene_monitor = true;
        bool use_smoothing = true;
        std::string smoothing_filter_plugin_name = "online_signal_smoothing::ButterworthFilterPlugin";
        bool check_collisions = true;
        double self_collision_proximity_threshold = 0.01;
        double scene_collision_proximity_threshold = 0.02;
        double collision_check_rate = 10.0;
        double lower_singularity_threshold = 17.0;
        double hard_stop_singularity_threshold = 30.0;
        double leaving_singularity_threshold_multiplier = 2.0;
        double singularity_step_scale = 0.01;
        bool halt_all_joints_in_joint_mode = true;
        bool halt_all_joints_in_cartesian_mode = true;
        std::vector<double> joint_limit_margins = {0.1};
        double override_velocity_scaling_factor = 0.0;
        struct Scale {
            double linear = 0.4;
            double rotational = 0.8;
            double joint = 0.5;
        } scale;
        struct PoseTracking {
            double linear_tolerance = 0.001;
            double angular_tolerance = 0.01;
        } pose_tracking;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        int64_t thread_priority = 40;
        double publish_period = 0.01;
        double max_expected_latency = 0.1;
        double incoming_command_timeout = 0.1;
        bool apply_twist_commands_about_ee_frame = true;
        bool publish_joint_positions = true;
        bool publish_joint_velocities = true;
        bool publish_joint_accelerations = false;
        bool check_octomap_collisions = false;
        bool is_primary_planning_scene_monitor = true;
        bool use_smoothing = true;
        bool check_collisions = true;
        double self_collision_proximity_threshold = 0.01;
        double scene_collision_proximity_threshold = 0.02;
        double collision_check_rate = 10.0;
        double lower_singularity_threshold = 17.0;
        double hard_stop_singularity_threshold = 30.0;
        double leaving_singularity_threshold_multiplier = 2.0;
        double singularity_step_scale = 0.01;
        bool halt_all_joints_in_joint_mode = true;
        bool halt_all_joints_in_cartesian_mode = true;
        double override_velocity_scaling_factor = 0.0;
        struct Scale {
            double linear = 0.4;
            double rotational = 0.8;
            double joint = 0.5;
        } scale;
        struct PoseTracking {
            double linear_tolerance = 0.001;
            double angular_tolerance = 0.01;
        } pose_tracking;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    template <typename NodeT>
    ParamListener(NodeT node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("servo"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.thread_priority = params.thread_priority;
      output.publish_period = params.publish_period;
      output.max_expected_latency = params.max_expected_latency;
      output.scale.linear = params.scale.linear;
      output.scale.rotational = params.scale.rotational;
      output.scale.joint = params.scale.joint;
      output.incoming_command_timeout = params.incoming_command_timeout;
      output.apply_twist_commands_about_ee_frame = params.apply_twist_commands_about_ee_frame;
      output.pose_tracking.linear_tolerance = params.pose_tracking.linear_tolerance;
      output.pose_tracking.angular_tolerance = params.pose_tracking.angular_tolerance;
      output.publish_joint_positions = params.publish_joint_positions;
      output.publish_joint_velocities = params.publish_joint_velocities;
      output.publish_joint_accelerations = params.publish_joint_accelerations;
      output.check_octomap_collisions = params.check_octomap_collisions;
      output.is_primary_planning_scene_monitor = params.is_primary_planning_scene_monitor;
      output.use_smoothing = params.use_smoothing;
      output.check_collisions = params.check_collisions;
      output.self_collision_proximity_threshold = params.self_collision_proximity_threshold;
      output.scene_collision_proximity_threshold = params.scene_collision_proximity_threshold;
      output.collision_check_rate = params.collision_check_rate;
      output.lower_singularity_threshold = params.lower_singularity_threshold;
      output.hard_stop_singularity_threshold = params.hard_stop_singularity_threshold;
      output.leaving_singularity_threshold_multiplier = params.leaving_singularity_threshold_multiplier;
      output.singularity_step_scale = params.singularity_step_scale;
      output.halt_all_joints_in_joint_mode = params.halt_all_joints_in_joint_mode;
      output.halt_all_joints_in_cartesian_mode = params.halt_all_joints_in_cartesian_mode;
      output.override_velocity_scaling_factor = params.override_velocity_scaling_factor;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "thread_priority")) {
            if(auto validation_result = gt_eq<int64_t>(param, 0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.thread_priority = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_period")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.publish_period = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_expected_latency")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.max_expected_latency = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "move_group_name")) {
            updated_params.move_group_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "active_subgroup")) {
            updated_params.active_subgroup = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_command_in_topic")) {
            updated_params.pose_command_in_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cartesian_command_in_topic")) {
            updated_params.cartesian_command_in_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_command_in_topic")) {
            updated_params.joint_command_in_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "command_in_type")) {
            if(auto validation_result = one_of<std::string>(param, {"unitless", "speed_units"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.command_in_type = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "scale.linear")) {
            updated_params.scale.linear = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "scale.rotational")) {
            updated_params.scale.rotational = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "scale.joint")) {
            updated_params.scale.joint = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "incoming_command_timeout")) {
            updated_params.incoming_command_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "apply_twist_commands_about_ee_frame")) {
            updated_params.apply_twist_commands_about_ee_frame = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_tracking.linear_tolerance")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.pose_tracking.linear_tolerance = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_tracking.angular_tolerance")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.pose_tracking.angular_tolerance = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "status_topic")) {
            updated_params.status_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "command_out_topic")) {
            updated_params.command_out_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "command_out_type")) {
            if(auto validation_result = one_of<std::string>(param, {"trajectory_msgs/JointTrajectory", "std_msgs/Float64MultiArray"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.command_out_type = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_joint_positions")) {
            updated_params.publish_joint_positions = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_joint_velocities")) {
            updated_params.publish_joint_velocities = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_joint_accelerations")) {
            updated_params.publish_joint_accelerations = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "monitored_planning_scene_topic")) {
            updated_params.monitored_planning_scene_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_topic")) {
            updated_params.joint_topic = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "check_octomap_collisions")) {
            updated_params.check_octomap_collisions = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "is_primary_planning_scene_monitor")) {
            updated_params.is_primary_planning_scene_monitor = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_smoothing")) {
            updated_params.use_smoothing = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "smoothing_filter_plugin_name")) {
            updated_params.smoothing_filter_plugin_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "check_collisions")) {
            updated_params.check_collisions = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "self_collision_proximity_threshold")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.self_collision_proximity_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "scene_collision_proximity_threshold")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.scene_collision_proximity_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "collision_check_rate")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.collision_check_rate = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "lower_singularity_threshold")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.lower_singularity_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "hard_stop_singularity_threshold")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.hard_stop_singularity_threshold = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "leaving_singularity_threshold_multiplier")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.leaving_singularity_threshold_multiplier = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "singularity_step_scale")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.singularity_step_scale = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "halt_all_joints_in_joint_mode")) {
            updated_params.halt_all_joints_in_joint_mode = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "halt_all_joints_in_cartesian_mode")) {
            updated_params.halt_all_joints_in_cartesian_mode = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_limit_margins")) {
            if(auto validation_result = lower_element_bounds<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.joint_limit_margins = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "override_velocity_scaling_factor")) {
            if(auto validation_result = bounds<double>(param, 0.0, 1.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.override_velocity_scaling_factor = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "thread_priority")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "This value is used when configuring the servo loop thread to use SCHED_FIFO scheduling We use a slightly lower priority than the ros2_control default in order to reduce jitter Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html";
          descriptor.read_only = true;
          descriptor.integer_range.resize(1);
          descriptor.integer_range.at(0).from_value = 0;
          descriptor.integer_range.at(0).to_value = std::numeric_limits<int64_t>::max();
          auto parameter = to_parameter_value(updated_params.thread_priority);
          parameters_interface_->declare_parameter(prefix_ + "thread_priority", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_period")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = " 1 / (Nominal publish rate) [seconds]";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.publish_period);
          parameters_interface_->declare_parameter(prefix_ + "publish_period", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_expected_latency")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum expected latency between generating a servo command and the controller receiving it [seconds].";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.max_expected_latency);
          parameters_interface_->declare_parameter(prefix_ + "max_expected_latency", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "move_group_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the moveit move_group of your robot This parameter does not have a default value and must be passed to the node during launch time.";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING;
          parameters_interface_->declare_parameter(prefix_ + "move_group_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "active_subgroup")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "This parameter can be used to switch online to actuating a subgroup of the move group. If it is empty, the full move group is actuated.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.active_subgroup);
          parameters_interface_->declare_parameter(prefix_ + "active_subgroup", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_command_in_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic on which servo will receive the pose commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.pose_command_in_topic);
          parameters_interface_->declare_parameter(prefix_ + "pose_command_in_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cartesian_command_in_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic on which servo will receive the twist commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.cartesian_command_in_topic);
          parameters_interface_->declare_parameter(prefix_ + "cartesian_command_in_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_command_in_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic on which servo will receive the joint jog commands";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joint_command_in_topic);
          parameters_interface_->declare_parameter(prefix_ + "joint_command_in_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "command_in_type")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The unit of the incoming command. unitless commands are in the range [-1:1] as if from joystick speed_units are in m/s and rad/s";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.command_in_type);
          parameters_interface_->declare_parameter(prefix_ + "command_in_type", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "scale.linear")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Max linear velocity in m/s if using speed units or tracking a pose, else a scaling factor on the unitless commanded velocity. Used for Cartesian and pose tracking commands.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.scale.linear);
          parameters_interface_->declare_parameter(prefix_ + "scale.linear", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "scale.rotational")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Max angular velocity in rad/s if using speed units or tracking a pose, else a scaling factor on the unitless commanded velocity. Used for Cartesian and pose tracking commands.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.scale.rotational);
          parameters_interface_->declare_parameter(prefix_ + "scale.rotational", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "scale.joint")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Joint angular/linear velocity scale. Only used for unitless joint commands.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.scale.joint);
          parameters_interface_->declare_parameter(prefix_ + "scale.joint", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "incoming_command_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Commands will be discarded if they are older than the timeout, in seconds.Important because ROS may drop some messages.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.incoming_command_timeout);
          parameters_interface_->declare_parameter(prefix_ + "incoming_command_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "apply_twist_commands_about_ee_frame")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true, the angular velocity specified in the twist command is applied about the ee frame axes if false, the twist computed will include the linear component from rotation of ee frame about the planning frame, due to the existence of a lever between the two frames.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.apply_twist_commands_about_ee_frame);
          parameters_interface_->declare_parameter(prefix_ + "apply_twist_commands_about_ee_frame", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_tracking.linear_tolerance")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The allowable linear error, in meters, when tracking a pose.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.pose_tracking.linear_tolerance);
          parameters_interface_->declare_parameter(prefix_ + "pose_tracking.linear_tolerance", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_tracking.angular_tolerance")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The allowable angular error, in radians, when tracking a pose.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.pose_tracking.angular_tolerance);
          parameters_interface_->declare_parameter(prefix_ + "pose_tracking.angular_tolerance", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "status_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic to which the status will be published";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.status_topic);
          parameters_interface_->declare_parameter(prefix_ + "status_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "command_out_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic on which servo will publish the joint trajectory Change this to the topic your controller requires.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.command_out_topic);
          parameters_interface_->declare_parameter(prefix_ + "command_out_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "command_out_type")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The type of command that servo will publish";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.command_out_type);
          parameters_interface_->declare_parameter(prefix_ + "command_out_type", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_joint_positions")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true servo will publish joint positions in the output command";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_joint_positions);
          parameters_interface_->declare_parameter(prefix_ + "publish_joint_positions", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_joint_velocities")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true servo will publish joint velocities in the output command";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_joint_velocities);
          parameters_interface_->declare_parameter(prefix_ + "publish_joint_velocities", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_joint_accelerations")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true servo will publish joint accelerations in the output command";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_joint_accelerations);
          parameters_interface_->declare_parameter(prefix_ + "publish_joint_accelerations", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "monitored_planning_scene_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the planning scene topic. planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.monitored_planning_scene_topic);
          parameters_interface_->declare_parameter(prefix_ + "monitored_planning_scene_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_topic")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The topic on which joint states can be monitored";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joint_topic);
          parameters_interface_->declare_parameter(prefix_ + "joint_topic", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "check_octomap_collisions")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true, the planning scene monitor also listens to octomap updates for collision checking.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.check_octomap_collisions);
          parameters_interface_->declare_parameter(prefix_ + "check_octomap_collisions", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "is_primary_planning_scene_monitor")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If is_primary_planning_scene_monitor is set to true, the Servo server's PlanningScene advertises the /get_planning_scene service, which other nodes can use to get information about the planning environment. If a different node in your system will be publishing the planning scene, this should be set to false";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.is_primary_planning_scene_monitor);
          parameters_interface_->declare_parameter(prefix_ + "is_primary_planning_scene_monitor", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_smoothing")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Enables the use of smoothing plugins for joint trajectory smoothing";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.use_smoothing);
          parameters_interface_->declare_parameter(prefix_ + "use_smoothing", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "smoothing_filter_plugin_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The name of the smoothing plugin to be used";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.smoothing_filter_plugin_name);
          parameters_interface_->declare_parameter(prefix_ + "smoothing_filter_plugin_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "check_collisions")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "If true, servo will check for collision using the planning scene monitor.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.check_collisions);
          parameters_interface_->declare_parameter(prefix_ + "check_collisions", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "self_collision_proximity_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Start decelerating when a self-collision is this far [m]";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.self_collision_proximity_threshold);
          parameters_interface_->declare_parameter(prefix_ + "self_collision_proximity_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "scene_collision_proximity_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Start decelerating when a collision is this far [m]";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.scene_collision_proximity_threshold);
          parameters_interface_->declare_parameter(prefix_ + "scene_collision_proximity_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "collision_check_rate")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "[Hz] Collision-checking can easily bog down a CPU if done too often. Collision checking begins slowing down when nearer than a specified distance.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.collision_check_rate);
          parameters_interface_->declare_parameter(prefix_ + "collision_check_rate", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "lower_singularity_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Start decelerating when the condition number hits this (close to singularity)";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.lower_singularity_threshold);
          parameters_interface_->declare_parameter(prefix_ + "lower_singularity_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "hard_stop_singularity_threshold")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Stop when the condition number hits this";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.hard_stop_singularity_threshold);
          parameters_interface_->declare_parameter(prefix_ + "hard_stop_singularity_threshold", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "leaving_singularity_threshold_multiplier")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "When 'lower_singularity_threshold' is triggered, but we are moving away from singularity, move this many times faster than if we were moving further into singularity";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.leaving_singularity_threshold_multiplier);
          parameters_interface_->declare_parameter(prefix_ + "leaving_singularity_threshold_multiplier", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "singularity_step_scale")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The test vector towards singularity is scaled by this factor during singularity check";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.singularity_step_scale);
          parameters_interface_->declare_parameter(prefix_ + "singularity_step_scale", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "halt_all_joints_in_joint_mode")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Halt all joints in joint mode, else halt only the joints at their limit";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.halt_all_joints_in_joint_mode);
          parameters_interface_->declare_parameter(prefix_ + "halt_all_joints_in_joint_mode", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "halt_all_joints_in_cartesian_mode")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Halt all joints in cartesian mode, else halt only the joints at their limit";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.halt_all_joints_in_cartesian_mode);
          parameters_interface_->declare_parameter(prefix_ + "halt_all_joints_in_cartesian_mode", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_limit_margins")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Added as a buffer to joint variable position bounds [in that joint variable's respective units]. Can be of size 1, which applies the margin to all joints, or the same size as the number of degrees of freedom of the active joint group. If moving quickly, make these values larger.";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.joint_limit_margins);
          parameters_interface_->declare_parameter(prefix_ + "joint_limit_margins", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "override_velocity_scaling_factor")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Scaling factor when servo overrides the velocity (e.g, near singularities)";
          descriptor.read_only = false;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = 1.0;
          auto parameter = to_parameter_value(updated_params.override_velocity_scaling_factor);
          parameters_interface_->declare_parameter(prefix_ + "override_velocity_scaling_factor", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "thread_priority");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "thread_priority") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<int64_t>(param, 0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'thread_priority': {}", validation_result.error()));
      }
      updated_params.thread_priority = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "publish_period");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "publish_period") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'publish_period': {}", validation_result.error()));
      }
      updated_params.publish_period = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_expected_latency");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "max_expected_latency") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'max_expected_latency': {}", validation_result.error()));
      }
      updated_params.max_expected_latency = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "move_group_name");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "move_group_name") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.move_group_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "active_subgroup");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "active_subgroup") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.active_subgroup = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "pose_command_in_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "pose_command_in_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.pose_command_in_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "cartesian_command_in_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "cartesian_command_in_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cartesian_command_in_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "joint_command_in_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "joint_command_in_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_command_in_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "command_in_type");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "command_in_type") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"unitless", "speed_units"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'command_in_type': {}", validation_result.error()));
      }
      updated_params.command_in_type = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "scale.linear");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "scale.linear") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.scale.linear = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "scale.rotational");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "scale.rotational") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.scale.rotational = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "scale.joint");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "scale.joint") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.scale.joint = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "incoming_command_timeout");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "incoming_command_timeout") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.incoming_command_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "apply_twist_commands_about_ee_frame");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "apply_twist_commands_about_ee_frame") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.apply_twist_commands_about_ee_frame = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "pose_tracking.linear_tolerance");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "pose_tracking.linear_tolerance") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'pose_tracking.linear_tolerance': {}", validation_result.error()));
      }
      updated_params.pose_tracking.linear_tolerance = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "pose_tracking.angular_tolerance");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "pose_tracking.angular_tolerance") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'pose_tracking.angular_tolerance': {}", validation_result.error()));
      }
      updated_params.pose_tracking.angular_tolerance = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "status_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "status_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.status_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "command_out_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "command_out_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.command_out_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "command_out_type");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "command_out_type") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"trajectory_msgs/JointTrajectory", "std_msgs/Float64MultiArray"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'command_out_type': {}", validation_result.error()));
      }
      updated_params.command_out_type = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "publish_joint_positions");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "publish_joint_positions") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_joint_positions = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "publish_joint_velocities");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "publish_joint_velocities") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_joint_velocities = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "publish_joint_accelerations");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "publish_joint_accelerations") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_joint_accelerations = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "monitored_planning_scene_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "monitored_planning_scene_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.monitored_planning_scene_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "joint_topic");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "joint_topic") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_topic = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "check_octomap_collisions");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "check_octomap_collisions") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.check_octomap_collisions = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "is_primary_planning_scene_monitor");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "is_primary_planning_scene_monitor") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.is_primary_planning_scene_monitor = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "use_smoothing");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "use_smoothing") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_smoothing = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "smoothing_filter_plugin_name");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "smoothing_filter_plugin_name") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.smoothing_filter_plugin_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "check_collisions");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "check_collisions") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.check_collisions = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "self_collision_proximity_threshold");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "self_collision_proximity_threshold") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'self_collision_proximity_threshold': {}", validation_result.error()));
      }
      updated_params.self_collision_proximity_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "scene_collision_proximity_threshold");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "scene_collision_proximity_threshold") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'scene_collision_proximity_threshold': {}", validation_result.error()));
      }
      updated_params.scene_collision_proximity_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "collision_check_rate");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "collision_check_rate") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'collision_check_rate': {}", validation_result.error()));
      }
      updated_params.collision_check_rate = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "lower_singularity_threshold");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "lower_singularity_threshold") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'lower_singularity_threshold': {}", validation_result.error()));
      }
      updated_params.lower_singularity_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "hard_stop_singularity_threshold");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "hard_stop_singularity_threshold") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'hard_stop_singularity_threshold': {}", validation_result.error()));
      }
      updated_params.hard_stop_singularity_threshold = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "leaving_singularity_threshold_multiplier");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "leaving_singularity_threshold_multiplier") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'leaving_singularity_threshold_multiplier': {}", validation_result.error()));
      }
      updated_params.leaving_singularity_threshold_multiplier = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "singularity_step_scale");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "singularity_step_scale") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'singularity_step_scale': {}", validation_result.error()));
      }
      updated_params.singularity_step_scale = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "halt_all_joints_in_joint_mode");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "halt_all_joints_in_joint_mode") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.halt_all_joints_in_joint_mode = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "halt_all_joints_in_cartesian_mode");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "halt_all_joints_in_cartesian_mode") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.halt_all_joints_in_cartesian_mode = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "joint_limit_margins");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "joint_limit_margins") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = lower_element_bounds<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'joint_limit_margins': {}", validation_result.error()));
      }
      updated_params.joint_limit_margins = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "override_velocity_scaling_factor");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "override_velocity_scaling_factor") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = bounds<double>(param, 0.0, 1.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'override_velocity_scaling_factor': {}", validation_result.error()));
      }
      updated_params.override_velocity_scaling_factor = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace servo
