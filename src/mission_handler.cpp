/* each ros package must have these */
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/srv/get_path_srv.hpp>
#include <mrs_msgs/srv/validate_reference_array.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_msgs/srv/transform_reference_srv.hpp>
#include <mrs_msgs/srv/transform_reference_array_srv.hpp>

#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>


#include <mrs_robot_diagnostics/enums/uav_state.h>
#include <mrs_robot_diagnostics/enums/tracker_state.h>
#include <mrs_robot_diagnostics/enums/enum_helpers.h>

#include <mrs_msgs/msg/uav_state.hpp>

#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

#include <atomic>
#include <tuple>
#include <mutex>

#include <iroc_mission_handler/action/mission.hpp>
#include <iroc_mission_handler/srv/upload_mission_srv.hpp>
#include <iroc_mission_handler/srv/unload_mission_srv.hpp>
#include "iroc_mission_handler/enums/mission_state.h"
#include "iroc_mission_handler/subtask_manager.h"

namespace iroc_mission_handler
{

// Type aliases for better readability
using Mission           = iroc_mission_handler::action::Mission;
using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;

class MissionHandler : public mrs_lib::Node {
public:
  MissionHandler(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;


  // | --------------------- types and structs --------------------- |
  struct result_t
  {
    bool success;
    std::string message;
  };

  /**
   * \brief Struct to hold path segments.
   *
   * This struct contains a path, a validity flag, a vector of subtasks, and execution mode.
   * - `path`: The path segment.
   * - `is_valid`: A boolean indicating whether the path segment is a valid movement path (true) or just heading changes (false).
   * - `subtasks`: A vector of subtasks that will be executed at the end of the path segment.
   * - `parallel_execution`: Whether subtasks should be executed in parallel or sequentially.
   */
  struct path_segment_t
  {
    mrs_msgs::msg::Path path;
    bool is_valid;
    std::vector<iroc_mission_handler::msg::Subtask> subtasks;
    bool parallel_execution = false;
  };

  /**
   * \brief Struct to hold trajectory information and track subtasks.
   *
   * This struct contains a trajectory reference, a vector of trajectory indices, a vector of subtasks, and execution mode.
   * - `reference`: The trajectory reference.
   * - `idxs`: A vector of indices corresponding to the trajectory points indices (each index corresponds to a point in the trajectory).
   * - `subtasks`: A vector of subtasks associated with the trajectory.
   * - `parallel_execution`: Whether subtasks should be executed in parallel or sequentially.
   */
  struct trajectory_t
  {
    mrs_msgs::msg::TrajectoryReference reference;
    std::vector<long int> idxs;
    std::vector<iroc_mission_handler::msg::Subtask> subtasks;
    bool parallel_execution = false;
  };

  /**
   * \brief Struct to hold metrics for the mission handler.
   *
   * - `remaining_distance`: The remaining distance to the mission finish point.
   * - `eta`: The estimated time of arrival to the mission end.
   * - `progress`: The overall mission progress as a percentage (0.0 - 100.0).
   */
  struct metrics_t
  {
    double remaining_distance = 0.0;
    double eta                = 0.0;
    double progress           = 0.0;
  };

  typedef mrs_robot_diagnostics::state_t state_t;
  // enum_helpers::enum_updater<state_t> uav_state_             = {"UAV STATE", state_t::UNKNOWN};
  enum_helpers::enum_updater<state_t> uav_state_;
  // enum_helpers::enum_updater<mission_state_t> mission_state_ = {"MISSION STATE", mission_state_t::IDLE};
  enum_helpers::enum_updater<mission_state_t> mission_state_;
  mission_state_t previous_mission_state_ = mission_state_t::IDLE;

  std::string robot_name_;
  std::atomic_bool is_initialized_ = false;
  double _min_distance_threshold_; // Minimum distance to consider a segment as valid (not just a heading change)
  double _trajectory_sampling_period_;
  double _takeoff_timeout_s_;       // Max seconds to wait for the UAV to reach hover after a takeoff call

  rclcpp::Time takeoff_started_at_; // Timestamp of the most recent takeoff service call

  // | -------------------- subtask management ------------------- |
  // TODO: implement subtask manager and use it to manage the execution of subtasks
  std::unique_ptr<SubtaskManager> subtask_manager_;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::State> sh_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  void controlManagerDiagCallback(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);

  // | ----------------------- ROS services ---------------------- |
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_takeoff_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_land_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_land_home_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv> sc_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv> sc_get_path_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_hover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_mission_flying_to_start_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_mission_start_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_mission_pause_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray> sc_mission_validation_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv> sc_trajectory_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv> sc_transform_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceArraySrv> sc_transform_reference_array_;

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_activation_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ss_pausing_;
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UploadMissionSrv> ss_upload_mission_;
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UnloadMissionSrv> ss_unload_mission_;

  std::atomic<bool> is_mission_staged_{false};

  bool missionActivationServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool missionPausingServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool uploadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Request> request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Response> response);
  bool unloadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Request> request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Response> response);

  // | ----------------------- main timer ----------------------- |
  std::shared_ptr<TimerType> timer_main_;
  void timerMain();

  std::shared_ptr<TimerType> timer_feedback_;
  void timerFeedback();

  void initialize(void);
  void shutdown();

  // Action server
  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;
  std::shared_ptr<GoalHandleMission> current_goal_handle_;
  std::recursive_mutex action_server_mutex_;

  void actionPublishFeedback();
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);
  void handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  // | --------------------- mission feedback and trajectory t-------------------- |
  std::vector<trajectory_t> trajectories_;
  int current_trajectory_idx_          = 0; // Index of the current trajectory being executed
  int current_trajectory_waypoint_idx_ = 0; // Index of the current waypoint in the current trajectory

  std::atomic_bool is_current_trajectory_finished_ = false;
  std::atomic_bool is_trajectory_sent_             = false;

  // Waypoint information (which waypoint is currently being followed)
  int mission_waypoint_idx_ = 0; // Index of the current waypoint being followed

  metrics_t mission_metrics_;  // Metrics for the current mission
  metrics_t waypoint_metrics_; // Metrics for the current waypoint

  // Trajectory sampling period (it is to compute the mission metrics because the trajectory is sampled at this period)
  double mission_progress_before_pause_ = 0.0; // Progress before the mission was paused

  // | ------------------ Additional functions ------------------ |
  result_t createMission(const std::shared_ptr<const Mission::Goal> goal);
  bool replanMission();
  void resetMission();

  // Trajectory management functions
  result_t sendTrajectoryToController(const trajectory_t &trajectory);
  void createSubtasks(const std::vector<iroc_mission_handler::msg::Subtask> &subtasks);

  result_t validateTrajectory(const trajectory_t &trajectory);
  std::vector<path_segment_t> segmentPath(const mrs_msgs::msg::Path &msg, const std::vector<iroc_mission_handler::msg::Waypoint> &waypoints);
  std::tuple<std::vector<mrs_msgs::msg::Reference>, std::vector<long int>> generateHeadingTrajectory(const mrs_msgs::msg::Path &path, double T);
  std::tuple<result_t, std::vector<trajectory_t>> generateTrajectoriesFromSegments(const std::vector<path_segment_t> &path_segments);

  // Miscellaneous functions
  double distance(const mrs_msgs::msg::Reference &waypoint_1, const mrs_msgs::msg::Reference &waypoint_2);
  void updateMissionState(const mission_state_t &new_state);

  // Call service methods overloads
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                       const std::shared_ptr<typename ServiceType::Response> &response);
};

MissionHandler::MissionHandler(rclcpp::NodeOptions options)
    : mrs_lib::Node("MissionHandler", options.enable_logger_service(true)), uav_state_(this_node_ptr()->get_logger(), "UAV STATE", state_t::UNKNOWN),
      mission_state_(this_node_ptr()->get_logger(), "MISSION STATE", mission_state_t::IDLE) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_action_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();
}

void MissionHandler::initialize() {

  // Load configuration files
  mrs_lib::ParamLoader param_loader(node_, "MissionHandler");

  param_loader.loadParam("robot_name", robot_name_);

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("trajectory_generation_config");

  // Load parameters
  const auto main_timer_rate     = param_loader.loadParam2<double>("mission_handler/main_timer_rate");
  const auto feedback_timer_rate = param_loader.loadParam2<double>("mission_handler/feedback_timer_rate");

  _min_distance_threshold_     = param_loader.loadParam2<double>("mrs_uav_trajectory_generation/min_waypoint_distance");
  _trajectory_sampling_period_ = param_loader.loadParam2<double>("mrs_uav_trajectory_generation/sampling_dt");
  _takeoff_timeout_s_          = param_loader.loadParam2<double>("mission_handler/takeoff_timeout");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |
  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(node_, rclcpp::Rate(1.0));

  mrs_lib::SubscriberHandlerOptions sh_opts;
  sh_opts.node               = node_;
  sh_opts.node_name          = "MissionHandler";
  sh_opts.no_message_timeout = std::chrono::seconds(5);
  sh_opts.timeout_manager    = tim_mgr_;
  sh_opts.threadsafe         = true;
  sh_opts.autostart          = true;

  sh_state_                = mrs_lib::SubscriberHandler<mrs_msgs::msg::State>(sh_opts, "~/uav_state_in");
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(sh_opts, "~/control_manager_diagnostics_in",
                                                                                                  &MissionHandler::controlManagerDiagCallback, this);

  // | --------------------- service clients -------------------- |
  sc_takeoff_                 = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_takeoff_in", cbkgrp_sc_);
  sc_land_                    = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_land_in", cbkgrp_sc_);
  sc_land_home_               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_land_home_in", cbkgrp_sc_);
  sc_path_                    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>(node_, "~/svc_path_in", cbkgrp_sc_);
  sc_get_path_                = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>(node_, "~/svc_get_path_in", cbkgrp_sc_);
  sc_hover_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_hover_in", cbkgrp_sc_);
  sc_mission_flying_to_start_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_mission_flying_to_start_in", cbkgrp_sc_);
  sc_mission_start_           = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_mission_start_in", cbkgrp_sc_);
  sc_mission_pause_           = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/svc_mission_pause_in", cbkgrp_sc_);
  sc_mission_validation_      = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray>(node_, "~/svc_mission_validation_in", cbkgrp_sc_);
  sc_trajectory_reference_    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>(node_, "~/svc_trajectory_reference_in", cbkgrp_sc_);
  sc_transform_reference_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv>(node_, "~/svc_transform_reference_in", cbkgrp_sc_);
  sc_transform_reference_array_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceArraySrv>(node_, "~/svc_transform_reference_array_in", cbkgrp_sc_);


  // | --------------------- service servers -------------------- |
  ss_activation_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/svs_mission_activation_out",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        return missionActivationServiceCallback(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_pausing_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/svs_mission_pausing_out",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        return missionPausingServiceCallback(request, response);
      },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_upload_mission_ = mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UploadMissionSrv>(
      node_, "~/svs_upload_mission_out",
      [this](std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Request> request,
             std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Response> response) { return uploadMissionServiceCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  ss_unload_mission_ = mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UnloadMissionSrv>(
      node_, "~/svs_unload_mission_out",
      [this](std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Request> request,
             std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Response> response) { return unloadMissionServiceCallback(request, response); },
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  // | ------------------------- timers ------------------------- |
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), [this]() { this->timerMain(); });
  }

  {
    timer_feedback_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(feedback_timer_rate, clock_), [this]() { this->timerFeedback(); });
  }

  action_server_ptr_ = rclcpp_action::create_server<Mission>(
      node_, "mission_handler", [this](auto uuid, auto goal) { return handle_goal(uuid, goal); },
      [this](auto goal_handle) { return handle_cancel(goal_handle); }, [this](auto goal_handle) { handle_accepted(goal_handle); },
      rcl_action_server_get_default_options(), cbkgrp_action_);

  // | -------------------- subtask manager -------------------- |
  subtask_manager_ = std::make_unique<SubtaskManager>(node_);

  RCLCPP_INFO(node_->get_logger(), "initialized");
  RCLCPP_INFO(node_->get_logger(), "--------------------");
  is_initialized_ = true;
}

// | ------------------------- timers  ------------------------ |
/**
 * \brief Main timer callback function, which is called periodically to handle the mission state and UAV state.
 *
 * This function checks the UAV state, updates the mission state accordingly, and handles the action server goals.
 * It also manages the transition between different mission states based on the UAV's current state.
 *
 * Similar to a behavior tree, this function evaluates the current state and transitions, executing actions based on conditions and priorities.
 *
 * \param event The timer event containing information about the timer.
 */
void MissionHandler::timerMain() {
  std::scoped_lock lock(action_server_mutex_);
  if (!is_initialized_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Waiting for nodelet initialization");
    return;
  }

  // | -------------------- UAV state parsing ------------------- |
  if (sh_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<state_t>(sh_state_.getMsg()->state));
  }

  // |-----------------------------------------------------------|
  // |                   State machine logic                     |
  // |-----------------------------------------------------------|
  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    return;
  }

  // Detect landing state and update mission state accordingly
  const bool not_idle_or_land = mission_state_.value() != mission_state_t::IDLE && mission_state_.value() != mission_state_t::LAND;
  if (uav_state_.value() == state_t::LAND && not_idle_or_land) {
    RCLCPP_INFO(node_->get_logger(), "Landing detected. Switching to LAND state.");
    updateMissionState(mission_state_t::LAND);
    return;
  }

  // Check for RC mode during active missions
  if (uav_state_.value() == state_t::RC_MODE) {
    if (mission_state_.value() != mission_state_t::PAUSED_DUE_TO_RC_MODE) {
      RCLCPP_INFO(node_->get_logger(), "Mission is paused due to active MRS Remote mode. Disable the mode to continue with the mission execution.");
      updateMissionState(mission_state_t::PAUSED_DUE_TO_RC_MODE);
    }
    return;
  }

  // Check for manual control during active missions
  if (uav_state_.value() == state_t::MANUAL) {
    auto result                  = std::make_shared<Mission::Result>();
    result->robot_result.name    = robot_name_;
    result->robot_result.success = false;
    result->robot_result.message = "Mission cancelled because drone is under manual control.";

    current_goal_handle_->abort(result);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Mission cancelled because drone is under manual control.");

    updateMissionState(mission_state_t::IDLE);
    resetMission();
    return;
  }

  switch (mission_state_.value()) {
  case mission_state_t::EXECUTING: {
    if (current_trajectory_idx_ >= trajectories_.size()) {
      if (uav_state_.value() == state_t::HOVER) { // Wait for the UAV currently executing trajectory to finish
        updateMissionState(mission_state_t::FINISHED);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Mission finished. No more trajectories to execute.");
      }
      break;
    }

    if (!mrs_robot_diagnostics::is_flying(uav_state_.value())) {
      RCLCPP_INFO(node_->get_logger(), "UAV is not flying. Calling takeoff.");

      auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp = callService<std_srvs::srv::Trigger>(sc_takeoff_, request);

      if (!resp.success) {
        auto result                  = std::make_shared<Mission::Result>();
        result->robot_result.name    = robot_name_;
        result->robot_result.success = false;
        result->robot_result.message = "Takeoff service call failed: " + resp.message;
        RCLCPP_WARN_STREAM(node_->get_logger(), result->robot_result.message);
        current_goal_handle_->abort(result);
        updateMissionState(mission_state_t::IDLE);
        resetMission();
        return;
      }

      takeoff_started_at_ = clock_->now();
      updateMissionState(mission_state_t::TAKEOFF);
      break;
    }

    // Check if the current trajectory is finished
    if (is_current_trajectory_finished_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory " << current_trajectory_idx_ << " finished.");

      if (!trajectories_[current_trajectory_idx_].subtasks.empty()) {
        // TODO maybe remove or set as debug
        RCLCPP_INFO_STREAM(node_->get_logger(), "Subtasks to execute: ");
        for (const auto &subtask : trajectories_[current_trajectory_idx_].subtasks)
          RCLCPP_INFO_STREAM(node_->get_logger(), "- " << subtask.type);

        RCLCPP_INFO_STREAM(node_->get_logger(), "Executing subtasks in the waypoint: " << mission_waypoint_idx_);
        subtask_manager_->createSubtasks(trajectories_[current_trajectory_idx_].subtasks);

        updateMissionState(mission_state_t::EXECUTING_SUBTASK);
        break;
      }

      // Move to next trajectory
      current_trajectory_idx_++;
      is_current_trajectory_finished_ = false;
      break;
    }

    // Send and start the trajectory
    if (uav_state_.value() == state_t::HOVER && !is_trajectory_sent_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Starting trajectory " << current_trajectory_idx_ + 1 << "/" << trajectories_.size());
      auto trajectory_result = sendTrajectoryToController(trajectories_[current_trajectory_idx_]);
      if (!trajectory_result.success) {
        RCLCPP_WARN(node_->get_logger(), "Failed to send trajectory: %s", trajectory_result.message.c_str());

        auto mission_result                  = std::make_shared<Mission::Result>();
        mission_result->robot_result.name    = robot_name_;
        mission_result->robot_result.success = false;
        mission_result->robot_result.message = trajectory_result.message;
        current_goal_handle_->abort(mission_result);

        current_trajectory_idx_ = 0;
        updateMissionState(mission_state_t::IDLE);
        resetMission();
        return;
      }

      auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp = callService<std_srvs::srv::Trigger>(sc_mission_start_, request);
      if (!resp.success) {
        RCLCPP_WARN(node_->get_logger(), " Mission start call was not successful with message: %s", resp.message.c_str());
        updateMissionState(mission_state_t::MISSION_LOADED);
      }
      is_trajectory_sent_ = true;
      break;
    }

    break;
  }

  case mission_state_t::EXECUTING_SUBTASK: {
    // Execute the current subtask
    if (trajectories_[current_trajectory_idx_].parallel_execution) {
      subtask_manager_->startAllSubtasks();
    } else {

      double progress = 0.0;
      if (subtask_manager_->isCurrentSubtaskCompleted(progress)) {
        subtask_manager_->startNextSubtask();
      } else {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Subtask is still running. Progress: " << progress * 100.0 << "%");
      }
    }

    // Check if any critical subtasks have failed
    if (subtask_manager_->areCriticalSubtasksFailed()) {
      RCLCPP_WARN(node_->get_logger(), " Critical subtask failed. Aborting mission.");
      auto mission_result                  = std::make_shared<Mission::Result>();
      mission_result->robot_result.name    = robot_name_;
      mission_result->robot_result.success = false;
      mission_result->robot_result.message = "Critical subtask failed.";
      current_goal_handle_->abort(mission_result);

      updateMissionState(mission_state_t::IDLE);
      resetMission();
      return;
    }

    // Continue with the mission if all subtasks are completed
    if (subtask_manager_->areAllSubtasksCompleted()) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "All subtasks completed for trajectory " << current_trajectory_idx_);

      // Move to next trajectory
      current_trajectory_idx_++;
      is_current_trajectory_finished_ = false;
      updateMissionState(mission_state_t::EXECUTING);
    }

    break;
  }

  case mission_state_t::TAKEOFF: {
    if (mrs_robot_diagnostics::is_flying(uav_state_.value())) {
      RCLCPP_INFO(node_->get_logger(), "UAV reached hover altitude. Starting mission execution.");
      updateMissionState(mission_state_t::EXECUTING);
      break;
    }

    const double elapsed_s = (clock_->now() - takeoff_started_at_).seconds();
    if (elapsed_s > _takeoff_timeout_s_) {
      auto result                  = std::make_shared<Mission::Result>();
      result->robot_result.name    = robot_name_;
      result->robot_result.success = false;
      result->robot_result.message = "Takeoff timed out after " + std::to_string(static_cast<int>(_takeoff_timeout_s_)) + "s.";
      RCLCPP_WARN_STREAM(node_->get_logger(), result->robot_result.message);
      current_goal_handle_->abort(result);
      updateMissionState(mission_state_t::IDLE);
      resetMission();
      return;
    }

    break;
  }

  case mission_state_t::FINISHED: {
    switch (current_goal_handle_->get_goal()->robot_goal.terminal_action) {
    case Mission::Goal::TERMINAL_ACTION_LAND: {
      RCLCPP_INFO(node_->get_logger(), "Executing terminal action. Calling land");
      auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp = callService<std_srvs::srv::Trigger>(sc_land_, request);
      if (!resp.success) {
        RCLCPP_WARN(node_->get_logger(), " Land call was not successful with message: %s", resp.message.c_str());
        return;
      }

      updateMissionState(mission_state_t::LAND);
      break;
    }

    case Mission::Goal::TERMINAL_ACTION_RTH: {
      RCLCPP_INFO(node_->get_logger(), "Executing terminal action. Calling land home");
      auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp = callService<std_srvs::srv::Trigger>(sc_land_home_, request);
      if (!resp.success) {
        RCLCPP_WARN(node_->get_logger(), " Land home call was not successful with message: %s", resp.message.c_str());
        return;
      }

      updateMissionState(mission_state_t::RTH);
      break;
    }

    default: {
      auto mission_result                  = std::make_shared<Mission::Result>();
      mission_result->robot_result.name    = robot_name_;
      mission_result->robot_result.success = true;
      mission_result->robot_result.message = "Mission finished.";
      current_goal_handle_->succeed(mission_result);

      updateMissionState(mission_state_t::IDLE);
      break;
    }
    }

    // Reset mission state and trajectory tracking
    resetMission();
    break;
  }

  case mission_state_t::LAND: {
    if (uav_state_.value() == state_t::ARMED || uav_state_.value() == state_t::DISARMED || uav_state_.value() == state_t::OFFBOARD) {
      RCLCPP_INFO(node_->get_logger(), "Landing finished.");

      // iroc_mission_handler::MissionResult action_server_result;
      auto mission_result = std::make_shared<Mission::Result>();

      if (previous_mission_state_ == mission_state_t::FINISHED) {
        mission_result->robot_result.name    = robot_name_;
        mission_result->robot_result.success = true;
        mission_result->robot_result.message = "Mission finished";

        RCLCPP_INFO(node_->get_logger(), "Mission finished.");
        current_goal_handle_->succeed(mission_result);
      } else {
        mission_result->robot_result.name    = robot_name_;
        mission_result->robot_result.success = false;
        mission_result->robot_result.message = "Mission stopped due to landing.";

        RCLCPP_WARN(node_->get_logger(), "Mission stopped due to landing.");
        current_goal_handle_->abort(mission_result);
      }

      updateMissionState(mission_state_t::IDLE);
      resetMission();
    }

    break;
  }

  case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
    // mission continue if we are again not in RC_mode
    if (uav_state_.value() != state_t::RC_MODE) {
      RCLCPP_INFO(node_->get_logger(), "RC mode disabled. Switching to previous mission mode");
      updateMissionState(previous_mission_state_);
    }
    break;
  }

  default:
    break;
  }
}

void MissionHandler::timerFeedback() {
  if (!is_initialized_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Waiting for initialization");
    return;
  }
  actionPublishFeedback();
}

// | ----------------- service server callback ---------------- |
bool MissionHandler::missionActivationServiceCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::scoped_lock lock(action_server_mutex_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received mission activation request.");

  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    response->success = false;
    response->message = "No active mission.";
    RCLCPP_WARN(node_->get_logger(), "No active mission.");
    return true;
  }

  switch (mission_state_.value()) {
  case mission_state_t::MISSION_LOADED: {
    RCLCPP_INFO(node_->get_logger(), "Mission loaded, starting mission with first trajectory.");
    is_current_trajectory_finished_ = false;
    updateMissionState(mission_state_t::EXECUTING);
    break;
  }

  case mission_state_t::PAUSED: {
    RCLCPP_INFO(node_->get_logger(), "Replanning mission from current position");
    const auto replan_res = replanMission();
    if (!replan_res) {
      RCLCPP_WARN(node_->get_logger(), " Failed to replan mission.");
      response->success = false;
      response->message = "failed to replan mission";
      return true;
    } else {
      RCLCPP_INFO(node_->get_logger(), " Replanning mission successfully.");
      updateMissionState(mission_state_t::MISSION_LOADED);
    }

    RCLCPP_INFO(node_->get_logger(), "Resuming mission with current trajectory.");
    updateMissionState(mission_state_t::EXECUTING);
    break;
  }

  case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
    response->success = false;
    response->message = "Mission is paused due to active MRS Remote mode. Disable the mode to continue mission execution.";
    RCLCPP_WARN(node_->get_logger(), "Mission is paused due to active MRS Remote mode. Disable the mode to continue mission execution.");
    return true;
  }

  default: {
    response->success = false;
    response->message = "Mission is already activated.";
    RCLCPP_WARN(node_->get_logger(), "Mission is already activated.");
    return true;
  }
  }

  response->success = true;
  response->message = "Mission activated successfully.";
  return true;
}

bool MissionHandler::missionPausingServiceCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                   const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::scoped_lock lock(action_server_mutex_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received mission pausing request.");
  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    response->success = false;
    response->message = "No active mission.";
    RCLCPP_WARN(node_->get_logger(), "No active mission.");
  }

  mission_progress_before_pause_ = mission_metrics_.progress;
  switch (mission_state_.value()) {
  case mission_state_t::MISSION_LOADED: {
    RCLCPP_INFO(node_->get_logger(), "Mission paused before execution.");
    response->success = true;
    response->message = "Mission paused before execution.";
    updateMissionState(mission_state_t::PAUSED);
    break;
  }

  case mission_state_t::EXECUTING: {
    RCLCPP_INFO(node_->get_logger(), "Calling hover service to stop trajectory tracking.");
    auto request      = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto resp   = callService<std_srvs::srv::Trigger>(sc_hover_, request);
    response->success = resp.success;
    response->message = resp.message;
    if (!resp.success) {
      RCLCPP_WARN(node_->get_logger(), " Failed to call hover service with message: %s", resp.message.c_str());
      break;
    }

    updateMissionState(mission_state_t::PAUSED);
    break;
  }

  default: {
    response->success = false;
    response->message = "Mission is in the state in which cannot be paused.";
    RCLCPP_WARN(node_->get_logger(), "Mission is in the state in which cannot be paused.");
    break;
  }
  }

  return true;
}

bool MissionHandler::uploadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Request> request,
                                                  const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Response> response) {
  std::scoped_lock lock(action_server_mutex_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received upload mission request.");

  if (!is_initialized_) {
    response->success = false;
    response->message = "Not initialized";
    return true;
  }

  if (mission_state_.value() != mission_state_t::IDLE) {
    response->success = false;
    std::stringstream ss;
    ss << "Robot is not IDLE, current state: " << to_string(mission_state_.value());
    response->message = ss.str();
    RCLCPP_WARN_STREAM(node_->get_logger(), "Upload rejected: " << response->message);
    return true;
  }

  // Build a synthetic action goal from the service request
  auto synthetic_goal        = std::make_shared<Mission::Goal>();
  synthetic_goal->robot_goal = request->robot_goal;

  const auto result = createMission(synthetic_goal);

  if (!result.success) {
    response->success = false;
    response->message = result.message;
    RCLCPP_WARN_STREAM(node_->get_logger(), "Upload failed: " << result.message);
    return true;
  }

  is_mission_staged_ = true;
  updateMissionState(mission_state_t::MISSION_LOADED);

  response->success = true;
  response->message = result.message;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Mission staged successfully: " << result.message);
  return true;
}

bool MissionHandler::unloadMissionServiceCallback([[maybe_unused]] const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Request> request,
                                                  const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Response> response) {
  std::scoped_lock lock(action_server_mutex_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Received unload mission request.");

  if (!is_mission_staged_) {
    response->success = false;
    response->message = "No staged mission to unload";
    RCLCPP_WARN(node_->get_logger(), "Unload rejected: no staged mission.");
    return true;
  }

  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    response->success = false;
    response->message = "Action is currently executing, cannot unload";
    RCLCPP_WARN(node_->get_logger(), "Unload rejected: action executing.");
    return true;
  }

  resetMission();
  is_mission_staged_ = false;
  updateMissionState(mission_state_t::IDLE);

  response->success = true;
  response->message = "Mission unloaded successfully";
  RCLCPP_INFO(node_->get_logger(), "Mission unloaded successfully.");
  return true;
}

// | ----------------- msg callback ---------------- |
/**
 * \brief Callback function for ControlManagerDiagnostics messages.
 *
 * This function processes the diagnostics data to update the mission state, current trajectory, and waypoint information.
 * It calculates the distance to the next waypoint, estimated time of arrival (ETA), and progress towards the next waypoint.
 * It also checks if the current waypoint has been reached or if the current trajectory has been completed.
 *
 * \param diagnostics The ControlManagerDiagnostics message containing the current state of the control manager.
 */
void MissionHandler::controlManagerDiagCallback(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr diagnostics) {
  std::scoped_lock lock(action_server_mutex_);

  if (!is_initialized_ ||                                       // Node initialization check
      !diagnostics || !diagnostics->tracker_status.have_goal || // Diagnostics check
      current_trajectory_idx_ >= trajectories_.size() ||        // Current trajectory index check
      mission_state_.value() != mission_state_t::EXECUTING) {   // Mission state check
    return;
  }

  // Restarting flag given that we will process the current trajectory
  if (is_trajectory_sent_) {
    is_trajectory_sent_ = false;
  }
  // Get current state
  int current_point_idx            = diagnostics->tracker_status.trajectory_idx;
  trajectory_t &current_trajectory = trajectories_.at(current_trajectory_idx_);

  int previous_waypoint_point_idx = current_trajectory_waypoint_idx_ > 0 ? current_trajectory.idxs[current_trajectory_waypoint_idx_ - 1] : 0;
  int next_waypoint_point_idx     = current_trajectory.idxs[current_trajectory_waypoint_idx_];

  // | ----------------------- Check if current waypoint is reached ----------------------- |
  if (current_point_idx >= current_trajectory.idxs[current_trajectory_waypoint_idx_]) {
    RCLCPP_INFO(node_->get_logger(), "Reached waypoint %d in trajectory %d", current_trajectory_waypoint_idx_, current_trajectory_idx_);

    // Reached the current waypoint, update the mission state and indices
    mission_waypoint_idx_++;
    current_trajectory_waypoint_idx_++;

    if (current_trajectory_waypoint_idx_ >= current_trajectory.idxs.size()) {
      // If we reached the last waypoint in the trajectory, mark it as finished and reset the trajectory waypoint index
      RCLCPP_INFO(node_->get_logger(), "Trajectory %d finished", current_trajectory_idx_);
      is_current_trajectory_finished_  = true;
      current_trajectory_waypoint_idx_ = 0;
    }

    return;
  }

  // | ----------------------- Update waypoint metrics ----------------------- |
  const mrs_msgs::msg::Reference current_position       = current_trajectory.reference.points.at(current_point_idx);
  const mrs_msgs::msg::Reference next_waypoint_position = current_trajectory.reference.points.at(next_waypoint_point_idx);

  // Number of points in the current path segment (waypoint to next waypoint)
  const int number_of_points = next_waypoint_point_idx - previous_waypoint_point_idx;
  double waypoint_progress   = number_of_points > 0 ? (static_cast<double>(current_point_idx - previous_waypoint_point_idx) / number_of_points) * 100.0 : 0.0;

  waypoint_metrics_.remaining_distance = distance(current_position, next_waypoint_position);
  waypoint_metrics_.eta                = std::max(static_cast<double>(next_waypoint_point_idx - current_point_idx) * _trajectory_sampling_period_, 0.0);
  waypoint_metrics_.progress           = std::min(waypoint_progress, 100.0);

  // | ----------------------- Update mission metrics ----------------------- |
  double remaining_distance     = 0.0;
  unsigned int remaining_points = 0;
  unsigned int total_num_points = 0;
  for (size_t i = 0; i < trajectories_.size(); i++) {
    total_num_points += trajectories_[i].reference.points.size() - 1; // Add the number of points in the trajectory

    if (i < current_trajectory_idx_) {
      continue; // Skip trajectories before the current one
    }
    for (size_t j = 0; j < trajectories_[i].reference.points.size() - 1; j++) {
      if (i <= current_trajectory_idx_ && j < current_point_idx) {
        continue; // Skip points before the current waypoint
      }

      const mrs_msgs::msg::Reference start_position = trajectories_[i].reference.points.at(j);
      const mrs_msgs::msg::Reference end_position   = trajectories_[i].reference.points.at(j + 1);

      remaining_distance += distance(start_position, end_position);
      remaining_points++;
    }
    remaining_points++; // Count the last point of the trajectory as well
  }
  double current_progress = static_cast<double>(total_num_points - remaining_points) / total_num_points * 100.0;

  mission_metrics_.progress           = std::min(mission_progress_before_pause_ + current_progress * (1.0 - (mission_progress_before_pause_ / 100.0)), 100.0);
  mission_metrics_.remaining_distance = remaining_distance;
  mission_metrics_.eta                = static_cast<double>(remaining_points) * _trajectory_sampling_period_;

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1.0, "Current waypoint metrics: Remaining distance: %.2f, ETA: %.2f, Progress: %.2f%%",
                        waypoint_metrics_.remaining_distance, waypoint_metrics_.eta, waypoint_metrics_.progress);
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1.0, "Mission metrics: Remaining distance: %.2f, ETA: %.2f, Progress: %.2f%%",
                        mission_metrics_.remaining_distance, mission_metrics_.eta, mission_metrics_.progress);
}

// | ---------------------- action server callbacks --------------------- |

/*!
 * Handles and processes goals from action clients.
 *
 * Workflow:
 * 1. Calls virtual methods defined in child classes to get goals for each robot
 * 2. Ensures all missions follow the MissionRobotGoal message structure
 *    required by Mission Handler
 *
 * @param goal The incoming goal from the action client
 */
rclcpp_action::GoalResponse MissionHandler::handle_goal(const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const Mission::Goal> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request with ID %s", rclcpp_action::to_string(uuid).c_str());

  if (!is_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Rejecting goal: not initialized yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Accept if IDLE (normal slow-path) or MISSION_LOADED with a staged mission (fast-path).
  // Any other active state means a mission is already running on this robot.
  const auto state = mission_state_.value();
  if (state != mission_state_t::IDLE && !is_mission_staged_) {
    RCLCPP_WARN(node_->get_logger(), "Rejecting goal: robot not idle (state: %s).", to_string(state));
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node_->get_logger(), "Accepting goal.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void MissionHandler::handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle) {

  if (!is_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Not initialized yet, rejecting goal.");
    return;
  }

  // Fast-path: mission was pre-validated via upload service — skip createMission()
  if (is_mission_staged_) {
    {
      std::scoped_lock lock(action_server_mutex_);
      is_mission_staged_   = false;
      current_goal_handle_ = goal_handle;
    }
    RCLCPP_INFO(node_->get_logger(), "Fast-path: using pre-staged mission.");
    return;
  }

  auto goal = goal_handle->get_goal();

  // Slow-path: serialize createMission() under the same mutex as the upload path to
  // prevent a data race if an action arrives concurrently with an upload service call.
  result_t result;
  {
    std::scoped_lock lock(action_server_mutex_);
    result               = createMission(goal);
    current_goal_handle_ = goal_handle;
  }

  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create mission from goal with message: %s", result.message.c_str());
    auto result_msg                  = std::make_shared<Mission::Result>();
    result_msg->robot_result.name    = robot_name_;
    result_msg->robot_result.success = false;
    result_msg->robot_result.message = "Failed to create mission from goal: " + result.message;
    goal_handle->abort(result_msg);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Mission created successfully from goal.");
  updateMissionState(mission_state_t::MISSION_LOADED);
}

rclcpp_action::CancelResponse MissionHandler::handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle) {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");

  if (current_goal_handle_->is_active()) {
    switch (mission_state_.value()) {
    case mission_state_t::EXECUTING: {
      RCLCPP_INFO(node_->get_logger(), "Drone is in the movement -> Calling hover.");

      auto request    = std::make_shared<std_srvs::srv::Trigger::Request>();
      const auto resp = callService<std_srvs::srv::Trigger>(sc_hover_, request);

      if (!resp.success) {
        RCLCPP_WARN(node_->get_logger(), "Failed to call hover service.");
      }

      auto result                  = std::make_shared<Mission::Result>();
      result->robot_result.name    = robot_name_;
      result->robot_result.success = false;
      result->robot_result.message = "Mission cancelled by client request.";
      current_goal_handle_->abort(result);
      RCLCPP_INFO(node_->get_logger(), "Mission stopped by cancel request.");
      updateMissionState(mission_state_t::IDLE);
      return rclcpp_action::CancelResponse::ACCEPT;
      break;
    }
    default:
      auto result                  = std::make_shared<Mission::Result>();
      result->robot_result.name    = robot_name_;
      result->robot_result.success = false;
      result->robot_result.message = "Mission cancelled by client request.";
      current_goal_handle_->abort(result);
      RCLCPP_INFO(node_->get_logger(), "Mission stopped by cancel request.");
      updateMissionState(mission_state_t::IDLE);
      return rclcpp_action::CancelResponse::ACCEPT;
      break;
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "No active mission to cancel.");
    return rclcpp_action::CancelResponse::REJECT;
  }
}


void MissionHandler::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);

  if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
    return;
  }

  auto feedback                                          = std::make_shared<Mission::Feedback>();
  feedback->robot_feedback.name                          = robot_name_;
  feedback->robot_feedback.message                       = to_string(mission_state_.value());
  feedback->robot_feedback.goal_idx                      = mission_waypoint_idx_;
  feedback->robot_feedback.distance_to_closest_goal      = waypoint_metrics_.remaining_distance;
  feedback->robot_feedback.goal_estimated_arrival_time   = waypoint_metrics_.eta;
  feedback->robot_feedback.goal_progress                 = waypoint_metrics_.progress;
  feedback->robot_feedback.distance_to_finish            = mission_metrics_.remaining_distance;
  feedback->robot_feedback.finish_estimated_arrival_time = mission_metrics_.eta;
  feedback->robot_feedback.mission_progress              = mission_metrics_.progress;

  current_goal_handle_->publish_feedback(feedback);
}

// | -------------------- support functions ------------------- |
MissionHandler::result_t MissionHandler::createMission(const std::shared_ptr<const Mission::Goal> goal) {
  std::stringstream ss;

  // Parameter validation
  if (!(goal->robot_goal.frame_id == Mission::Goal::FRAME_ID_LOCAL || goal->robot_goal.frame_id == Mission::Goal::FRAME_ID_LATLON ||
        goal->robot_goal.frame_id == Mission::Goal::FRAME_ID_FCU)) {
    ss << "Unknown frame_id = \'" << int(goal->robot_goal.frame_id) << "\', use the predefined ones.";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
    return {false, ss.str()};
  }

  if (!(goal->robot_goal.height_id == Mission::Goal::HEIGHT_ID_AGL || goal->robot_goal.height_id == Mission::Goal::HEIGHT_ID_AMSL ||
        goal->robot_goal.height_id == Mission::Goal::HEIGHT_ID_FCU)) {
    ss << "Unknown height_id = \'" << int(goal->robot_goal.height_id) << "\', use the predefined ones.";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
    return {false, ss.str()};
  }

  if (!(goal->robot_goal.terminal_action == Mission::Goal::TERMINAL_ACTION_NONE || goal->robot_goal.terminal_action == Mission::Goal::TERMINAL_ACTION_LAND ||
        goal->robot_goal.terminal_action == Mission::Goal::TERMINAL_ACTION_RTH)) {
    ss << "Unknown terminal_action = \'" << int(goal->robot_goal.terminal_action) << "\', use the predefined ones.";
    RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
    return {false, ss.str()};
  }

  for (const auto &point : goal->robot_goal.points) {
    // Validate subtasks for each point
    auto [success, error_message] = subtask_manager_->validateSubtasks(point.subtasks);
    if (!success) {
      return {false, "Subtask validation failed for point: " + error_message};
    }

    // Validate parallel execution logic
    if (point.parallel_execution) {
      std::unordered_set<std::string> seen;
      for (const auto &subtask : point.subtasks) {
        if (seen.count(subtask.type)) {
          return {false, "Subtask type '" + subtask.type + "' is duplicated in parallel execution."};
        } else {
          seen.insert(subtask.type);
        }
      }
    }

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Point reference: x: " << point.reference.position.x << " y: " << point.reference.position.y
                                                                    << " z: " << point.reference.position.z << " h: " << point.reference.heading);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "Point has " << point.subtasks.size() << " subtasks, parallel execution: " << (point.parallel_execution ? "true" : "false"));
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Subtasks: ");
    for (const auto &subtask : point.subtasks) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "- Type: " << subtask.type << ", Parameters: " << subtask.parameters);
    }
  }
  RCLCPP_INFO(node_->get_logger(), "All subtasks validated successfully");

  std::string frame_id;
  switch (goal->robot_goal.frame_id) {
  case Mission::Goal::FRAME_ID_LOCAL: {
    frame_id = "local_origin";
    break;
  }

  case Mission::Goal::FRAME_ID_LATLON: {
    frame_id = "latlon_origin";
    break;
  }

  case Mission::Goal::FRAME_ID_FCU: {
    frame_id = "fcu_untilted";
    break;
  }

  default:
    break;
  }

  if (sh_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<state_t>(sh_state_.getMsg()->state));
  }

  // Reject mission if fcu_frame is set and uav not flying
  if (goal->robot_goal.frame_id == Mission::Goal::FRAME_ID_FCU) {
    if (uav_state_.value() != state_t::HOVER) {
      ss << "FCU frame is set but uav is not in the air ";
      RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
      return {false, ss.str()};
    }
  }

  // Saving the AGL height points specified in the goal, this is needed as they will be
  // replaced after doing a transformation with latlon points
  std::vector<double> height_points;
  if (goal->robot_goal.height_id == Mission::Goal::HEIGHT_ID_AGL) {
    for (const auto &point : goal->robot_goal.points) {
      height_points.push_back(point.reference.position.z);
    }
  }

  // Create reference array with received points to transform it into current control frame
  mrs_msgs::msg::ReferenceArray goal_points_array;
  goal_points_array.header.frame_id = frame_id;
  goal_points_array.array.clear();
  goal_points_array.array.reserve(goal->robot_goal.points.size());

  for (const auto &point : goal->robot_goal.points) {
    goal_points_array.array.push_back(point.reference);
  }

  auto request  = std::make_shared<mrs_msgs::srv::TransformReferenceArraySrv::Request>();
  auto response = std::make_shared<mrs_msgs::srv::TransformReferenceArraySrv::Response>();

  request->array       = goal_points_array;
  request->to_frame_id = "";

  auto service_result = callService<mrs_msgs::srv::TransformReferenceArraySrv>(sc_transform_reference_array_, request, response);

  if (!service_result.success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to call transform reference array service with message: %s", service_result.message.c_str());
    return {false, "Failed to call transform reference array service"};
  }

  if (goal->robot_goal.height_id == Mission::Goal::HEIGHT_ID_AGL && goal->robot_goal.frame_id != Mission::Goal::FRAME_ID_FCU) {
    // Replacing the height points after the transformation, as when receiving LATLON points the transformation also considers the height as AMSL.
    auto size = response->array.array.size();
    for (size_t i = 0; i < response->array.array.size(); i++) {
      response->array.array.at(i).position.z = height_points.at(i);
    }
  }

  for (const auto &point : response->array.array) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "Transformed point x: " << point.position.x << " y: " << point.position.y << " z: " << point.position.z << " h: " << point.heading);
  }

  mrs_msgs::msg::Path msg_path;
  msg_path.points                     = response->array.array;
  msg_path.header.stamp               = clock_->now();
  msg_path.fly_now                    = false;
  msg_path.use_heading                = true;
  msg_path.dont_prepend_current_state = false; // do not use the current position for planning of the path
  msg_path.header.frame_id            = response->array.header.frame_id;

  // Segmenting the path into segments based on subtasks and heading trajectories
  std::vector<path_segment_t> path_segments = segmentPath(msg_path, goal->robot_goal.points);

  // Generating trajectory from the path segments
  auto [result, trajectories] = generateTrajectoriesFromSegments(path_segments);
  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to get trajectory from segments: %s", result.message.c_str());
    return {false, result.message};
  } else if (trajectories.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No trajectories generated from segments.");
    return {false, "No trajectories generated from segments"};
  }

  for (const auto &trajectory : trajectories) {
    auto validation_result = validateTrajectory(trajectory);
    if (!validation_result.success) {
      RCLCPP_WARN(node_->get_logger(), "Trajectory validation failed: %s", validation_result.message.c_str());
      RCLCPP_WARN_STREAM(node_->get_logger(), "Trajectory validation failed: " << validation_result.message);
      return {false, validation_result.message};
    } else {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Trajectory validation succeeded: " << validation_result.message);
    }
  }

  // Setting the mission information
  trajectories_ = trajectories;

  return {true, "Mission created successfully for " + robot_name_ + ", with " + std::to_string(trajectories.size()) + " trajectories."};
}

/**
 * \brief Validates the trajectory against the safety area and sends it to the control manager.
 *
 * This function checks if the trajectory points are within the safety area using a service call.
 * If the trajectory is valid, it sends the trajectory to the control manager for execution.
 *
 * \param trajectory The trajectory to be validated.
 *
 * \return A result_t structure indicating success or failure of the validation and a message.
 */
MissionHandler::result_t MissionHandler::validateTrajectory(const trajectory_t &trajectory) {
  // Create a ReferenceArray from the trajectory points for sending to the validation service
  mrs_msgs::msg::ReferenceArray waypointArray;
  waypointArray.header = trajectory.reference.header;
  waypointArray.array  = trajectory.reference.points;

  auto request   = std::make_shared<mrs_msgs::srv::ValidateReferenceArray::Request>();
  request->array = waypointArray;

  auto response = sc_mission_validation_.callSync(request);

  if (!response) {
    RCLCPP_WARN(node_->get_logger(), "Failed to call mission validation service.");
    return {false, "Failed to call mission validation service"};
  }

  const bool all_success = std::all_of(response.value()->success.begin(), response.value()->success.end(), [](bool v) { return v; });

  if (all_success) {
    RCLCPP_INFO(node_->get_logger(), "Successfully called service with response \"%s\".", response.value()->message.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "Trajectory points outside of safety area, with response \"%s\".", response.value()->message.c_str());
    std::vector<mrs_msgs::msg::Reference> invalid_points;
    for (auto &point_id : trajectory.idxs) {
      if (!response.value()->success.at(point_id))
        invalid_points.push_back(trajectory.reference.points.at(point_id));
    }

    for (auto &point : invalid_points)
      RCLCPP_WARN(node_->get_logger(), "Unvalid point position x: %.2f y: %.2f z: %.2f h: %.2f", point.position.x, point.position.y, point.position.z,
                  point.heading);

    if (invalid_points.size() == 0) {
      RCLCPP_WARN(node_->get_logger(),
                  "The given points are valid, however the generated trajectory seems to be outside of safety area or within an obstacle.");
      return {false, "The given points are valid for: " + robot_name_ +
                         ", however the generated trajectory seems to be outside of safety area or within an obstacle."};
    } else {
      return {false, "Unvalid trajectory for " + robot_name_ + ", trajectory is outside of safety area"};
    }
  }

  return {true, "Trajectory is valid for " + robot_name_ + ", sending to control manager"};
}

/**
 * \brief Segments the path into smaller segments based on the distance between points and subtasks.
 *
 * This function processes the input path message and segments it into smaller paths based on the distance between the consecutive points. If the distance is
 * less than a threshold (0.05), it marks the segment as invalid to be processed as a heading trajectory later. It also checks for subtasks at each waypoint
 * to determine if a new segment should be started.
 *
 * \param msg The input path message containing reference points.
 * \param waypoints A vector of waypoints with their subtasks and execution flags.
 *
 * \return A vector of segmented paths.
 *
 * \details This example illustrates how the segmentation works with a path consisting of several points (P0, P1, P2, P3, P4, P5) and their respective
 * distances.
 *
 *                     Y
 *                     ↑
 *                     | dist > 0.05m                                             ▶ = 0°
 *        P1 ▶ ────────┼─────────────── P2 ▶                                      ▲ = 90°
 *       /             |                 \ dist < 0.05m (heading only)            ◀ = 180°
 *      /              |                  P3 ▼                                    ▼ = 270°
 *     /               |                   \
 *   P0 ▲              |                    \
 *                     |                     \
 * ────────────────────┼──────────────────────\──────→ X
 *                     |                       \
 *                     |                        \
 *                     |              [Subtask] P4 ▶ ────── P5 ▲
 *
 *  Segmentation Analysis:
 *   - P0 to P1: Valid segment (distance > 0.05m)
 *   - P1 to P2: Valid segment (distance > 0.05m)
 *   - P2 to P3: Invalid segment (distance < 0.05m, heading trajectory)
 *   - P3 to P4: Valid segment (distance > 0.05m, new subtask)
 *   - P4 to P5: Valid segment (distance > 0.05m)
 */
std::vector<MissionHandler::path_segment_t> MissionHandler::segmentPath(const mrs_msgs::msg::Path &msg,
                                                                        const std::vector<iroc_mission_handler::msg::Waypoint> &waypoints) {
  // Input validation
  if (msg.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Empty path provided to segmentPath.");
    return {};
  }

  if (msg.points.size() != waypoints.size()) {
    RCLCPP_WARN(node_->get_logger(), "Number of waypoints (%zu) does not match number of points in the path (%zu).", waypoints.size(), msg.points.size());
    return {};
  }

  std::vector<path_segment_t> path_segments;

  path_segment_t current_segment;
  current_segment.path.header      = msg.header;
  current_segment.path.fly_now     = msg.fly_now;
  current_segment.path.use_heading = msg.use_heading;

  // Add the first point to start a segment
  current_segment.path.points.push_back(msg.points[0]);
  current_segment.is_valid = true; // Start with the first point as valid

  // Process points from index 1 to end
  for (size_t i = 1; i < msg.points.size(); i++) {
    const double dist = distance(msg.points[i - 1], msg.points[i]);

    if (dist < _min_distance_threshold_) {
      if (current_segment.is_valid) { // If the segment is valid, we need to finalize it and start a new one
        path_segments.push_back(current_segment);

        current_segment.path.points.clear();
        current_segment.path.points.push_back(msg.points[i - 1]); // Start a new segment with the current point
        current_segment.is_valid = false;                         // Mark the segment as invalid for heading trajectory processing
      }
    } else {
      if (!current_segment.is_valid) { // If the segment was invalid, we need to reset it
        path_segments.push_back(current_segment);

        current_segment.path.points.clear();
        current_segment.path.points.push_back(msg.points[i - 1]); // Start a new segment with the current point
        current_segment.is_valid = true;                          // Reset validity for the next segment
      }
    }

    // Add the current point to the segment
    current_segment.path.points.push_back(msg.points[i]);

    // Check if there are subtasks for the current waypoint that require segment break
    if (!waypoints.at(i).subtasks.empty()) {
      current_segment.subtasks           = waypoints.at(i).subtasks;
      current_segment.parallel_execution = waypoints.at(i).parallel_execution;
      path_segments.push_back(current_segment);

      // Reset the current segment for the next points
      current_segment.path.points.clear();
      current_segment.subtasks.clear();
      current_segment.parallel_execution = false;

      // Start a new segment with the current point
      current_segment.path.points.push_back(msg.points[i]);
      if (i + 1 < msg.points.size() && distance(msg.points[i], msg.points[i + 1]) < _min_distance_threshold_) {
        current_segment.is_valid = false;
      } else {
        current_segment.is_valid = true;
      }
    }
  }

  // Add the last segment if it has more than one point or if there are no segments yet (i.e., the first point was added)
  if (current_segment.path.points.size() > 1 || path_segments.empty()) {
    path_segments.push_back(current_segment);
  }

  // Debugging information
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Path frame_id: " << msg.header.frame_id << ", fly_now: " << (msg.fly_now ? "true" : "false")
                                                             << ", use_heading: " << (msg.use_heading ? "true" : "false"));
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Segmented path into " << path_segments.size() << " segments.");

  for (size_t segment_idx = 0; segment_idx < path_segments.size(); ++segment_idx) {
    const auto &segment = path_segments[segment_idx];
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Segment " << segment_idx + 1 << ": " << (segment.is_valid ? "Valid" : "Invalid") << ", Subtasks: "
                                                        << segment.subtasks.size() << ", Parallel: " << (segment.parallel_execution ? "Yes" : "No"));
    for (const auto &point : segment.path.points)
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "  Point position x: " << point.position.x << " y: " << point.position.y << " z: " << point.position.z
                                                                      << " heading: " << point.heading);
  }

  return path_segments;
}

/**
 * \brief Generates a trajectory from the given path segments.
 *
 * This function processes the path segments and generates a trajectory by calling the `getPath` service for valid
 * segments. For invalid segments, it generates a heading trajectory using the `generateHeadingTrajectory` function. It
 * will append trajectories to `trajectories_` if a subtask is present. For heading trajectories, it will append the
 * trajectory points to the current trajectory.
 *
 * \param path_segments A vector of path segments to process.
 *
 * \return A tuple containing the result of the trajectory generation and a vector of generated trajectories.
 */
std::tuple<MissionHandler::result_t, std::vector<MissionHandler::trajectory_t>>
MissionHandler::generateTrajectoriesFromSegments(const std::vector<path_segment_t> &path_segments) {
  rclcpp::Time trajectory_generation_start_time_ = clock_->now();

  std::vector<trajectory_t> trajectories;

  mrs_msgs::msg::TrajectoryReference current_trajectory;
  std::vector<long int> current_trajectory_idxs;

  bool is_first_segment = true;
  for (auto segment : path_segments) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[MissionHandler]: Processing segment with " << segment.path.points.size() << " points, "
                                                                                          << (segment.is_valid ? "valid" : "invalid")
                                                                                          << ", subtasks: " << segment.subtasks.size());
    if (!is_first_segment) {
      segment.path.dont_prepend_current_state = true; // Do not add current position to the trajectory
    } else {
      is_first_segment = false;
    }

    std::vector<mrs_msgs::msg::Reference> points_to_add;
    std::vector<long int> idxs_to_add;

    // If the segment is invalid, we need to generate a heading trajectory otherwise we will call the getPath service
    if (segment.is_valid) {
      auto service_request  = std::make_shared<mrs_msgs::srv::GetPathSrv::Request>();
      auto service_response = std::make_shared<mrs_msgs::srv::GetPathSrv::Response>();
      service_request->path = segment.path;


      auto service_result = callService<mrs_msgs::srv::GetPathSrv>(sc_get_path_, service_request, service_response);

      if (!service_result.success) {
        RCLCPP_WARN(node_->get_logger(), "Failed to call getPath service with message: %s", service_result.message.c_str());
        return {result_t{false, service_result.message}, {}};
      }

      // Copy the header of the trajectory
      current_trajectory.header       = service_response->trajectory.header;
      current_trajectory.header.stamp = clock_->now();
      current_trajectory.input_id     = service_response->trajectory.input_id;
      current_trajectory.use_heading  = service_response->trajectory.use_heading;
      current_trajectory.fly_now      = service_response->trajectory.fly_now;
      current_trajectory.loop         = service_response->trajectory.loop;
      current_trajectory.dt           = service_response->trajectory.dt;

      // Save the trajectory points and indices from the response
      points_to_add = service_response->trajectory.points;
      idxs_to_add   = service_response->waypoint_trajectory_idxs;
    } else {
      std::tie(points_to_add, idxs_to_add) = generateHeadingTrajectory(segment.path, _trajectory_sampling_period_);
    }

    RCLCPP_DEBUG_STREAM(node_->get_logger(), " Points to add and their headings:");
    for (const auto &point : points_to_add)
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "  Point position x: " << point.position.x << " y: " << point.position.y << " z: " << point.position.z
                                                                      << " heading: " << point.heading);

    if ((points_to_add.empty() || idxs_to_add.empty()) && !segment.subtasks.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "No points or indices to add for the segment, skipping. %zu subtasks will not be executed.", segment.subtasks.size());
      return {result_t{false, "Empty trajectory was generated and subtasks could not be executed. Check the points in the path."}, {}};
    }

    // Add points to the current trajectory
    const long int points_size = current_trajectory.points.size(); // Save the size of points before appending new ones
    current_trajectory.points.insert(current_trajectory.points.end(), points_to_add.begin(), points_to_add.end());

    for (const auto &idx : idxs_to_add) { // Add trajectory idxs to vector
      auto idx_to_add = points_size + idx;
      current_trajectory_idxs.push_back(idx_to_add);
    }

    // If the segment has subtasks, we need to finalize the current trajectory (it will stop the mission while executing
    // the subtask if it is needed)
    if (segment.subtasks.size() > 0) {
      trajectory_t trajectory;
      trajectory.reference          = current_trajectory;
      trajectory.idxs               = current_trajectory_idxs;
      trajectory.subtasks           = segment.subtasks;
      trajectory.parallel_execution = segment.parallel_execution;

      trajectories.push_back(trajectory);

      // Reset variables for the next trajectory
      current_trajectory = mrs_msgs::msg::TrajectoryReference();
      current_trajectory_idxs.clear();
    }
  }

  // If there are still points in the current trajectory, we need to finalize it
  if (!current_trajectory.points.empty()) {
    trajectory_t trajectory;
    trajectory.reference = current_trajectory;
    trajectory.idxs      = current_trajectory_idxs;

    trajectories.push_back(trajectory);
  }

  // Log the trajectory generation time and number of trajectories
  const double generation_time = (clock_->now() - trajectory_generation_start_time_).seconds();
  // ROS_INFO("[MissionHandler]: Trajectory generation took: %f seconds", generation_time);
  RCLCPP_INFO(node_->get_logger(), "Trajectory generation took: %.2f seconds", generation_time);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Number of trajectories generated: " << trajectories.size());

  for (size_t i = 0; i < trajectories.size(); ++i) {
    const auto &trajectory = trajectories[i];
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "Trajectory " << i << " has " << trajectory.reference.points.size() << " points and " << trajectory.idxs.size() << " waypoints.");
  }

  return {result_t{true, "Successfully generated trajectory"}, trajectories};
}

/**
 * \brief Generates a heading trajectory based on the input path.
 *
 * This function takes a path and generates a trajectory by interpolating the heading between consecutive points.
 * If the distance between two points is less than a threshold, it generates intermediate points with interpolated headings.
 *
 * \param path The input path containing reference points.
 * \param T The period for heading interpolation (default is 0.2 seconds).
 *
 * \return A tuple containing the generated trajectory and the indices of the trajectory points.
 */
std::tuple<std::vector<mrs_msgs::msg::Reference>, std::vector<long int>> MissionHandler::generateHeadingTrajectory(const mrs_msgs::msg::Path &path,
                                                                                                                   double T = 0.2) {
  using radians  = mrs_lib::geometry::radians;
  using sradians = mrs_lib::geometry::sradians;

  std::vector<mrs_msgs::msg::Reference> trajectory;
  std::vector<long int> trajectory_idxs;

  if (path.points.empty()) {
    return {trajectory, trajectory_idxs};
  }

  // Interpolation to the next points within segment
  mrs_msgs::msg::Reference p0 = path.points[0];
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto &p1 = path.points[i];

    double h0 = p0.heading;
    double h1 = p1.heading;

    // Only interpolate heading if it's different
    if (std::abs(sradians::diff(h0, h1)) > 1e-6) {
      // Calculate number of samples for heading interpolation
      auto heading_diff = sradians::diff(h0, h1);
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "[MissionHandler]: h1 " << h0 << ", h2 " << h1 << " diff: " << heading_diff);
      // absolute value of heading diff
      int num_heading_samples = static_cast<int>(std::ceil(std::abs(heading_diff) / T));

      for (int j = 1; j <= num_heading_samples; ++j) {
        double t = static_cast<double>(j) / num_heading_samples;

        mrs_msgs::msg::Reference point;
        // Copy position from p0
        point.position.x = p0.position.x;
        point.position.y = p0.position.y;
        point.position.z = p0.position.z;

        point.heading = h0 + t * (std::abs(heading_diff));
        trajectory.push_back(point);
      }

      trajectory_idxs.push_back(trajectory.size() - 1);
    }
    p0 = p1;
  }

  // Print trajectory
  for (const auto &point : trajectory)
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Trajectory point position x: " << point.position.x << " y: " << point.position.y << " z: " << point.position.z
                                                                             << " heading: " << point.heading);
  return {trajectory, trajectory_idxs};
}

/**
 * \brief Replans the mission from the current trajectory and goal index.
 *
 * This function replans the mission by generating a new trajectory from the remaining path segments after the current goal.
 * It validates the new trajectory and updates the mission state accordingly. It just takes the remaining points from the current trajectory, the next
 * trajectories are not considered and will be added later.
 *
 * \return True if the replanning was successful, false otherwise.
 */
bool MissionHandler::replanMission() {
  std::scoped_lock lock(action_server_mutex_);

  RCLCPP_WARN(node_->get_logger(), "Replanning trajectory %d, current goal index: %d, waypoints %zu, points %zu", current_trajectory_idx_,
              current_trajectory_waypoint_idx_, trajectories_[current_trajectory_idx_].idxs.size(),
              trajectories_[current_trajectory_idx_].reference.points.size());

  // Create waypoints from the remaining points and subtasks
  std::vector<iroc_mission_handler::msg::Waypoint> remaining_waypoints;
  remaining_waypoints.resize(trajectories_[current_trajectory_idx_].idxs.size() - current_trajectory_waypoint_idx_);
  remaining_waypoints.back().subtasks           = trajectories_[current_trajectory_idx_].subtasks;           // Copy the last subtask to the last point
  remaining_waypoints.back().parallel_execution = trajectories_[current_trajectory_idx_].parallel_execution; // Copy the parallel execution flag

  std::vector<mrs_msgs::msg::Reference> remaining_points;
  for (size_t i = current_trajectory_waypoint_idx_; i < trajectories_[current_trajectory_idx_].idxs.size(); i++) {
    remaining_points.push_back(trajectories_[current_trajectory_idx_].reference.points[trajectories_[current_trajectory_idx_].idxs[i]]);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[MissionHandler]: Remaining point "
                                                 << i - current_trajectory_waypoint_idx_ << ": x: " << remaining_points.back().position.x
                                                 << " y: " << remaining_points.back().position.y << " z: " << remaining_points.back().position.z
                                                 << " heading: " << remaining_points.back().heading);
  }

  mrs_msgs::msg::Path remaining_path;
  remaining_path.points                     = remaining_points;
  remaining_path.header.stamp               = clock_->now();
  remaining_path.fly_now                    = false;
  remaining_path.use_heading                = true;
  remaining_path.dont_prepend_current_state = false; // Use the current position for planning of the path
  remaining_path.header.frame_id            = trajectories_[current_trajectory_idx_].reference.header.frame_id;

  std::vector<path_segment_t> path_segments = segmentPath(remaining_path, remaining_waypoints);

  // Generating trajectory from the path segments
  auto [result, recomputed_trajectories] = generateTrajectoriesFromSegments(path_segments);
  if (!result.success) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get trajectory from segments: " << result.message);
    return false;
  } else if (recomputed_trajectories.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "No trajectories generated from segments.");
    return false;
  }

  for (const auto &trajectory : recomputed_trajectories) {
    auto validation_result = validateTrajectory(trajectory);
    if (!validation_result.success) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Trajectory validation failed: " << validation_result.message);
      RCLCPP_WARN_STREAM(node_->get_logger(), "Trajectory points outside of safety area!");
      return false;
    } else {
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                          "Trajectory has " << trajectory.reference.points.size() << " points and " << trajectory.idxs.size() << " waypoints.");
    }
  }
  recomputed_trajectories.insert(recomputed_trajectories.end(), trajectories_.begin() + current_trajectory_idx_ + 1, trajectories_.end());

  // Setting the mission information
  trajectories_                    = recomputed_trajectories;
  current_trajectory_waypoint_idx_ = 0; // Reset the goal index to the first goal
  current_trajectory_idx_          = 0; // Reset the current trajectory index to the first trajectory
  is_current_trajectory_finished_  = false;

  return true;
}

/**
 * \brief Sends a trajectory to the control manager for execution.
 *
 * This function sends the trajectory reference to the control manager and initializes
 * the trajectory tracking variables.
 *
 * \param trajectory The trajectory to be sent to the control manager.
 *
 * \return A result_t structure indicating success or failure of the operation.
 */
MissionHandler::result_t MissionHandler::sendTrajectoryToController(const trajectory_t &trajectory) {

  auto request        = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>();
  auto response       = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Response>();
  request->trajectory = trajectory.reference;

  auto service_response = callService<mrs_msgs::srv::TrajectoryReferenceSrv>(sc_trajectory_reference_, request, response);

  if (!service_response.success) {
    RCLCPP_WARN(node_->get_logger(), "Failed to call trajectory reference service with message: %s", service_response.message.c_str());
    return {false, "Failed to call trajectory reference service: " + service_response.message};
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Trajectory sent successfully with " << trajectory.reference.points.size() << " points and " << trajectory.idxs.size() << " waypoints.");

  return {true, "Trajectory sent successfully"};
}

/**
 * \brief Executes the given subtasks.
 *
 * This function processes and executes the subtasks associated with a trajectory waypoint.
 * It can handle different types of subtasks based on their type field.
 *
 * \param subtasks A vector of subtasks to be executed.
 */
// TODO implement the execution of subtasks, for now it just logs the subtasks that should be executed

void MissionHandler::createSubtasks(const std::vector<iroc_mission_handler::msg::Subtask> &subtasks) {
  // Create all subtasks at once
  bool success = subtask_manager_->createSubtasks(subtasks);
  if (!success) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to create subtasks");
    return;
  }

  // Start all subtasks
  success = subtask_manager_->startAllSubtasks();
  if (!success) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to start subtasks");
    return;
  }
}


double MissionHandler::distance(const mrs_msgs::msg::Reference &waypoint_1, const mrs_msgs::msg::Reference &waypoint_2) {
  using vec3_t = mrs_lib::geometry::vec_t<3>;

  return mrs_lib::geometry::dist(vec3_t(waypoint_1.position.x, waypoint_1.position.y, waypoint_1.position.z),
                                 vec3_t(waypoint_2.position.x, waypoint_2.position.y, waypoint_2.position.z));
}

void MissionHandler::updateMissionState(const mission_state_t &new_state) {
  if (mission_state_.value() == new_state) {
    return;
  }

  previous_mission_state_ = mission_state_.value();
  mission_state_.set(new_state);
  actionPublishFeedback();
}

void MissionHandler::resetMission() {
  std::scoped_lock lock(action_server_mutex_);

  current_trajectory_idx_          = 0;
  current_trajectory_waypoint_idx_ = 0;

  mission_waypoint_idx_ = 0;

  waypoint_metrics_.remaining_distance = 0.0;
  waypoint_metrics_.eta                = 0.0;
  waypoint_metrics_.progress           = 0.0;

  mission_metrics_.remaining_distance = 0.0;
  mission_metrics_.eta                = 0.0;
  mission_metrics_.progress           = 0.0;

  is_current_trajectory_finished_ = false;
  trajectories_.clear();
  actionPublishFeedback();
  RCLCPP_INFO(node_->get_logger(), "Mission reset successfully.");
}

template <typename ServiceType>
MissionHandler::result_t MissionHandler::callService(mrs_lib::ServiceClientHandler<ServiceType> &sc,
                                                     const std::shared_ptr<typename ServiceType::Request> &request) {

  auto response = sc.callSync(request);

  if (response) {
    if (response.value()->success) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "  with response \"" << response.value()->message << "\".");
      return {true, response.value()->message};
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "with response \"" << response.value()->message << "\".");
      return {false, response.value()->message};
    }
  } else {
    const std::string msg = std::string("Failed to call service ") + sc.getService() + ".";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, msg);
    return {false, msg};
  }
}

template <typename ServiceType>
MissionHandler::result_t MissionHandler::callService(mrs_lib::ServiceClientHandler<ServiceType> &sc,
                                                     const std::shared_ptr<typename ServiceType::Request> &request,
                                                     const std::shared_ptr<typename ServiceType::Response> &response) {
  auto temp_response = sc.callSync(request);

  if (temp_response) {
    if (temp_response.value()->success) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "  with response \"" << temp_response.value()->message << "\".");
      *response = *(temp_response.value());
      return {true, temp_response.value()->message};
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "with response \"" << temp_response.value()->message << "\".");
      *response = *(temp_response.value());
      return {false, temp_response.value()->message};
    }
  } else {
    const std::string msg = std::string("Failed to call service ") + sc.getService() + ".";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, msg);
    return {false, msg};
  }
}

} // namespace iroc_mission_handler
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(iroc_mission_handler::MissionHandler)
