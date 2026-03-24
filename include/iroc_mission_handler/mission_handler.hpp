#pragma once

/* ROS */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/* mrs_lib */
#include <mrs_lib/node.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

/* ROS services */
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/srv/get_path_srv.hpp>
#include <mrs_msgs/srv/validate_reference_array.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>
#include <mrs_msgs/srv/transform_reference_srv.hpp>
#include <mrs_msgs/srv/transform_reference_array_srv.hpp>

/* ROS messages */
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>

/* MRS diagnostics */
#include <mrs_robot_diagnostics/enums/uav_state.h>
#include <mrs_robot_diagnostics/enums/enum_helpers.h>

/* IROC */
#include <iroc_mission_handler/action/mission.hpp>
#include <iroc_mission_handler/srv/upload_mission_srv.hpp>
#include <iroc_mission_handler/srv/unload_mission_srv.hpp>
#include "iroc_mission_handler/enums/mission_state.h"
#include "iroc_mission_handler/subtask_manager.h"

#include <iroc_common/result.h>

/* STL */
#include <atomic>
#include <mutex>
#include <vector>

namespace iroc_mission_handler
{

// Type aliases for better readability
using Mission           = iroc_mission_handler::action::Mission;
using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;

class MissionHandler : public mrs_lib::Node {
public:
  MissionHandler(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;


  // | --------------------- types and structs --------------------- |
  // Use shared result_t from iroc_common
  using result_t = iroc_common::result_t;

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
    mrs_msgs::msg::Path                             path;
    bool                                            is_valid;
    std::vector<iroc_mission_handler::msg::Subtask> subtasks;
    bool                                            parallel_execution = false;
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
    mrs_msgs::msg::TrajectoryReference              reference;
    std::vector<long int>                           idxs;
    std::vector<iroc_mission_handler::msg::Subtask> subtasks;
    bool                                            parallel_execution = false;
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
  enum_helpers::enum_updater<state_t> uav_state_;
  enum_helpers::enum_updater<mission_state_t> mission_state_;
  mission_state_t                             previous_mission_state_ = mission_state_t::IDLE;

  std::string      robot_name_;
  std::atomic_bool is_initialized_ = false;
  double           _min_distance_threshold_; // Minimum distance to consider a segment as valid (not just a heading change)
  double           _trajectory_sampling_period_;
  double           _takeoff_timeout_s_; // Max seconds to wait for the UAV to reach hover after a takeoff call

  rclcpp::Time takeoff_started_at_; // Timestamp of the most recent takeoff service call

  // | -------------------- subtask management ------------------- |
  std::unique_ptr<SubtaskManager> subtask_manager_;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::State>                     sh_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  void controlManagerDiagCallback(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);

  // | ----------------------- ROS services ---------------------- |
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_takeoff_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_land_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_land_home_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>                    sc_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>                 sc_get_path_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_hover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_flying_to_start_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_start_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_pause_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray>     sc_mission_validation_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>     sc_trajectory_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv>      sc_transform_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceArraySrv> sc_transform_reference_array_;

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                      ss_activation_;
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                      ss_pausing_;
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UploadMissionSrv> ss_upload_mission_;
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UnloadMissionSrv> ss_unload_mission_;

  std::atomic<bool> is_mission_staged_{false};

  bool missionActivationServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool missionPausingServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool uploadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Request>  request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Response> response);
  bool unloadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Request>  request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Response> response);

  // | ----------------------- main timer ----------------------- |
  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();

  std::shared_ptr<TimerType> timer_feedback_;
  void                       timerFeedback();

  void initialize(void);
  void shutdown();

  // Action server
  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;
  std::shared_ptr<GoalHandleMission>        current_goal_handle_;
  std::recursive_mutex                      action_server_mutex_;

  void actionPublishFeedback();
  // Action server callbacks
  rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);
  void                          handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  // | --------------------- mission feedback and trajectory t-------------------- |
  std::vector<trajectory_t> trajectories_;
  size_t                    current_trajectory_idx_          = 0; // Index of the current trajectory being executed
  size_t                    current_trajectory_waypoint_idx_ = 0; // Index of the current waypoint in the current trajectory

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
  bool     replanMission();
  void     resetMission();

  // Trajectory management functions
  result_t sendTrajectoryToController(const trajectory_t &trajectory);
  void     createSubtasks(const std::vector<iroc_mission_handler::msg::Subtask> &subtasks);

  result_t                    validateTrajectory(const trajectory_t &trajectory);
  std::vector<path_segment_t> segmentPath(const mrs_msgs::msg::Path &msg, const std::vector<iroc_mission_handler::msg::Waypoint> &waypoints);
  std::tuple<std::vector<mrs_msgs::msg::Reference>, std::vector<long int>> generateHeadingTrajectory(const mrs_msgs::msg::Path &path, double T);
  std::tuple<result_t, std::vector<trajectory_t>>                          generateTrajectoriesFromSegments(const std::vector<path_segment_t> &path_segments);

  // Miscellaneous functions
  double distance(const mrs_msgs::msg::Reference &waypoint_1, const mrs_msgs::msg::Reference &waypoint_2);
  void   updateMissionState(const mission_state_t &new_state);

  // Call service methods overloads
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                       const std::shared_ptr<typename ServiceType::Response> &response);
};

} // namespace iroc_mission_handler
