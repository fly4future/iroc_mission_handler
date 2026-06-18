#pragma once
/**
 * \file mission_handler.hpp
 * \brief Per-robot mission executor with state machine and subtask plugin support.
 *
 * MissionHandler is a ROS 2 composable component that manages the full lifecycle of
 * a single-robot mission: trajectory generation, path following, subtask execution,
 * pause/resume, and return-to-home or land on completion.
 *
 * State machine: IDLE -> TAKEOFF -> MISSION_LOADED -> EXECUTING -> EXECUTING_SUBTASK -> FINISHED -> LAND/RTH
 *                                                      ^                                     |
 *                                                      |---------- PAUSED <------------------|
 *
 * Key design decisions:
 * - Missions are decomposed into trajectory segments, each with optional subtasks.
 * - Subtasks are managed via a pluginlib-based SubtaskManager supporting parallel or sequential execution.
 * - Replan on resume: when a paused mission resumes, the trajectory is regenerated from the current position.
 */

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
#include <mrs_msgs/msg/state.hpp>

/* MRS diagnostics */
#include <mrs_robot_diagnostics/enums/uav_state.hpp>
#include <mrs_robot_diagnostics/enums/helpers/enum_helpers.hpp>

/* IROC */
#include <iroc_mission_handler/action/mission.hpp>
#include <iroc_mission_handler/msg/subtask.hpp>
#include <iroc_mission_handler/msg/waypoint.hpp>
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

/**
 * \brief Per-robot mission executor (ROS 2 composable component).
 *
 * Receives mission goals from the fleet manager via an action server, generates
 * trajectories from waypoints, sends them to the MRS control pipeline, manages
 * subtask execution at waypoints, and reports progress via action feedback.
 */
class MissionHandler : public mrs_lib::Node {
public:
  MissionHandler(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;   ///< Callback group for subscribers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;      ///< Callback group for service servers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;      ///< Callback group for service clients.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;  ///< Callback group for timers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_action_;  ///< Callback group for action server.

  // | --------------------- Types and structs --------------------- |

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

  // | --------------------- State tracking --------------------- |

  typedef mrs_robot_diagnostics::state_t state_t;
  enum_helpers::enum_updater<state_t> uav_state_;           ///< Tracks the current UAV state (from MRS diagnostics).
  enum_helpers::enum_updater<mission_state_t> mission_state_; ///< Tracks the current mission state machine state.
  mission_state_t                             previous_mission_state_ = mission_state_t::IDLE;

  std::string      robot_name_;                  ///< Name of the robot this handler manages.
  std::atomic_bool is_initialized_ = false;
  double           _min_distance_threshold_;     ///< Minimum distance to consider a segment as valid movement (not just heading change).
  double           _trajectory_sampling_period_;  ///< Sampling period for trajectory generation.
  double           _takeoff_timeout_s_;          ///< Max seconds to wait for hover after takeoff call.

  rclcpp::Time takeoff_started_at_; ///< Timestamp of the most recent takeoff service call.

  // | -------------------- Subtask management ------------------- |

  std::unique_ptr<SubtaskManager> subtask_manager_; ///< Manages subtask plugin lifecycle (load, start, monitor, stop).

  // | ---------------------- ROS subscribers --------------------- |

  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::State>                     sh_state_;               ///< UAV state subscriber.
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_; ///< Control manager diagnostics subscriber.

  /** \brief Callback for control manager diagnostics; updates trajectory tracking state. */
  void controlManagerDiagCallback(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);

  // | ----------------------- ROS service clients ---------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_takeoff_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_land_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_land_home_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>                    sc_path_;                    ///< Send path to MRS path follower.
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>                 sc_get_path_;                ///< Get planned path from MRS planner.
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_hover_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_flying_to_start_; ///< Notify MRS: flying to mission start point.
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_start_;           ///< Notify MRS: mission execution started.
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                    sc_mission_pause_;           ///< Notify MRS: mission paused.
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray>     sc_mission_validation_;      ///< Validate trajectory references are reachable.
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>     sc_trajectory_reference_;    ///< Send trajectory to MRS tracker.
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv>      sc_transform_reference_;     ///< Transform a single reference between frames.
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceArraySrv> sc_transform_reference_array_; ///< Transform an array of references between frames.

  // | ----------------------- ROS service servers ---------------------- |

  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                      ss_activation_;      ///< Service to activate/resume mission execution.
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>                      ss_pausing_;         ///< Service to pause mission execution.
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UploadMissionSrv> ss_upload_mission_;  ///< Service to upload (stage) a mission.
  mrs_lib::ServiceServerHandler<iroc_mission_handler::srv::UnloadMissionSrv> ss_unload_mission_;  ///< Service to unload a staged mission.

  std::atomic<bool> is_mission_staged_{false}; ///< True when a mission has been uploaded but not yet executing.

  /** \brief Handles mission activation requests. Transitions from MISSION_LOADED to EXECUTING. */
  bool missionActivationServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /** \brief Handles mission pause requests. Transitions from EXECUTING to PAUSED. */
  bool missionPausingServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /** \brief Handles mission upload: validates subtasks, generates trajectories, and stages the mission. */
  bool uploadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Request>  request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UploadMissionSrv::Response> response);

  /** \brief Handles mission unload: resets staged mission state back to IDLE. */
  bool unloadMissionServiceCallback(const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Request>  request,
                                    const std::shared_ptr<iroc_mission_handler::srv::UnloadMissionSrv::Response> response);

  // | ----------------------- Timers ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  /**
   * \brief Main state machine loop.
   * Handles state transitions: IDLE, TAKEOFF (waits for hover), MISSION_LOADED,
   * EXECUTING (monitors trajectory progress), EXECUTING_SUBTASK, FINISHED, LAND/RTH.
   */
  void                       timerMain();

  std::shared_ptr<TimerType> timer_feedback_;
  /** \brief Publishes mission progress feedback to the action server. */
  void                       timerFeedback();

  /** \brief Loads configuration, creates subscribers, service clients/servers, timers, action server, and SubtaskManager. */
  void initialize(void);
  /** \brief Graceful shutdown handler. */
  void shutdown();

  // | ----------------------- Action server ----------------------- |

  rclcpp_action::Server<Mission>::SharedPtr action_server_ptr_;   ///< Per-robot Mission action server.
  std::shared_ptr<GoalHandleMission>        current_goal_handle_; ///< Currently active mission goal.
  std::recursive_mutex                      action_server_mutex_;

  /** \brief Collects current metrics and publishes feedback to the action client. */
  void actionPublishFeedback();

  /** \brief Validates an incoming mission goal. Accepts if initialized and no mission is active. */
  rclcpp_action::GoalResponse   handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Mission::Goal> goal);

  /** \brief Stores the goal handle and begins mission execution (triggers takeoff if needed). */
  void                          handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle);

  /** \brief Handles cancel requests; transitions to hover and resets mission state. */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission> goal_handle);

  // | ----------------------- Mission execution state ----------------------- |

  std::vector<trajectory_t> trajectories_;                              ///< All trajectory segments for the current mission.
  size_t                    current_trajectory_idx_          = 0;       ///< Index of the trajectory segment currently being executed.
  size_t                    current_trajectory_waypoint_idx_ = 0;       ///< Index of the current waypoint within the active trajectory.

  std::atomic_bool is_current_trajectory_finished_ = false; ///< Set by ControlManagerDiag callback when tracker finishes.
  std::atomic_bool is_trajectory_sent_             = false; ///< True after trajectory has been sent to the controller.

  int mission_waypoint_idx_ = 0; ///< Global index of the current waypoint being followed across all segments.

  metrics_t mission_metrics_;  ///< Metrics for the overall mission (distance, ETA, progress).
  metrics_t waypoint_metrics_; ///< Metrics for the current waypoint segment.

  double mission_progress_before_pause_ = 0.0; ///< Cached progress percentage before a pause, used for smooth resume.

  // | ----------------------- Mission management methods ----------------------- |

  /**
   * \brief Creates a mission from the action goal: generates path segments, computes trajectories, and validates them.
   * \return result_t indicating success or failure with message.
   */
  result_t createMission(const std::shared_ptr<const Mission::Goal> goal);

  /**
   * \brief Replans the mission from the current position after a pause/resume.
   * Regenerates remaining trajectory segments from the UAV's current location.
   * \return True if replanning succeeded.
   */
  bool replanMission();

  /** \brief Resets all mission state (trajectories, indices, metrics) back to defaults. */
  void resetMission();

  // | ----------------------- Trajectory management ----------------------- |

  /**
   * \brief Sends a trajectory to the MRS trajectory tracker via service call.
   * \return result_t indicating success or failure.
   */
  result_t sendTrajectoryToController(const trajectory_t &trajectory);

  /** \brief Creates subtask executor instances for the given subtask definitions via SubtaskManager. */
  void createSubtasks(const std::vector<iroc_mission_handler::msg::Subtask> &subtasks);

  /**
   * \brief Validates that all trajectory reference points are reachable within the safety area.
   * \return result_t indicating validation success or failure.
   */
  result_t validateTrajectory(const trajectory_t &trajectory);

  /**
   * \brief Segments a path into individual path_segment_t based on waypoint subtask boundaries.
   * Each segment ends at a waypoint that has subtasks attached.
   */
  std::vector<path_segment_t> segmentPath(const mrs_msgs::msg::Path &msg, const std::vector<iroc_mission_handler::msg::Waypoint> &waypoints);

  /**
   * \brief Generates a sampled heading trajectory from a path with period T.
   * Interpolates heading between waypoints to create smooth heading transitions.
   * \return Tuple of (reference points with headings, corresponding trajectory indices).
   */
  std::tuple<std::vector<mrs_msgs::msg::Reference>, std::vector<long int>> generateHeadingTrajectory(const mrs_msgs::msg::Path &path, double T);

  /**
   * \brief Converts path segments into trajectory_t objects ready for the controller.
   * Calls generateHeadingTrajectory() and validates each resulting trajectory.
   * \return Tuple of (result, vector of trajectories).
   */
  std::tuple<result_t, std::vector<trajectory_t>> generateTrajectoriesFromSegments(const std::vector<path_segment_t> &path_segments);

  // | ----------------------- Utility methods ----------------------- |

  /** \brief Computes Euclidean distance between two Reference points (3D position). */
  double distance(const mrs_msgs::msg::Reference &waypoint_1, const mrs_msgs::msg::Reference &waypoint_2);

  /** \brief Updates the mission state and logs the transition. */
  void updateMissionState(const mission_state_t &new_state);

  // | ----------------------- Service call helpers ----------------------- |

  /** \brief Convenience wrapper around iroc_common::callService() (fire-and-forget variant). */
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  /** \brief Convenience wrapper around iroc_common::callService() (response-capturing variant). */
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                       const std::shared_ptr<typename ServiceType::Response> &response);
};

} // namespace iroc_mission_handler
