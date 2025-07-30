/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <actionlib/server/simple_action_server.h>
#include <iroc_mission_handler/MissionAction.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/ValidateReferenceArray.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TransformReferenceSrv.h>
#include <mrs_msgs/TransformReferenceArraySrv.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_robot_diagnostics/enums/uav_state.h>
#include <mrs_robot_diagnostics/enums/tracker_state.h>
#include <mrs_robot_diagnostics/enums/enum_helpers.h>

#include <mrs_robot_diagnostics/UavState.h>

#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

#include <atomic>
#include <tuple>
#include <mutex>

#include "iroc_mission_handler/enums/mission_state.h"
#include "iroc_mission_handler/subtask_manager.h"

//}

namespace iroc_mission_handler {

/* class MissionHandler //{ */

class MissionHandler : public nodelet::Nodelet {
 public:
  virtual void onInit();

 private:
  ros::NodeHandle nh_;

  // | --------------------- types and structs --------------------- |
  struct result_t {
    bool success;
    std::string message;
  };

  /**
   * \brief Struct to hold path segments.
   *
   * This struct contains a path, a validity flag, and a vector of subtasks.
   * - `path`: The path segment.
   * - `is_valid`: A boolean indicating whether the path segment is a valid movement path (true) or just heading changes (false).
   * - `subtasks`: A vector of subtasks that will be executed at the end of the path segment.
   */
  struct path_segment_t {
    mrs_msgs::Path path;
    bool is_valid;
    std::vector<iroc_mission_handler::Subtask> subtasks;
  };

  /**
   * \brief Struct to hold trajectory information and track subtasks.
   *
   * This struct contains a trajectory reference, a vector of trajectory indices, and a vector of subtasks.
   * - `reference`: The trajectory reference.
   * - `idxs`: A vector of indices corresponding to the trajectory points indices (each index corresponds to a point in the trajectory).
   * - `subtasks`: A vector of subtasks associated with the trajectory.
   */
  struct trajectory_t {
    mrs_msgs::TrajectoryReference reference;
    std::vector<long int> idxs;
    std::vector<iroc_mission_handler::Subtask> subtasks;
  };

  /**
   * \brief Struct to hold metrics for the mission handler.
   *
   * - `remaining_distance`: The remaining distance to the mission finish point.
   * - `eta`: The estimated time of arrival to the mission end.
   * - `progress`: The overall mission progress as a percentage (0.0 - 100.0).
   */
  struct metrics_t {
    double remaining_distance = 0.0;
    double eta = 0.0;
    double progress = 0.0;
  };

  typedef mrs_robot_diagnostics::uav_state_t uav_state_t;
  enum_helpers::enum_updater<uav_state_t> uav_state_ = {"UAV STATE", uav_state_t::UNKNOWN};
  enum_helpers::enum_updater<mission_state_t> mission_state_ = {"MISSION STATE", mission_state_t::IDLE};
  mission_state_t previous_mission_state_ = mission_state_t::IDLE;

  std::string robot_name_;
  std::atomic_bool is_initialized_ = false;
  double _min_distance_threshold_; // Minimum distance to consider a segment as valid (not just a heading change)
  double _trajectory_sampling_period_;

  // | -------------------- subtask management ------------------- |
  std::unique_ptr<SubtaskManager> subtask_manager_;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState> sh_uav_state_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

  // | ----------------------- ROS services ---------------------- |
  ros::ServiceClient sc_takeoff_;
  ros::ServiceClient sc_land_;
  ros::ServiceClient sc_land_home_;
  ros::ServiceClient sc_path_;
  ros::ServiceClient sc_get_path_;
  ros::ServiceClient sc_hover_;
  ros::ServiceClient sc_mission_flying_to_start_;
  ros::ServiceClient sc_mission_start_;
  ros::ServiceClient sc_mission_pause_;
  ros::ServiceClient sc_mission_validation_;
  ros::ServiceClient sc_trajectory_reference_;
  ros::ServiceClient sc_transform_reference_;
  ros::ServiceClient sc_transform_reference_array_;

  ros::ServiceServer ss_activation_;
  ros::ServiceServer ss_pausing_;

  bool missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool missionPausingServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | ----------------------- main timer ----------------------- |
  ros::Timer timer_main_;
  ros::Timer timer_feedback_;
  void timerMain(const ros::TimerEvent& event);
  void timerFeedback(const ros::TimerEvent& event);

  // | --------------------- actionlib stuff -------------------- |
  typedef actionlib::SimpleActionServer<iroc_mission_handler::MissionAction> MissionHandlerActionServer;
  std::unique_ptr<MissionHandlerActionServer> action_server_ptr_;

  void actionCallbackGoal();
  void actionCallbackPreempt();
  void actionPublishFeedback();

  typedef iroc_mission_handler::MissionGoal ActionServerGoal;
  ActionServerGoal action_server_goal_;
  std::recursive_mutex action_server_mutex_;

  // | --------------------- mission feedback and trajectory t-------------------- |
  std::vector<trajectory_t> trajectories_;
  int current_trajectory_idx_ = 0;          // Index of the current trajectory being executed
  int current_trajectory_waypoint_idx_ = 0; // Index of the current waypoint in the current trajectory

  std::atomic_bool is_current_trajectory_finished_ = false;

  // Waypoint information (which waypoint is currently being followed)
  int mission_waypoint_idx_ = 0; // Index of the current waypoint being followed

  metrics_t mission_metrics_;  // Metrics for the current mission
  metrics_t waypoint_metrics_; // Metrics for the current waypoint

  // Trajectory sampling period (it is to compute the mission metrics because the trajectory is sampled at this period)
  double mission_progress_before_pause_ = 0.0; // Progress before the mission was paused

  // | ------------------ Additional functions ------------------ |
  result_t createMission(const ActionServerGoal& action_server_goal);
  bool replanMission();
  void resetMission();

  // Trajectory management functions
  result_t sendTrajectoryToController(const trajectory_t& trajectory);
  void createSubtasks(const std::vector<iroc_mission_handler::Subtask>& subtasks);

  result_t validateTrajectory(const trajectory_t& trajectory);
  std::vector<path_segment_t> segmentPath(const mrs_msgs::Path& msg, const std::vector<std::vector<Subtask>>& waypoint_subtasks = {});
  std::tuple<std::vector<mrs_msgs::Reference>, std::vector<long int>> generateHeadingTrajectory(const mrs_msgs::Path& path, double T);
  std::tuple<result_t, std::vector<trajectory_t>> generateTrajectoriesFromSegments(const std::vector<path_segment_t>& path_segments);

  // Miscellaneous functions
  double distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2);
  std::tuple<bool, mrs_msgs::ReferenceArray> transformReferenceArray(mrs_msgs::TransformReferenceArraySrv transformArraySrv);
  void updateMissionState(const mission_state_t& new_state);

  // Call service methods overloads
  template <typename ServiceType>
  result_t callService(ros::ServiceClient& sc, typename ServiceType::Request req, typename ServiceType::Response& res);

  template <typename ServiceType>
  result_t callService(ros::ServiceClient& sc, typename ServiceType::Request req = {});

  result_t callService(ros::ServiceClient& sc, const bool val);
};
//}

/* onInit() //{ */

void MissionHandler::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // Load configuration files
  mrs_lib::ParamLoader param_loader(nh_, "MissionHandler");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("robot_name", robot_name_);

  param_loader.addYamlFileFromParam("trajectory_generation_config");

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  // Load parameters
  const auto main_timer_rate = param_loader.loadParam2<double>("mission_handler/main_timer_rate");
  const auto feedback_timer_rate = param_loader.loadParam2<double>("mission_handler/feedback_timer_rate");

  _min_distance_threshold_ = param_loader.loadParam2<double>("mrs_uav_trajectory_generation/min_waypoint_distance");
  _trajectory_sampling_period_ = param_loader.loadParam2<double>("mrs_uav_trajectory_generation/sampling_dt");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MissionHandler]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |
  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));

  mrs_lib::SubscribeHandlerOptions sh_opts;
  sh_opts.nh = nh_;
  sh_opts.node_name = "MissionHandler";
  sh_opts.no_message_timeout = ros::Duration(5.0);
  sh_opts.timeout_manager = tim_mgr_;
  sh_opts.threadsafe = true;
  sh_opts.autostart = true;
  sh_opts.queue_size = 10;
  sh_opts.transport_hints = ros::TransportHints().tcpNoDelay();

  sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState>(sh_opts, "in/uav_state");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(sh_opts, "control_manager_diagnostics_in",
                                                                                            &MissionHandler::controlManagerDiagCallback, this);

  // | --------------------- service clients -------------------- |
  sc_takeoff_ = nh_.serviceClient<std_srvs::Trigger>("svc/takeoff");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/offboard\' -> \'%s\'", sc_takeoff_.getService().c_str());

  sc_land_ = nh_.serviceClient<std_srvs::Trigger>("svc/land");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land\' -> \'%s\'", sc_land_.getService().c_str());

  sc_land_home_ = nh_.serviceClient<std_srvs::Trigger>("svc/land_home");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land_home\' -> \'%s\'", sc_land_home_.getService().c_str());

  sc_path_ = nh_.serviceClient<mrs_msgs::PathSrv>("svc/path");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/path\' -> \'%s\'", sc_path_.getService().c_str());

  sc_get_path_ = nh_.serviceClient<mrs_msgs::GetPathSrv>("svc/get_path");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/get_path\' -> \'%s\'", sc_get_path_.getService().c_str());

  sc_hover_ = nh_.serviceClient<std_srvs::Trigger>("svc/hover");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/hover\' -> \'%s\'", sc_hover_.getService().c_str());

  sc_mission_flying_to_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_flying_to_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_flying_to_start\' -> \'%s\'", sc_mission_flying_to_start_.getService().c_str());

  sc_mission_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_start\' -> \'%s\'", sc_mission_start_.getService().c_str());

  sc_mission_pause_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_pause");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_pause\' -> \'%s\'", sc_mission_pause_.getService().c_str());

  sc_mission_validation_ = nh_.serviceClient<mrs_msgs::ValidateReferenceArray>("svc/mission_validation");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_validation\' -> \'%s\'", sc_mission_validation_.getService().c_str());

  sc_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("svc/trajectory_reference_out");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/trajectory_reference_out\' -> \'%s\'", sc_trajectory_reference_.getService().c_str());

  sc_transform_reference_ = nh_.serviceClient<mrs_msgs::TransformReferenceSrv>("svc/transform_reference");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/transform_reference\' -> \'%s\'", sc_transform_reference_.getService().c_str());

  sc_transform_reference_array_ = nh_.serviceClient<mrs_msgs::TransformReferenceArraySrv>("svc/transform_reference_array");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/transform_reference_array\' -> \'%s\'", sc_transform_reference_.getService().c_str());

  // | --------------------- service servers -------------------- |
  ss_activation_ = nh_.advertiseService("svc_server/mission_activation", &MissionHandler::missionActivationServiceCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_activation\' -> \'%s\'", ss_activation_.getService().c_str());

  ss_pausing_ = nh_.advertiseService("svc_server/mission_pausing", &MissionHandler::missionPausingServiceCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_pausing\' -> \'%s\'", ss_pausing_.getService().c_str());

  // | ------------------------- timers ------------------------- |
  timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &MissionHandler::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &MissionHandler::timerFeedback, this);

  // | ------------------ action server methods ----------------- |
  action_server_ptr_ = std::make_unique<MissionHandlerActionServer>(nh_, ros::this_node::getName(), false);
  action_server_ptr_->registerGoalCallback(boost::bind(&MissionHandler::actionCallbackGoal, this));
  action_server_ptr_->registerPreemptCallback(boost::bind(&MissionHandler::actionCallbackPreempt, this));
  action_server_ptr_->start();

  // | -------------------- subtask manager -------------------- |
  CommonHandlers common_handlers = {
      .nh = nh_,
      .sh_opts = sh_opts,
  };
  subtask_manager_ = std::make_unique<SubtaskManager>(common_handlers);

  // | --------------------- finish the init -------------------- |
  ROS_INFO("[MissionHandler]: initialized");
  ROS_INFO("[MissionHandler]: --------------------");
  is_initialized_ = true;
}

//}

// | ------------------------- timers  ------------------------ |

/* timerMain() //{ */

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
void MissionHandler::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  std::scoped_lock lock(action_server_mutex_);
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionHandler]: Waiting for nodelet initialization");
    return;
  }

  // | -------------------- UAV state parsing ------------------- |
  if (sh_uav_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<uav_state_t>(sh_uav_state_.getMsg()->state));
  }

  // |-----------------------------------------------------------|
  // |                   State machine logic                     |
  // |-----------------------------------------------------------|
  if (!action_server_ptr_->isActive()) {
    return;
  }

  // Detect landing state and update mission state accordingly
  const bool not_idle_or_land = mission_state_.value() != mission_state_t::IDLE && mission_state_.value() != mission_state_t::LAND;
  if (uav_state_.value() == uav_state_t::LAND && not_idle_or_land) {
    ROS_WARN_STREAM("[MissionHandler]: Landing detected. Switching to LAND state.");
    updateMissionState(mission_state_t::LAND);
    return;
  }

  // Check for RC mode during active missions
  if (uav_state_.value() == uav_state_t::RC_MODE) {
    if (mission_state_.value() != mission_state_t::PAUSED_DUE_TO_RC_MODE) {
      ROS_INFO_STREAM_THROTTLE(1.0,
                               "[MissionHandler]: Mission is pause due to active MRS Remote mode. Disable the mode, to continue with the mission execution.");
      updateMissionState(mission_state_t::PAUSED_DUE_TO_RC_MODE);
    }
    return;
  }

  // Check for manual control during active missions
  if (uav_state_.value() == uav_state_t::MANUAL) {
    iroc_mission_handler::MissionResult action_server_result;
    action_server_result.name = robot_name_;
    action_server_result.success = false;
    action_server_result.message = "Mission cancelled because drone is under manual control.";

    ROS_INFO("[MissionHandler]: %s", action_server_result.message.c_str());
    action_server_ptr_->setAborted(action_server_result);

    updateMissionState(mission_state_t::IDLE);
    resetMission();
    return;
  }

  switch (mission_state_.value()) {
    case mission_state_t::EXECUTING: {
      if (current_trajectory_idx_ >= trajectories_.size()) {
        if (uav_state_.value() == uav_state_t::HOVER) { // Wait for the UAV currently executing trajectory to finish
          updateMissionState(mission_state_t::FINISHED);
          ROS_WARN_STREAM("[MissionHandler]: No more trajectories to execute. Current trajectory index: " << current_trajectory_idx_);
        }
        break;
      }

      if (!mrs_robot_diagnostics::is_flying(uav_state_.value())) {
        ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Calling takeoff");
        auto takeoff_res = callService<std_srvs::Trigger>(sc_takeoff_);
        if (!takeoff_res.success) {
          ROS_ERROR_THROTTLE(1.0, "[MissionHandler]: %s", takeoff_res.message.c_str());
          updateMissionState(mission_state_t::IDLE);
        }
        break;
      }

      // Check if the current trajectory is finished
      if (is_current_trajectory_finished_) {
        ROS_INFO_STREAM("[MissionHandler]: Finished tracking trajectory " << current_trajectory_idx_);

        if (!trajectories_[current_trajectory_idx_].subtasks.empty()) {
          ROS_INFO("[MissionHandler]: Executing subtasks in the waypoint %d ", mission_waypoint_idx_);
          createSubtasks(trajectories_[current_trajectory_idx_].subtasks);

          updateMissionState(mission_state_t::EXECUTING_SUBTASK);
        }

        // Move to next trajectory
        current_trajectory_idx_++;
        is_current_trajectory_finished_ = false;
        break;
      }

      // Send and start the trajectory
      if (uav_state_.value() == uav_state_t::HOVER) {
        ROS_INFO_STREAM("[MissionHandler]: Starting trajectory id " << current_trajectory_idx_ << ", total " << trajectories_.size());
        auto result = sendTrajectoryToController(trajectories_[current_trajectory_idx_]);
        if (!result.success) {
          ROS_WARN_STREAM("[MissionHandler]: Failed to send trajectory: " << result.message);

          iroc_mission_handler::MissionResult action_server_result;
          action_server_result.name = robot_name_;
          action_server_result.success = false;
          action_server_result.message = result.message;
          action_server_ptr_->setAborted(action_server_result);

          current_trajectory_idx_ = 0;
          updateMissionState(mission_state_t::IDLE);
          resetMission();
          return;
        }

        auto start_res = callService<std_srvs::Trigger>(sc_mission_start_);
        if (!start_res.success) {
          ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call mission start service: %s", start_res.message.c_str());
          updateMissionState(mission_state_t::MISSION_LOADED);
        }
        break;
      }

      break;
    }

    case mission_state_t::EXECUTING_SUBTASK: {
      if (subtask_manager_->areAllSubtasksCompleted()) {
        updateMissionState(mission_state_t::EXECUTING);
      }
      break;
    }

    case mission_state_t::FINISHED: {
      switch (action_server_goal_.terminal_action) {
        case ActionServerGoal::TERMINAL_ACTION_LAND: {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Executing terminal action. Calling land");
          auto resp = callService<std_srvs::Trigger>(sc_land_);
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call land service.");
            return;
          }

          updateMissionState(mission_state_t::LAND);
          break;
        }

        case ActionServerGoal::TERMINAL_ACTION_RTH: {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Executing terminal action. Calling land home");
          auto resp = callService<std_srvs::Trigger>(sc_land_home_);
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call land home service.");
            return;
          }

          updateMissionState(mission_state_t::RTH);
          break;
        }

        default: {
          iroc_mission_handler::MissionResult action_server_result;
          action_server_result.name = robot_name_;
          action_server_result.success = true;
          action_server_result.message = "Mission finished";
          ROS_INFO("[MissionHandler]: Mission finished.");
          action_server_ptr_->setSucceeded(action_server_result);

          updateMissionState(mission_state_t::IDLE);
          break;
        }
      }

      // Reset mission state and trajectory tracking
      resetMission();
      break;
    }

    case mission_state_t::LAND: {
      if (uav_state_.value() == uav_state_t::ARMED || uav_state_.value() == uav_state_t::DISARMED || uav_state_.value() == uav_state_t::OFFBOARD) {
        ROS_INFO_STREAM("[MissionHandler]: Landing finished.");

        iroc_mission_handler::MissionResult action_server_result;
        if (previous_mission_state_ == mission_state_t::FINISHED) {
          action_server_result.name = robot_name_;
          action_server_result.success = true;
          action_server_result.message = "Mission finished";

          ROS_INFO("[MissionHandler]: Mission finished.");
          action_server_ptr_->setSucceeded(action_server_result);
        } else {
          action_server_result.name = robot_name_;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped due to landing.";

          ROS_WARN("[MissionHandler]: Mission stopped due to landing.");
          action_server_ptr_->setAborted(action_server_result);
        }

        updateMissionState(mission_state_t::IDLE);
        resetMission();
      }

      break;
    }

    case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
      // mission continue if we are again not in RC_mode
      if (uav_state_.value() != uav_state_t::RC_MODE) {
        ROS_INFO("[MissionHandler]: RC mode disabled. Switching to previous mission mode");
        updateMissionState(previous_mission_state_);
      }
      break;
    }

    default:
      break;
  }
}
//}

/* timerFeedback() //{ */
void MissionHandler::timerFeedback([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionHandler]: Waiting for nodelet initialization");
    return;
  }
  actionPublishFeedback();
}
//}

// | ----------------- service server callback ---------------- |

/*  missionActivationServiceCallback()//{ */
bool MissionHandler::missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[MissionHandler]: Received mission activation request.");

  if (!action_server_ptr_->isActive()) {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
    return true;
  }

  switch (mission_state_.value()) {
    case mission_state_t::MISSION_LOADED: {
      ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Already flying. Starting mission with first trajectory.");
      is_current_trajectory_finished_ = false;
      updateMissionState(mission_state_t::EXECUTING);
      break;
    }

    case mission_state_t::PAUSED: {
      ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Replanning mission from current position");
      const auto replan_res = replanMission();
      if (!replan_res) {
        ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to replan mission.");
        res.success = false;
        res.message = "failed to replan mission";
        return true;
      } else {
        ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Replanning mission successfully.");
        updateMissionState(mission_state_t::MISSION_LOADED);
      }

      ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Resuming mission with current trajectory.");
      updateMissionState(mission_state_t::EXECUTING);
      break;
    }

    case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
      res.success = false;
      res.message = "Mission is paused due to active MRS Remote mode. Disable the mode to continue mission execution.";
      ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
      return true;
    }

    default: {
      res.success = false;
      res.message = "Mission is already activated.";
      ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
      return true;
    }
  }

  res.success = true;
  res.message = "Mission activated successfully.";
  return true;
}

//}

/*  missionPausingServiceCallback()//{ */
bool MissionHandler::missionPausingServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[MissionHandler]: Received mission pausing request.");
  if (!action_server_ptr_->isActive()) {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
  }

  mission_progress_before_pause_ = mission_metrics_.progress;
  switch (mission_state_.value()) {
    case mission_state_t::MISSION_LOADED: {
      ROS_INFO_STREAM("[MissionHandler]: Mission paused before execution.");
      res.success = true;
      res.message = "Mission paused before execution.";
      updateMissionState(mission_state_t::PAUSED);
      break;
    }

    case mission_state_t::EXECUTING: {
      ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Mission paused. Hover started.");
      auto resp = callService<std_srvs::Trigger>(sc_mission_pause_);
      res.success = resp.success;
      res.message = resp.message;
      if (!resp.success) {
        ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call stop trajectory tracking service.");
        break;
      }

      updateMissionState(mission_state_t::PAUSED);
      break;
    }

    default: {
      res.success = false;
      res.message = "Mission is in the state in which cannot be paused.";
      ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
      break;
    }
  }

  return true;
}
//}

// | ----------------- msg callback ---------------- |

/* controlManagerDiagCallback() //{ */

/**
 * \brief Callback function for ControlManagerDiagnostics messages.
 *
 * This function processes the diagnostics data to update the mission state, current trajectory, and waypoint information.
 * It calculates the distance to the next waypoint, estimated time of arrival (ETA), and progress towards the next waypoint.
 * It also checks if the current waypoint has been reached or if the current trajectory has been completed.
 *
 * \param diagnostics The ControlManagerDiagnostics message containing the current state of the control manager.
 */
void MissionHandler::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics) {
  std::scoped_lock lock(action_server_mutex_);

  if (!is_initialized_ ||                                       // Node initialization check
      !diagnostics || !diagnostics->tracker_status.have_goal || // Diagnostics check
      current_trajectory_idx_ >= trajectories_.size() ||        // Current trajectory index check
      mission_state_.value() != mission_state_t::EXECUTING) {   // Mission state check
    return;
  }

  // Get current state
  int current_point_idx = diagnostics->tracker_status.trajectory_idx;
  trajectory_t& current_trajectory = trajectories_.at(current_trajectory_idx_);

  int previous_waypoint_point_idx = current_trajectory_waypoint_idx_ > 0 ? current_trajectory.idxs[current_trajectory_waypoint_idx_ - 1] : 0;
  int next_waypoint_point_idx = current_trajectory.idxs[current_trajectory_waypoint_idx_];

  // | ----------------------- Check if current waypoint is reached ----------------------- |
  if (current_point_idx >= current_trajectory.idxs[current_trajectory_waypoint_idx_]) {
    ROS_INFO("[MissionHandler]: Reached %d waypoint in trajectory %d", current_trajectory_waypoint_idx_, current_trajectory_idx_);

    // Reached the current waypoint, update the mission state and indices
    mission_waypoint_idx_++;
    current_trajectory_waypoint_idx_++;

    if (current_trajectory_waypoint_idx_ >= current_trajectory.idxs.size()) {
      // If we reached the last waypoint in the trajectory, mark it as finished and reset the trajectory waypoint index
      ROS_INFO("[MissionHandler]: Reached last waypoint in trajectory %d", current_trajectory_idx_);
      is_current_trajectory_finished_ = true;
      current_trajectory_waypoint_idx_ = 0;
      return;
    }
  }

  // | ----------------------- Update waypoint metrics ----------------------- |
  const mrs_msgs::Reference current_position = current_trajectory.reference.points.at(current_point_idx);
  const mrs_msgs::Reference next_waypoint_position = current_trajectory.reference.points.at(next_waypoint_point_idx);

  // Number of points in the current path segment (waypoint to next waypoint)
  const int number_of_points = next_waypoint_point_idx - previous_waypoint_point_idx;
  double waypoint_progress = number_of_points > 0 ? (static_cast<double>(current_point_idx - previous_waypoint_point_idx) / number_of_points) * 100.0 : 0.0;

  waypoint_metrics_.remaining_distance = distance(current_position, next_waypoint_position);
  waypoint_metrics_.eta = std::max(static_cast<double>(next_waypoint_point_idx - current_point_idx) * _trajectory_sampling_period_, 0.0);
  waypoint_metrics_.progress = std::min(waypoint_progress, 100.0);

  // | ----------------------- Update mission metrics ----------------------- |
  double remaining_distance = 0.0;
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

      const mrs_msgs::Reference start_position = trajectories_[i].reference.points.at(j);
      const mrs_msgs::Reference end_position = trajectories_[i].reference.points.at(j + 1);

      remaining_distance += distance(start_position, end_position);
      remaining_points++;
    }
    remaining_points++; // Count the last point of the trajectory as well
  }
  double current_progress = static_cast<double>(total_num_points - remaining_points) / total_num_points * 100.0;

  mission_metrics_.progress = std::min(mission_progress_before_pause_ + current_progress * (1.0 - (mission_progress_before_pause_ / 100.0)), 100.0);
  mission_metrics_.remaining_distance = remaining_distance;
  mission_metrics_.eta = static_cast<double>(remaining_points) * _trajectory_sampling_period_;

  ROS_DEBUG_STREAM_THROTTLE(1.0, "[MissionHandler]: Current waypoint metrics: \n"
                                     << "  Remaining distance: " << waypoint_metrics_.remaining_distance << "\n"
                                     << "  ETA: " << waypoint_metrics_.eta << "\n"
                                     << "  Progress: " << waypoint_metrics_.progress << "%\n");
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[MissionHandler]: Mission metrics: \n"
                                     << "  Remaining distance: " << mission_metrics_.remaining_distance << "\n"
                                     << "  ETA: " << mission_metrics_.eta << "\n"
                                     << "  Progress: " << mission_metrics_.progress << "%\n");
}
//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */
void MissionHandler::actionCallbackGoal() {
  std::scoped_lock lock(action_server_mutex_);
  boost::shared_ptr<const iroc_mission_handler::MissionGoal> new_action_server_goal = action_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[MissionHandler]: Action server received a new goal: \n" << *new_action_server_goal);

  if (!is_initialized_) {
    iroc_mission_handler::MissionResult action_server_result;
    action_server_result.name = robot_name_;
    action_server_result.success = false;
    action_server_result.message = "Not initialized yet";
    ROS_WARN("[MissionHandler]: not initialized yet");
    action_server_ptr_->setAborted(action_server_result);
    return;
  }

  const auto result = createMission(*new_action_server_goal);

  if (!result.success) {
    iroc_mission_handler::MissionResult action_server_result;
    action_server_result.name = robot_name_;
    action_server_result.success = false;
    action_server_result.message = result.message;
    ROS_WARN("[MissionHandler]: mission aborted");
    action_server_ptr_->setAborted(action_server_result);
    return;
  }

  updateMissionState(mission_state_t::MISSION_LOADED);
  action_server_goal_ = *new_action_server_goal;
}
//}

/*  actionCallbackPreempt()//{ */
void MissionHandler::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (action_server_ptr_->isActive()) {
    if (action_server_ptr_->isNewGoalAvailable()) {
      ROS_INFO("[MissionHandler]: Preemption toggled for ActionServer.");
      iroc_mission_handler::MissionResult action_server_result;
      action_server_result.name = robot_name_;
      action_server_result.success = false;
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[MissionHandler]: " << action_server_result.message);
      action_server_ptr_->setPreempted(action_server_result);
      updateMissionState(mission_state_t::IDLE);
    } else {
      ROS_INFO("[MissionHandler]: Cancel toggled for ActionServer.");
      switch (mission_state_.value()) {
        case mission_state_t::EXECUTING: {
          ROS_INFO_STREAM_THROTTLE(1.0, "Drone is in the movement -> Calling hover.");
          auto resp = callService<std_srvs::Trigger>(sc_hover_);
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call hover service.");
          }
          iroc_mission_handler::MissionResult action_server_result;
          action_server_result.name = robot_name_;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped.";
          action_server_ptr_->setAborted(action_server_result);
          ROS_INFO("[MissionHandler]: Mission stopped.");
          updateMissionState(mission_state_t::IDLE);
          break;
        }

        default:
          iroc_mission_handler::MissionResult action_server_result;
          action_server_result.name = robot_name_;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped.";
          action_server_ptr_->setAborted(action_server_result);
          ROS_INFO("[MissionHandler]: Mission stopped.");
          updateMissionState(mission_state_t::IDLE);
          break;
      }
    }
  }
}
//}

/* actionPublishFeedback()//{ */
void MissionHandler::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);

  if (action_server_ptr_->isActive()) {
    iroc_mission_handler::MissionFeedback action_server_feedback;
    action_server_feedback.name = robot_name_;
    action_server_feedback.message = to_string(mission_state_.value());

    action_server_feedback.goal_idx = mission_waypoint_idx_;
    action_server_feedback.distance_to_closest_goal = waypoint_metrics_.remaining_distance;
    action_server_feedback.goal_estimated_arrival_time = waypoint_metrics_.eta;
    action_server_feedback.goal_progress = waypoint_metrics_.progress;

    action_server_feedback.distance_to_finish = mission_metrics_.remaining_distance;
    action_server_feedback.finish_estimated_arrival_time = mission_metrics_.eta;
    action_server_feedback.mission_progress = mission_metrics_.progress;

    action_server_ptr_->publishFeedback(action_server_feedback);
  }
}
//}

// | -------------------- support functions ------------------- |

/* createMission() //{ */
MissionHandler::result_t MissionHandler::createMission(const ActionServerGoal& action_server_goal) {
  std::stringstream ss;
  // Parameter validation
  if (!(action_server_goal.frame_id == ActionServerGoal::FRAME_ID_LOCAL || action_server_goal.frame_id == ActionServerGoal::FRAME_ID_LATLON ||
        action_server_goal.frame_id == ActionServerGoal::FRAME_ID_FCU)) {
    ss << "Unknown frame_id = \'" << int(action_server_goal.frame_id) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL || action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AMSL ||
        action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_FCU)) {
    ss << "Unknown height_id = \'" << int(action_server_goal.height_id) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE ||
        action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
        action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTH)) {
    ss << "Unknown terminal_action = \'" << int(action_server_goal.terminal_action) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }

  for (const auto& point : action_server_goal.points) {
    ROS_DEBUG("[MissionHandler]: Point: x:%f y:%f z:%f h:%f ", point.reference_point.position.x, point.reference_point.position.y,
              point.reference_point.position.z, point.reference_point.heading);
  }

  std::string frame_id;
  switch (action_server_goal.frame_id) {
    case ActionServerGoal::FRAME_ID_LOCAL: {
      frame_id = "local_origin";
      break;
    }

    case ActionServerGoal::FRAME_ID_LATLON: {
      frame_id = "latlon_origin";
      break;
    }

    case ActionServerGoal::FRAME_ID_FCU: {
      frame_id = "fcu_untilted";
      break;
    }

    default:
      break;
  }

  if (sh_uav_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<uav_state_t>(sh_uav_state_.getMsg()->state));
  }

  // Reject mission if fcu_frame is set and uav not flying
  if (action_server_goal.frame_id == ActionServerGoal::FRAME_ID_FCU) {
    if (uav_state_.value() != uav_state_t::HOVER) {
      ss << "FCU frame is set but uav is not in the air ";
      ROS_WARN_STREAM(ss.str());
      return {false, ss.str()};
    }
  }

  // Saving the AGL height points specified in the goal, this is needed as they will be
  // replaced after doing a transformation with latlon points
  std::vector<double> height_points;
  if (action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL) {
    for (const auto& point : action_server_goal.points) {
      height_points.push_back(point.reference_point.position.z);
    }
  }

  // Create reference array with received points to transform it into current control frame
  mrs_msgs::ReferenceArray goal_points_array;
  goal_points_array.header.frame_id = frame_id;
  goal_points_array.array.clear();
  goal_points_array.array.reserve(action_server_goal.points.size());
  for (const auto& point : action_server_goal.points) {
    goal_points_array.array.push_back(point.reference_point);
  }

  /* This could be replaced with TBD ControlManager Service "transformReferenceArray" */
  mrs_msgs::TransformReferenceArraySrv transformSrv_reference_array;
  transformSrv_reference_array.request.to_frame_id = "";
  transformSrv_reference_array.request.array = goal_points_array;

  auto [res, transformed_array] = transformReferenceArray(transformSrv_reference_array);
  if (!res) {
    ROS_WARN("[MissionHandler]: Failed while transforming the reference array!");
    return {false, "Reference array transformation failed"};
  }

  if (action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL && action_server_goal.frame_id != ActionServerGoal::FRAME_ID_FCU) {
    // Replacing the height points after the transformation, as when receiving LATLON points the transformation also considers the height as AMSL.
    for (size_t i = 0; i < transformed_array.array.size(); i++) {
      transformed_array.array.at(i).position.z = height_points.at(i);
    }
  }

  for (const auto& point : transformed_array.array) {
    ROS_DEBUG("[MissionHandler]: Transformed point x: %f  y: %f z: %f h: %f", point.position.x, point.position.y, point.position.z, point.heading);
  }

  mrs_msgs::Path msg_path;
  msg_path.points = transformed_array.array;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now = false;
  msg_path.use_heading = true;
  msg_path.dont_prepend_current_state = false; // do not use the current position for planning of the path
  msg_path.header.frame_id = transformed_array.header.frame_id;

  // Segmenting the path into segments based on subtasks and heading trajectories
  std::vector<std::vector<iroc_mission_handler::Subtask>> subtasks;
  subtasks.resize(msg_path.points.size());
  for (size_t i = 0; i < action_server_goal.points.size(); i++) {
    const auto& point = action_server_goal.points.at(i);
    if (point.subtasks.empty()) {
      continue;
    }
    subtasks.at(i) = point.subtasks;
  }
  std::vector<path_segment_t> path_segments = segmentPath(msg_path, subtasks);

  // Generating trajectory from the path segments
  auto [result, trajectories] = generateTrajectoriesFromSegments(path_segments);
  if (!result.success) {
    ROS_WARN_STREAM("Failed to get trajectory from segments: " << result.message);
    return {false, result.message};
  } else if (trajectories.empty()) {
    ROS_WARN_STREAM("No trajectories generated from segments.");
    return {false, "No trajectories generated from segments"};
  }

  for (const auto& trajectory : trajectories) {
    auto validation_result = validateTrajectory(trajectory);
    if (!validation_result.success) {
      ROS_WARN_STREAM("[MissionHandler]: Trajectory validation failed: " << validation_result.message);
      ROS_WARN_STREAM("Trajectory points outside of safety area!");
      return {false, validation_result.message};
    } else {
      ROS_DEBUG_STREAM("[MissionHandler]: Trajectory validation succeeded: " << validation_result.message);
    }
  }

  // Setting the mission information
  trajectories_ = trajectories;

  return {true, "Mission created successfully for " + robot_name_ + ", with " + std::to_string(trajectories.size()) + " trajectories."};
}
//}

/* validateTrajectory() //{ */

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
MissionHandler::result_t MissionHandler::validateTrajectory(const trajectory_t& trajectory) {
  // Create a ReferenceArray from the trajectory points for sending to the validation service
  mrs_msgs::ReferenceArray waypointArray;
  waypointArray.header = trajectory.reference.header;
  waypointArray.array = trajectory.reference.points;

  mrs_msgs::ValidateReferenceArray validateReferenceSrv;
  validateReferenceSrv.request.array = waypointArray;

  if (sc_mission_validation_.call(validateReferenceSrv)) {
    const bool all_success = std::all_of(validateReferenceSrv.response.success.begin(), validateReferenceSrv.response.success.end(), [](bool v) { return v; });
    if (all_success) {
      ROS_INFO_STREAM("Called service \"" << sc_mission_validation_.getService() << "\" with response \"" << validateReferenceSrv.response.message << "\".");
    } else {
      ROS_WARN_STREAM("Trajectory points outside of safety area, validation from  calling service \""
                      << sc_mission_validation_.getService() << "\" with response \"" << validateReferenceSrv.response.message << "\".");

      std::vector<mrs_msgs::Reference> invalid_points;
      for (auto& point_id : trajectory.idxs) {
        if (!validateReferenceSrv.response.success.at(point_id)) {
          invalid_points.push_back(trajectory.reference.points.at(point_id));
        }
      }

      for (auto& point : invalid_points) {
        ROS_WARN_STREAM("[MissionHandler]: Unvalid point: " << point);
      }

      if (invalid_points.size() == 0) {
        ROS_WARN("[MissionHandler]: The given points are valid, however the generated trajectory seems to be outside of safety area or within an obstacle.");
        return {false, "The given points are valid for: " + robot_name_ +
                           ", however the generated trajectory seems to be outside of safety area or within an obstacle."};
      } else {
        return {false, "Unvalid trajectory for " + robot_name_ + ", trajectory is outside of safety area"};
      }
    }
  }

  return {true, "Trajectory is valid for " + robot_name_ + ", sending to control manager"};
}
//}

/* segmentPath() //{ */

/**
 * \brief Segments the path into smaller segments based on the distance between points and subtasks.
 *
 * This function processes the input path message and segments it into smaller paths based on the distance between the consecutive points. If the distance is
 * less than a threshold (0.05), it marks the segment as invalid to be processed as a heading trajectory later. It also checks for subtasks at each waypoint to
 * determine if a new segment should be started.
 *
 * \param msg The input path message containing reference points.
 * \param waypoint_subtasks A vector of subtasks associated with each waypoint (it must match the size of the path points).
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
std::vector<MissionHandler::path_segment_t> MissionHandler::segmentPath(const mrs_msgs::Path& msg, const std::vector<std::vector<Subtask>>& waypoint_subtasks) {
  // Input validation
  if (msg.points.empty()) {
    ROS_WARN("[MissionHandler]: Empty path provided to segmentPath.");
    return {};
  }

  if (msg.points.size() != waypoint_subtasks.size()) {
    ROS_WARN("[MissionHandler]: Number of subtasks (%zu) does not match number of points in the path (%zu).", waypoint_subtasks.size(), msg.points.size());
    return {};
  }

  std::vector<path_segment_t> path_segments;

  path_segment_t current_segment;
  current_segment.path.header = msg.header;
  current_segment.path.fly_now = msg.fly_now;
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

    // Check if there are subtasks for the current point that require segment break
    if (!waypoint_subtasks.at(i).empty()) {
      current_segment.subtasks = waypoint_subtasks.at(i);
      path_segments.push_back(current_segment);

      // Reset the current segment for the next points
      current_segment.path.points.clear();
      current_segment.subtasks.clear();

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
  ROS_DEBUG_STREAM("[MissionHandler]: Path size: " << msg.points.size());
  ROS_DEBUG("[MissionHandler]: Path segments: %zu", path_segments.size());

  for (size_t segment_idx = 0; segment_idx < path_segments.size(); ++segment_idx) {
    ROS_DEBUG("[MissionHandler]: Segment %ld: %s", segment_idx + 1, path_segments[segment_idx].is_valid ? "Valid" : "Invalid");
    for (const auto& point : path_segments[segment_idx].path.points) {
      ROS_DEBUG("[MissionHandler]: Point: %f, %f, %f Heading: %f", point.position.x, point.position.y, point.position.z, point.heading);
    }
  }

  return path_segments;
}
//}

/* generateTrajectoriesFromSegments() //{ */

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
MissionHandler::generateTrajectoriesFromSegments(const std::vector<path_segment_t>& path_segments) {
  ros::Time trajectory_generation_start_time_ = ros::Time::now();

  std::vector<trajectory_t> trajectories;

  mrs_msgs::TrajectoryReference current_trajectory;
  std::vector<long int> current_trajectory_idxs;

  bool is_first_segment = true;
  for (auto segment : path_segments) {
    ROS_DEBUG("[MissionHandler]: Processing segment with %zu points", segment.path.points.size());

    if (!is_first_segment) {
      segment.path.dont_prepend_current_state = true; // Do not add current position to the trajectory
    } else {
      is_first_segment = false;
    }

    std::vector<mrs_msgs::Reference> points_to_add;
    std::vector<long int> idxs_to_add;

    // If the segment is invalid, we need to generate a heading trajectory otherwise we will call the getPath service
    if (segment.is_valid) {
      mrs_msgs::GetPathSrv getPathSrv;
      getPathSrv.request.path = segment.path;
      auto response = getPathSrv.response;

      // Calling getPath service
      auto result = callService<mrs_msgs::GetPathSrv>(sc_get_path_, getPathSrv.request, response);
      if (!result.success) {
        ROS_INFO_STREAM("Failed to call the service: \"" << sc_get_path_.getService());
        return {result_t{false, result.message}, {}};
      }

      // Copy the header of the trajectory
      current_trajectory.header = response.trajectory.header;
      current_trajectory.header.stamp = ros::Time::now();
      current_trajectory.input_id = response.trajectory.input_id;
      current_trajectory.use_heading = response.trajectory.use_heading;
      current_trajectory.fly_now = response.trajectory.fly_now;
      current_trajectory.loop = response.trajectory.loop;
      current_trajectory.dt = response.trajectory.dt;

      // Save the trajectory points and indices from the response
      points_to_add = response.trajectory.points;
      idxs_to_add = response.waypoint_trajectory_idxs;
    } else {
      std::tie(points_to_add, idxs_to_add) = generateHeadingTrajectory(segment.path, _trajectory_sampling_period_);
    }

    ROS_DEBUG("[MissionHandler]: Points to add: %zu, Indices to add: %zu", points_to_add.size(), idxs_to_add.size());
    if ((points_to_add.empty() || idxs_to_add.empty()) && !segment.subtasks.empty()) {
      ROS_ERROR("[MissionHandler]: No points or indices to add for the segment, skipping. %zu subtasks will not be executed.", segment.subtasks.size());
      return {result_t{false, "Empty trajectory was generated and subtasks could not be executed. Check the points in the path."}, {}};
    }

    // Add points to the current trajectory
    const long int points_size = current_trajectory.points.size(); // Save the size of points before appending new ones
    current_trajectory.points.insert(current_trajectory.points.end(), points_to_add.begin(), points_to_add.end());

    for (const auto& idx : idxs_to_add) { // Add trajectory idxs to vector
      auto idx_to_add = points_size + idx;
      current_trajectory_idxs.push_back(idx_to_add);
    }

    // If the segment has subtasks, we need to finalize the current trajectory (it will stop the mission while executing
    // the subtask if it is needed)
    if (segment.subtasks.size() > 0) {
      trajectory_t trajectory;
      trajectory.reference = current_trajectory;
      trajectory.idxs = current_trajectory_idxs;
      trajectory.subtasks = segment.subtasks;

      trajectories.push_back(trajectory);

      // Reset variables for the next trajectory
      current_trajectory = mrs_msgs::TrajectoryReference();
      current_trajectory_idxs.clear();
    }
  }

  // If there are still points in the current trajectory, we need to finalize it
  if (!current_trajectory.points.empty()) {
    trajectory_t trajectory;
    trajectory.reference = current_trajectory;
    trajectory.idxs = current_trajectory_idxs;

    trajectories.push_back(trajectory);
  }

  // Log the trajectory generation time and number of trajectories
  const double generation_time = (ros::Time::now() - trajectory_generation_start_time_).toSec();
  ROS_INFO("[MissionHandler]: Trajectory generation took: %f seconds", generation_time);
  ROS_DEBUG("[MissionHandler]: Number of trajectories generated: %zu", trajectories.size());

  for (size_t i = 0; i < trajectories.size(); ++i) {
    const auto& trajectory = trajectories[i];
    ROS_DEBUG("[MissionHandler]: Trajectory %zu has %zu points and %zu waypoints", i, trajectory.reference.points.size(), trajectory.idxs.size());
  }

  return {result_t{true, "Successfully generated trajectory"}, trajectories};
}
//}

/* generateHeadingTrajectory() //{ */

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
std::tuple<std::vector<mrs_msgs::Reference>, std::vector<long int>> MissionHandler::generateHeadingTrajectory(const mrs_msgs::Path& path, double T = 0.2) {
  using radians = mrs_lib::geometry::radians;
  using sradians = mrs_lib::geometry::sradians;

  std::vector<mrs_msgs::Reference> trajectory;
  std::vector<long int> trajectory_idxs;

  if (path.points.empty()) {
    return {trajectory, trajectory_idxs};
  }

  // Interpolation to the next points within segment
  mrs_msgs::Reference p0 = path.points[0];
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto& p1 = path.points[i];

    double h0 = p0.heading;
    double h1 = p1.heading;

    // Only interpolate heading if it's different
    if (std::abs(sradians::diff(h0, h1)) > 1e-6) {
      // Calculate number of samples for heading interpolation
      auto heading_diff = sradians::diff(h0, h1);
      ROS_DEBUG("[MissionHandler]: h1 %f, h2 %f diff:%f", h0, h1, heading_diff);
      // absolute value of heading diff
      int num_heading_samples = static_cast<int>(std::ceil(std::abs(heading_diff) / T));

      for (int j = 1; j <= num_heading_samples; ++j) {
        double t = static_cast<double>(j) / num_heading_samples;

        mrs_msgs::Reference point;
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
  for (const auto& point : trajectory) {
    ROS_DEBUG("[MissionHandler]: Trajectory point: %f, %f, %f Heading: %f", point.position.x, point.position.y, point.position.z, point.heading);
  }
  return {trajectory, trajectory_idxs};
}
//}

/* replanMission() //{ */

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

  ROS_WARN_STREAM("[MissionHandler]: Replanning trajectory " << current_trajectory_idx_ << ", current goal index: " << current_trajectory_waypoint_idx_
                                                             << ", waypoints " << trajectories_[current_trajectory_idx_].idxs.size() << ", points "
                                                             << trajectories_[current_trajectory_idx_].reference.points.size());

  std::vector<std::vector<Subtask>> remaining_subtasks;
  remaining_subtasks.resize(trajectories_[current_trajectory_idx_].idxs.size() - current_trajectory_waypoint_idx_);
  remaining_subtasks.back() = trajectories_[current_trajectory_idx_].subtasks; // Copy the last subtask to the last point

  std::vector<mrs_msgs::Reference> remaining_points;
  for (size_t i = current_trajectory_waypoint_idx_; i < trajectories_[current_trajectory_idx_].idxs.size(); i++) {
    remaining_points.push_back(trajectories_[current_trajectory_idx_].reference.points[trajectories_[current_trajectory_idx_].idxs[i]]);
    ROS_DEBUG("[MissionHandler]: Remaining point %zu: %f, %f, %f Heading: %f", i - current_trajectory_waypoint_idx_, remaining_points.back().position.x,
              remaining_points.back().position.y, remaining_points.back().position.z, remaining_points.back().heading);
  }

  mrs_msgs::Path remaining_path;
  remaining_path.points = remaining_points;
  remaining_path.header.stamp = ros::Time::now();
  remaining_path.fly_now = false;
  remaining_path.use_heading = true;
  remaining_path.dont_prepend_current_state = false; // Use the current position for planning of the path
  remaining_path.header.frame_id = trajectories_[current_trajectory_idx_].reference.header.frame_id;

  std::vector<path_segment_t> path_segments = segmentPath(remaining_path, remaining_subtasks);

  // Generating trajectory from the path segments
  auto [result, recomputed_trajectories] = generateTrajectoriesFromSegments(path_segments);
  if (!result.success) {
    ROS_WARN_STREAM("Failed to get trajectory from segments: " << result.message);
    return false;
  } else if (recomputed_trajectories.empty()) {
    ROS_WARN_STREAM("No trajectories generated from segments.");
    return false;
  }

  for (const auto& trajectory : recomputed_trajectories) {
    auto validation_result = validateTrajectory(trajectory);
    if (!validation_result.success) {
      ROS_WARN_STREAM("[MissionHandler]: Trajectory validation failed: " << validation_result.message);
      ROS_WARN_STREAM("Trajectory points outside of safety area!");
      return false;
    } else {
      ROS_DEBUG_STREAM("[MissionHandler]: Trajectory validation succeeded: " << validation_result.message);
    }
  }
  recomputed_trajectories.insert(recomputed_trajectories.end(), trajectories_.begin() + current_trajectory_idx_ + 1, trajectories_.end());

  // Setting the mission information
  trajectories_ = recomputed_trajectories;
  current_trajectory_waypoint_idx_ = 0; // Reset the goal index to the first goal
  current_trajectory_idx_ = 0;          // Reset the current trajectory index to the first trajectory
  is_current_trajectory_finished_ = false;

  return true;
}
//}

/* sendTrajectoryToController() //{ */

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
MissionHandler::result_t MissionHandler::sendTrajectoryToController(const trajectory_t& trajectory) {
  mrs_msgs::TrajectoryReferenceSrv trajectory_srv;
  trajectory_srv.request.trajectory = trajectory.reference;

  auto result = callService<mrs_msgs::TrajectoryReferenceSrv>(sc_trajectory_reference_, trajectory_srv.request, trajectory_srv.response);

  if (!result.success) {
    ROS_WARN_STREAM("[MissionHandler]: Failed to send trajectory to controller: " << result.message);
    return {false, result.message};
  }

  ROS_INFO_STREAM("[MissionHandler]: Trajectory sent successfully with " << trajectory.reference.points.size() << " points and " << trajectory.idxs.size()
                                                                         << " waypoints.");
  return {true, "Trajectory sent successfully"};
}
//}

/* createSubtasks() //{ */

/**
 * \brief Executes the given subtasks.
 *
 * This function processes and executes the subtasks associated with a trajectory waypoint.
 * It can handle different types of subtasks based on their type field.
 *
 * \param subtasks A vector of subtasks to be executed.
 */
void MissionHandler::createSubtasks(const std::vector<iroc_mission_handler::Subtask>& subtasks) {
  for (size_t i = 0; i < subtasks.size(); ++i) {
    const auto& subtask = subtasks[i];
    ROS_INFO_STREAM("[MissionHandler]: Creating subtask of type: " << subtask.type);

    // Use the subtask manager to execute the subtask
    bool success = subtask_manager_->createSubtask(subtask, i);
    if (!success) {
      ROS_WARN_STREAM("[MissionHandler]: Failed to create subtask of type: " << subtask.type);
      continue;
    }

    auto [start_success, start_message] = subtask_manager_->startSubtask(i);
    if (!start_success) {
      ROS_WARN_STREAM("[MissionHandler]: Failed to start subtask of type: " << subtask.type << ", message: " << start_message);
      continue;
    }
  }
}
//}

/* transformReferenceArray() //{ */
std::tuple<bool, mrs_msgs::ReferenceArray> MissionHandler::transformReferenceArray(mrs_msgs::TransformReferenceArraySrv TransformArraySrv) {
  if (sc_transform_reference_array_.call(TransformArraySrv)) {
    if (TransformArraySrv.response.success) {
      ROS_INFO_STREAM("Transformation success \"" << sc_transform_reference_array_.getService() << "\" with response \"" << TransformArraySrv.response.message
                                                  << "\".");
      const auto transformed_array = TransformArraySrv.response.array;
      return std::make_tuple(true, transformed_array);
    } else {
      ROS_WARN_STREAM("Transformation failed \"" << sc_transform_reference_.getService() << "\" with response \"" << TransformArraySrv.response.message
                                                 << "\".");
      return std::make_tuple(false, TransformArraySrv.request.array);
    }
  } else {
    ROS_WARN_STREAM("Failed while calling service \"" << sc_transform_reference_.getService() << "\" with response \"" << TransformArraySrv.response.message
                                                      << "\".");
    return std::make_tuple(false, TransformArraySrv.request.array);
  }
}
//}

/* distance() //{ */
double MissionHandler::distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2) {
  using vec3_t = mrs_lib::geometry::vec_t<3>;

  return mrs_lib::geometry::dist(vec3_t(waypoint_1.position.x, waypoint_1.position.y, waypoint_1.position.z),
                                 vec3_t(waypoint_2.position.x, waypoint_2.position.y, waypoint_2.position.z));
}
//}

/* updateMissionState() //{ */

void MissionHandler::updateMissionState(const mission_state_t& new_state) {
  if (mission_state_.value() == new_state) {
    return;
  }

  previous_mission_state_ = mission_state_.value();
  mission_state_.set(new_state);
  actionPublishFeedback();
}
//}

/* resetMission() //{ */

void MissionHandler::resetMission() {
  std::scoped_lock lock(action_server_mutex_);

  current_trajectory_idx_ = 0;
  current_trajectory_waypoint_idx_ = 0;

  mission_waypoint_idx_ = 0;

  waypoint_metrics_.remaining_distance = 0.0;
  waypoint_metrics_.eta = 0.0;
  waypoint_metrics_.progress = 0.0;

  mission_metrics_.remaining_distance = 0.0;
  mission_metrics_.eta = 0.0;
  mission_metrics_.progress = 0.0;

  is_current_trajectory_finished_ = false;
  trajectories_.clear();
  actionPublishFeedback();
  ROS_INFO("[MissionHandler]: Mission reset successfully.");
}
//}

/* callService() //{ */

template <typename ServiceType>
MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, typename ServiceType::Request req, typename ServiceType::Response& res) {
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM_THROTTLE(1.0, msg);
    return {false, msg};
  }
}

template <typename ServiceType>
MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, typename ServiceType::Request req) {
  typename ServiceType::Response res;
  return callService<ServiceType>(sc, req, res);
}

MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, const bool val) {
  using ServiceType = std_srvs::SetBool;
  ServiceType::Request req;
  req.data = val;
  return callService<ServiceType>(sc, req);
}
//}

} // namespace iroc_mission_handler

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::MissionHandler, nodelet::Nodelet);
