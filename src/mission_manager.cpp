/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <actionlib/server/simple_action_server.h>
#include <mrs_mission_manager/waypointMissionAction.h>

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

#include <mrs_lib/geometry/misc.h>

#include <atomic>
#include <tuple>
#include <mutex>

#include <mrs_robot_diagnostics/parsing_functions.h>
#include <mrs_robot_diagnostics/UavState.h>

#include "mrs_mission_manager/enums/mission_state.h"


//}


using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

namespace mrs_mission_manager
{

/* class MissionManager //{ */

class MissionManager : public nodelet::Nodelet {
public:
  virtual void onInit();


private:
  ros::NodeHandle nh_;

  struct result_t
  {
    bool        success;
    std::string message;
  };

  typedef mrs_robot_diagnostics::tracker_state_t tracker_state_t;
  typedef mrs_robot_diagnostics::uav_state_t     uav_state_t;

  enum_helpers::enum_updater<uav_state_t>     uav_state_              = {"UAV STATE", uav_state_t::UNKNOWN};
  enum_helpers::enum_updater<mission_state_t> mission_state_          = {"MISSION STATE", mission_state_t::IDLE};
  mission_state_t                             previous_mission_state_ = mission_state_t::IDLE;

  std::string robot_name_;

  std::atomic_bool is_initialized_  = false;
  std::atomic_bool mission_info_processed_  = false;
  std::atomic_bool finished_tracking_  = false;
  std::atomic_bool action_finished_ = false;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState> sh_uav_state_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  // | ----------------------- ROS clients ---------------------- |
  ros::ServiceClient sc_takeoff_;
  ros::ServiceClient sc_land_;
  ros::ServiceClient sc_land_home_;
  ros::ServiceClient sc_path_;
  ros::ServiceClient sc_get_path_;
  ros::ServiceClient sc_hover_;
  ros::ServiceClient sc_mission_flying_to_start_;
  ros::ServiceClient sc_mission_start_;
  ros::ServiceClient sc_mission_validation_;
  ros::ServiceClient sc_trajectory_reference_;
  ros::ServiceClient sc_transform_reference_;
  ros::ServiceClient sc_transform_reference_array_;
  ros::ServiceServer ss_activation_;

  bool missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  ros::Timer timer_feedback_;
  void       timerMain(const ros::TimerEvent& event);
  void       timerFeedback(const ros::TimerEvent& event);

  // | --------------------- actionlib stuff -------------------- |
  //

  typedef actionlib::SimpleActionServer<mrs_mission_manager::waypointMissionAction> MissionManagerServer;
  void                                                                              actionCallbackGoal();
  void                                                                              actionCallbackPreempt();
  std::unique_ptr<MissionManagerServer>                                             mission_manager_server_ptr_;

  typedef mrs_mission_manager::waypointMissionGoal ActionServerGoal;
  ActionServerGoal                                 action_server_goal_;
  std::recursive_mutex                             action_server_mutex_;
  std::recursive_mutex                             mission_informaton_mutex;


  // | --------------------- mission feedback -------------------- |
  std::vector<int>                  path_ids_;
  int                               goal_idx_ = 0;
  int                               current_trajectory_idx_;
  int                               trajectory_length_;
  double                            finish_estimated_time_of_arrival_; //Estimated Time of Arrival (eta)
  double                            goal_estimated_time_of_arrival_; 
  int                               current_trajectory_goal_idx_;
  mrs_msgs::ReferenceStamped        waypoint_;
  mrs_msgs::TrajectoryReference     current_trajectory_; 

  double                            mission_progress_;
  double                            distance_to_finish_;
  double                            goal_progress_;
  int                               goal_start_ = 0;
  double                            distance_to_goal_;

  double tolerance_;
  const double _trajectory_samping_period_ = 0.2;

  // | ------------------ Additional functions ------------------ |

  result_t actionGoalValidation(const ActionServerGoal& goal);
  result_t validateMissionSrv(const mrs_msgs::Path msg) ;
  void     processMissionInfo(const mrs_msgs::ReferenceArray reference_ist);
  void     updateMissionState(const mission_state_t& new_state);
  std::tuple<bool,mrs_msgs::ReferenceStamped> transformReference(mrs_msgs::TransformReferenceSrv transformSrv);
  std::tuple<bool,mrs_msgs::ReferenceArray> transformReferenceArray(mrs_msgs::TransformReferenceArraySrv transformArraySrv);
  double   distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2);
  void     callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

  void     actionPublishFeedback(void);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);
};
//}

/* onInit() //{ */

void MissionManager::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "MissionManager");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("robot_name", robot_name_);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  const auto main_timer_rate     = param_loader.loadParam2<double>("main_timer_rate");
  const auto feedback_timer_rate = param_loader.loadParam2<double>("feedback_timer_rate");
  tolerance_ = param_loader.loadParam2<double>("correspondence_tolerance");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MissionManager]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MissionManager";
  shopts.no_message_timeout = ros::Duration(5.0);
  shopts.timeout_manager    = tim_mgr_;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState>(shopts, "in/uav_state");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in",
                                                                                            &MissionManager::callbackControlManagerDiag, this);

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

  sc_mission_validation_ = nh_.serviceClient<mrs_msgs::ValidateReferenceArray>("svc/mission_validation");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_validation\' -> \'%s\'", sc_mission_validation_.getService().c_str());

  sc_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("svc/trajectory_reference_out");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/trajectory_reference_out\' -> \'%s\'", sc_trajectory_reference_.getService().c_str());

  sc_transform_reference_ = nh_.serviceClient<mrs_msgs::TransformReferenceSrv>("svc/transform_reference");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/transform_reference\' -> \'%s\'", sc_transform_reference_.getService().c_str());

  sc_transform_reference_array_ = nh_.serviceClient<mrs_msgs::TransformReferenceArraySrv>("svc/transform_reference_array");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/transform_reference_array\' -> \'%s\'", sc_transform_reference_.getService().c_str());

  // | --------------------- service servers -------------------- |

  ss_activation_ = nh_.advertiseService("svc_server/mission_activation", &MissionManager::missionActivationServiceCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_activation\' -> \'%s\'", ss_activation_.getService().c_str());

  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &MissionManager::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &MissionManager::timerFeedback, this);

  // | ------------------ action server methods ----------------- |
  //
  mission_manager_server_ptr_ = std::make_unique<MissionManagerServer>(nh_, ros::this_node::getName(), false);
  mission_manager_server_ptr_->registerGoalCallback(boost::bind(&MissionManager::actionCallbackGoal, this));
  mission_manager_server_ptr_->registerPreemptCallback(boost::bind(&MissionManager::actionCallbackPreempt, this));
  mission_manager_server_ptr_->start();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[MissionManager]: initialized");
  ROS_INFO("[MissionManager]: --------------------");
  is_initialized_ = true;
}

//}

// | ------------------------- timers  ------------------------ |

/* timerMain() //{ */
void MissionManager::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  std::scoped_lock lock(action_server_mutex_);
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
    return;
  }

  const uav_state_t previous_uav_state = uav_state_.value();

  // | -------------------- UAV state parsing ------------------- |
  if (sh_uav_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<uav_state_t>(sh_uav_state_.getMsg()->state));
  }
  const bool not_idle_or_land = mission_state_.value() != mission_state_t::IDLE && mission_state_.value() != mission_state_t::LAND;

  if (uav_state_.value() == uav_state_t::LAND && not_idle_or_land) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MissionManager]: Landing detected. Switching to LAND state.");
    updateMissionState(mission_state_t::LAND);
    return;
  }

  if (mission_state_.value() == mission_state_t::LAND) {
    if (uav_state_.value() == uav_state_t::ARMED || uav_state_.value() == uav_state_t::DISARMED || uav_state_.value() == uav_state_t::OFFBOARD) {
      ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Landing finished.");
      if (mission_manager_server_ptr_->isActive()) {
        mrs_mission_manager::waypointMissionResult action_server_result;
        if (action_finished_) {
          action_server_result.success = true;
          action_server_result.message = "Mission finished";
          ROS_INFO("[MissionManager]: Mission finished.");
          mission_manager_server_ptr_->setSucceeded(action_server_result);
        } else {
          action_server_result.success = false;
          action_server_result.message = "Mission stopped due to landing.";
          ROS_WARN("[MissionManager]: Mission stopped due to landing.");
          mission_manager_server_ptr_->setAborted(action_server_result);
        }
      }
      updateMissionState(mission_state_t::IDLE);
      return;
    }
  }

  if (mission_manager_server_ptr_->isActive()) {

    // switch mission to idle if we are in Manual
    if (uav_state_.value() == uav_state_t::MANUAL) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Mission cancelled because drone is under manual control.";
      ROS_INFO("[MissionManager]: %s", action_server_result.message.c_str());
      mission_manager_server_ptr_->setAborted(action_server_result);
      updateMissionState(mission_state_t::IDLE);
      return;
    }

    // mission paused if we are in RC_mode
    if (uav_state_.value() == uav_state_t::RC_MODE) {
      ROS_INFO_STREAM_THROTTLE(1.0,
                               "[MissionManager]: Mission is pause due to active MRS Remote mode. Disable the mode, to continue with the mission execution.");
      updateMissionState(mission_state_t::PAUSED_DUE_TO_RC_MODE);
      return;
    }

    switch (mission_state_.value()) {

      case mission_state_t::TAKEOFF: {
        if ((previous_uav_state == uav_state_t::GOTO || previous_uav_state == uav_state_t::TAKEOFF) && uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Takeoff finished. Executing mission.");
          auto resp = callService<std_srvs::Trigger>(sc_mission_start_);
          if (!resp.success) {
            ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call mission start service.");
            return;
          }
          updateMissionState(mission_state_t::EXECUTING);
          return;
        }
        break;
      };

      case mission_state_t::EXECUTING: {
        // mission finished if were tracking the trajectory and we are now hovering
        if ((previous_uav_state == uav_state_t::TRAJECTORY || previous_uav_state == uav_state_t::GOTO) && uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Trajectory tracking finished.");
          action_finished_ = true;

          switch (action_server_goal_.terminal_action) {

            case ActionServerGoal::TERMINAL_ACTION_LAND: {
              ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executing terminal action. Calling land");
              auto resp = callService<std_srvs::Trigger>(sc_land_);
              if (!resp.success) {
                ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call land service.");
                return;
              }

              updateMissionState(mission_state_t::LAND);
              break;
            };

            case ActionServerGoal::TERMINAL_ACTION_RTL: {
              ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executing terminal action. Calling land home");
              auto resp = callService<std_srvs::Trigger>(sc_land_home_);
              if (!resp.success) {
                ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call land home service.");
                return;
              }

              updateMissionState(mission_state_t::RTL);
              break;
            };

            default: {  
              mrs_mission_manager::waypointMissionResult action_server_result;
              action_server_result.success = true;
              action_server_result.message = "Mission finished";
              ROS_INFO("[MissionManager]: Mission finished.");
              mission_manager_server_ptr_->setSucceeded(action_server_result);

              updateMissionState(mission_state_t::IDLE);
              break;
            };
          }
          return;
        }
        break;
      };

      case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
        // mission continue if we are again not in RC_mode
        if (uav_state_.value() != uav_state_t::RC_MODE) {
          ROS_INFO("[MissionManager]: RC mode disabled. Switching to previous mission mode");
          updateMissionState(previous_mission_state_);
          return;
        }
        break;
      };

      default:
        break;
    }
  }
}
//}

/* timerFeedback() //{ */
void MissionManager::timerFeedback([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
    return;
  }
  actionPublishFeedback();
}
//}

// | ----------------- service server callback ---------------- |

/*  missionActivationServiceCallback()//{ */

bool MissionManager::missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[MissionManager]: Received mission activation request");
  if (mission_manager_server_ptr_->isActive()) {

    switch (mission_state_.value()) {

      case mission_state_t::MISSION_LOADED: {
        if (!mrs_robot_diagnostics::is_flying(uav_state_.value())) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Calling takeoff");
          auto resp   = callService<std_srvs::Trigger>(sc_takeoff_);
          res.success = resp.success;
          res.message = resp.message;
          if (!resp.success) {
            ROS_ERROR_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
            break;
          }
          updateMissionState(mission_state_t::TAKEOFF);
          break;
        } else {

          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executing mission.");
          auto resp   = callService<std_srvs::Trigger>(sc_mission_start_);
          res.success = resp.success;
          res.message = resp.message;
          if (!resp.success) {
            ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call mission start service.");
            break;
          }
          updateMissionState(mission_state_t::EXECUTING);
          break;
        }
      };

      case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
        res.success = false;
        res.message = "Mission is pause due to active MRS Remote mode. Disable the mode, to continue with the mission execution.";
        ROS_ERROR_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
        break;
      };

      default: {
        res.success = false;
        res.message = "Mission is already activated.";
        ROS_WARN_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
        break;
      };
    }
  } else {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
  }
  return true;
}

//}

// | ----------------- msg callback ---------------- |

/* callbackControlManagerDiag() //{ */

void MissionManager::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics) {

  std::scoped_lock lock(mission_informaton_mutex);

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  const bool have_goal = diagnostics->tracker_status.have_goal;
  trajectory_length_ = diagnostics->tracker_status.trajectory_length;
 
  if (trajectory_length_ == 0 || !mission_info_processed_) {
        return;
  }

  //Get current trajectory ID, and the goal ID (from correspondence check)
  current_trajectory_idx_ = diagnostics->tracker_status.trajectory_idx;
  current_trajectory_goal_idx_ = path_ids_.at(goal_idx_); 

  /* Restart distance, as we  will recalculate based on current trajectory idx */
  distance_to_finish_ = 0.0;

  /* Calculating distance from current position to closest goal */
  const mrs_msgs::Reference current_position = current_trajectory_.points.at(current_trajectory_idx_);
  const int closest_goal_idx = path_ids_.at(goal_idx_);
  const mrs_msgs::Reference next_closest_goal = current_trajectory_.points.at(closest_goal_idx);

  distance_to_goal_ = distance (current_position, next_closest_goal);
  distance_to_finish_+= distance_to_goal_;

  /* Calculating distance between remaining goal segments */
  for(size_t i = goal_idx_ ; i < path_ids_.size() - 1; i++) {
    const int current_goal_idx = path_ids_.at(i);
    const mrs_msgs::Reference closest_goal_position = current_trajectory_.points.at(current_goal_idx);
    const int next_goal_idx = path_ids_.at(i + 1);
    mrs_msgs::Reference next_closest_goal_position = current_trajectory_.points.at(next_goal_idx);
    const auto dist = distance (closest_goal_position, next_closest_goal_position);
    distance_to_finish_+= dist;
  }

  const int total_goal_segment = closest_goal_idx - goal_start_;
  /* Calculate goal progress */
  if ( total_goal_segment > 0 ) {
    const int current_progress = (current_trajectory_idx_ - goal_start_);
    const int total_goal_segment = closest_goal_idx - goal_start_;
    const double calculated_goal_progress = (static_cast<double>(current_trajectory_idx_ - goal_start_) / total_goal_segment) * 100;
    goal_progress_ = std::min(calculated_goal_progress, 100.0);
    const double calculated_goal_time = static_cast<double>(closest_goal_idx - current_trajectory_idx_) * _trajectory_samping_period_; 
    goal_estimated_time_of_arrival_ = std::max(calculated_goal_time, 0.0);

  }
  else {
    goal_progress_ = 0;
  }

  /* Calculate overall mission progress */
  if ( trajectory_length_ > 0 ) {
    const double calculated_mission_progress = (static_cast<double>(current_trajectory_idx_) / trajectory_length_) * 100;
    mission_progress_ = std::min(calculated_mission_progress, 100.0);
    const double calculated_mission_time= static_cast<double>(trajectory_length_ - current_trajectory_idx_) * _trajectory_samping_period_;  
    finish_estimated_time_of_arrival_ = std::max(calculated_mission_time, 0.0);
  }
  else {
    ROS_WARN("[MissionManager]: Trajectory length is 0!! Check the trajectory was generated succesfully!");
    mission_progress_ = 0;
  }

  if (current_trajectory_idx_ >= current_trajectory_goal_idx_) {
    ROS_INFO("[MissionManager]: Reached %d waypoint!", goal_idx_ + 1);
    /* If last point in ID's from path points */
    if (current_trajectory_idx_  >= path_ids_.back()) {
      ROS_INFO("[MissionManager]: Reached last point");
      distance_to_finish_ = 0.0;
      distance_to_goal_ = 0.0;
      mission_progress_ = 100.0;
      goal_progress_ = 100.0;
      goal_estimated_time_of_arrival_ = 0.0;
      finish_estimated_time_of_arrival_ = 0.0;
      mission_info_processed_ = false;
    } 
    else {
      goal_start_ = closest_goal_idx; 
      goal_idx_++; //Reached current goal, calculating for next goal

    }
  }
}

//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */

void MissionManager::actionCallbackGoal() {
  std::scoped_lock                                                  lock(action_server_mutex_);
  boost::shared_ptr<const mrs_mission_manager::waypointMissionGoal> new_action_server_goal = mission_manager_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[MissionManager]: Action server received a new goal: \n" << *new_action_server_goal);

  if (!is_initialized_) {
    mrs_mission_manager::waypointMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Not initialized yet";
    ROS_ERROR("[MissionManager]: not initialized yet");
    mission_manager_server_ptr_->setAborted(action_server_result);
    return;
  }

  const auto result = actionGoalValidation(*new_action_server_goal);

  if (!result.success) {
    mrs_mission_manager::waypointMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = result.message;
    ROS_ERROR("[MissionManager]: mission aborted");
    mission_manager_server_ptr_->setAborted(action_server_result);
    return;
  }
  action_finished_ = false;
  updateMissionState(mission_state_t::MISSION_LOADED);
  action_server_goal_ = *new_action_server_goal;
}

//}

/*  actionCallbackPreempt()//{ */

void MissionManager::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (mission_manager_server_ptr_->isActive()) {

    if (mission_manager_server_ptr_->isNewGoalAvailable()) {
      ROS_INFO("[MissionManager]: Preemption toggled for ActionServer.");
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[MissionManager]: " << action_server_result.message);
      mission_manager_server_ptr_->setPreempted(action_server_result);
      updateMissionState(mission_state_t::IDLE);

    } else {
      ROS_INFO("[MissionManager]: Cancel toggled for ActionServer.");

      switch (mission_state_.value()) {
        case mission_state_t::TAKEOFF:
        case mission_state_t::EXECUTING: {

          ROS_INFO_STREAM_THROTTLE(1.0, "Drone is in the movement -> Calling hover.");
          auto resp = callService<std_srvs::Trigger>(sc_hover_);
          if (!resp.success) {
          ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call hover service.");
          }
          mrs_mission_manager::waypointMissionResult action_server_result;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped.";
          mission_manager_server_ptr_->setAborted(action_server_result);
          ROS_INFO("[MissionManager]: Mission stopped.");
          updateMissionState(mission_state_t::IDLE);
          break;
        };

        default:
          mrs_mission_manager::waypointMissionResult action_server_result;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped.";
          mission_manager_server_ptr_->setAborted(action_server_result);
          ROS_INFO("[MissionManager]: Mission stopped.");
          updateMissionState(mission_state_t::IDLE);
          break;
      }
    }
  }
}

//}

/* actionPublishFeedback()//{ */

void MissionManager::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);
  std::scoped_lock mission_lock(mission_informaton_mutex); //Is this right?

  if (mission_manager_server_ptr_->isActive()) {
    mrs_mission_manager::waypointMissionFeedback action_server_feedback;
    action_server_feedback.message = to_string(mission_state_.value());
    action_server_feedback.goal_idx = goal_idx_;
    action_server_feedback.distance_to_closest_goal = distance_to_goal_;
    action_server_feedback.goal_estimated_arrival_time = goal_estimated_time_of_arrival_;
    action_server_feedback.goal_progress = goal_progress_;
    action_server_feedback.distance_to_finish = distance_to_finish_;
    action_server_feedback.finish_estimated_arrival_time = finish_estimated_time_of_arrival_;
    action_server_feedback.mission_progress = mission_progress_;
    mission_manager_server_ptr_->publishFeedback(action_server_feedback);
  }
}

//}

// | -------------------- support functions ------------------- |

/* actionGoalValidation() //{ */

MissionManager::result_t MissionManager::actionGoalValidation(const ActionServerGoal& goal) {
  std::stringstream ss;
  if (!(goal.frame_id == ActionServerGoal::FRAME_ID_LOCAL || goal.frame_id == ActionServerGoal::FRAME_ID_LATLON)) {
    ss << "Unknown frame_id = \'" << int(goal.frame_id) << "\', use the predefined ones.";
    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(goal.height_id == ActionServerGoal::HEIGHT_ID_AGL || goal.height_id == ActionServerGoal::HEIGHT_ID_AMSL)) {
    ss << "Unknown height_id = \'" << int(goal.height_id) << "\', use the predefined ones.";
    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE || goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
        goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL)) {
    ss << "Unknown terminal_action = \'" << int(goal.terminal_action) << "\', use the predefined ones.";
    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  /* if (goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL) { */
  /*   ss << "Not implemented terminal action."; */
  /*   ROS_ERROR_STREAM_THROTTLE(1.0, ss.str()); */
  /*   return {false, ss.str()}; */
  /* } */

  for (int i=0; i< goal.points.size(); i++) { 
    ROS_INFO("[MissionManager]: Received goal : x=%f, y=%f, z=%f",
        goal.points.at(i).position.x,
        goal.points.at(i).position.y,
        goal.points.at(i).position.z);
  }

  std::string frame_id;

  switch (goal.frame_id) {
    case ActionServerGoal::FRAME_ID_LOCAL: {
      frame_id = robot_name_ + "/local_origin";
      break;
    }
    case ActionServerGoal::FRAME_ID_LATLON: {
      frame_id = "latlon_origin";
      break;
    }
    default:
      break;
  }

  std::vector<double> height_points;
  //Saving the AGL height points to be replaced after transformation of latlon points
  if (goal.height_id == ActionServerGoal::HEIGHT_ID_AGL) {
    for (size_t i=0; i < goal.points.size(); i++)  
      height_points.push_back(goal.points.at(i).position.z);
  }
 
  //Create reference array with received points to transform it into current control frame
  mrs_msgs::ReferenceArray goal_points_array;
  goal_points_array.header.frame_id = frame_id;
  goal_points_array.array = goal.points;

  mrs_msgs::ReferenceArray test_transform_array;

  /* This could be replaced with TBD ControlManager Service "transformReferenceArray" */
  mrs_msgs::TransformReferenceArraySrv transformSrv_reference_array;
  transformSrv_reference_array.request.to_frame_id ="";
  transformSrv_reference_array.request.array = goal_points_array; 

  auto [res, transformed_array] = transformReferenceArray(transformSrv_reference_array);

  if (!res) {
    ROS_WARN("[MissionManager]: Failed while transforming the reference array!");
    return {false, "Reference array transformation failed"};
  }

  if (goal.height_id == ActionServerGoal::HEIGHT_ID_AGL) {
    for (size_t i=0; i < transformed_array.array.size(); i++) {
      transformed_array.array.at(i).position.z = height_points.at(i);
    }
  }

  for (size_t i=0; i < transformed_array.array.size(); i++) {

    ROS_INFO("[MissionManager]: Transformed point %zu  x: %f  y: %f z: %f h: %f", i,
        transformed_array.array.at(i).position.x,
        transformed_array.array.at(i).position.y,
        transformed_array.array.at(i).position.z,
        transformed_array.array.at(i).heading
        );
  }

  mrs_msgs::Path msg_path;
  msg_path.points = transformed_array.array;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now      = false;
  msg_path.use_heading  = true;
  // do not use the current position for planning of the path
  msg_path.dont_prepend_current_state = false;
  msg_path.header.frame_id = transformed_array.header.frame_id;  

  /* Validate if path is within safety area */
  const auto result = validateMissionSrv(msg_path);

  if (!result.success) {
    ROS_WARN_STREAM("Trajectory points outside of safety area!");
    return {false, result.message};
  }
  else {
    ROS_INFO_STREAM("Valid trajectory");
    /* Trajectory is valid, processing information for feedback */
    processMissionInfo(transformed_array);
    return {result.success, result.message};
  }
}

//}

/* validateMissionSrv() //{ */

MissionManager::result_t MissionManager::validateMissionSrv(const mrs_msgs::Path msg) {

  /* Generation of trajectory */
  mrs_msgs::GetPathSrv            getPathSrv;
  mrs_msgs::ValidateReferenceArray validateReferenceSrv;

  getPathSrv.request.path = msg;
  if (sc_get_path_.call(getPathSrv)) {
    if (getPathSrv.response.success) {
      ROS_INFO_STREAM("Successfull response from \"" << sc_get_path_.getService() << "\" with response \"" << getPathSrv.response.message << "\".");
    }
    else {
      ROS_INFO_STREAM("Unsuccessfull response from \"" << sc_get_path_.getService() << "\" with response \"" << getPathSrv.response.message << "\".");
      return {false, getPathSrv.response.message};
    }
  }
  else {
      ROS_INFO_STREAM("Failed to call the service: \"" << sc_get_path_.getService());
      return {false, "Failed to call getPath"};
  }

  /* Validation of trajectory within safety area */
  current_trajectory_ = getPathSrv.response.trajectory;
  mrs_msgs::ReferenceArray       waypointArray;
  waypointArray.header               = current_trajectory_.header;
  waypointArray.array                 = current_trajectory_.points;
  validateReferenceSrv.request.array = waypointArray;

  if (sc_mission_validation_.call(validateReferenceSrv)) {
    const bool all_success = std::all_of(validateReferenceSrv.response.success.begin(), validateReferenceSrv.response.success.end(), [](bool v) { return v; });
    if (all_success) 
      ROS_INFO_STREAM("Called service \"" << sc_mission_validation_.getService() << "\" with response \"" << validateReferenceSrv.response.message << "\".");
    else {
      ROS_WARN_STREAM("Trajectory points outside of safety area, validation from  calling service \"" << sc_mission_validation_.getService() << "\" with response \"" << validateReferenceSrv.response.message << "\".");
      return {false, validateReferenceSrv.response.message};
    }
  }

  /* Sending generated trajectory to control manager */
  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = current_trajectory_;

  const bool res = sc_trajectory_reference_.call(srv);

  if (!srv.response.success) {
    ROS_WARN("Service call for trajectory_reference failed,  returned '%s'", srv.response.message.c_str());
    return {false, srv.response.message};
  }
  else {
    return {true, srv.response.message};
  }
}
//}

/* processMissionInfo() //{ */

void MissionManager::processMissionInfo(const mrs_msgs::ReferenceArray reference_array) {

  std::scoped_lock lock(mission_informaton_mutex);
  //Clear member variables used in feedback
  path_ids_.clear();
  goal_idx_ = 0;
  distance_to_finish_ = 0.0;
  distance_to_goal_ = 0.0;
  mission_progress_ = 0.0;
  goal_progress_ = 0.0;
  mission_info_processed_ = false;

  int current_goal_idx = 0;
  mrs_msgs::Reference current_point;
  mrs_msgs::Reference current_trajectory_point;

  //Find corresponding trajectory ID's from the original received points
  //Currently finding closest sample from trajectory generation and original points
  //TODO: Improve correspondence implementation, can be integrated within MRS trajectory generation to be more accurate
  for (size_t i =0; i < current_trajectory_.points.size(); i++) {
    current_point = reference_array.array.at(current_goal_idx); 
    current_trajectory_point = current_trajectory_.points.at(i);
    const double dist = distance(current_point, current_trajectory_point);

    if ( dist < tolerance_ &&  path_ids_.size() < reference_array.array.size()) {
      ROS_INFO("Found the %d point in trajectory, with ID: %zu", current_goal_idx , i);
      path_ids_.push_back(i);
      if (++current_goal_idx == reference_array.array.size()) {
        ROS_INFO("[MissionManager]: Found all path points ID");
        break;
      } 
    }
  }

  if (path_ids_.size() == reference_array.array.size()){
    mission_info_processed_ = true;
  } else { 
    ROS_WARN("[MissionManager]: Did not found all the path correspondences for feedback!");
  }
}
//}

/* transformReference() //{ */
std::tuple<bool,mrs_msgs::ReferenceStamped> MissionManager::transformReference(mrs_msgs::TransformReferenceSrv transformSrv){

  mrs_msgs::ReferenceStamped waypoint_out;
  if (sc_transform_reference_.call(transformSrv)) {
    if (transformSrv.response.success) {
      waypoint_out = transformSrv.response.reference;
      return std::make_tuple(true, waypoint_out);
    }
    else {
      ROS_WARN_STREAM("Transformation failed \"" << sc_transform_reference_.getService() << "\" with response \"" << transformSrv.response.message << "\".");
      return std::make_tuple(false, waypoint_out);
    }
  }
  else {
    ROS_WARN_STREAM("Failed while calling service \"" << sc_transform_reference_.getService() << "\" with response \"" << transformSrv.response.message << "\".");
    return std::make_tuple(false, waypoint_out);
  }
}
//}


/* transformReference() //{ */
std::tuple<bool,mrs_msgs::ReferenceArray> MissionManager::transformReferenceArray(mrs_msgs::TransformReferenceArraySrv TransformArraySrv){

  if (sc_transform_reference_array_.call(TransformArraySrv)) {

    if (TransformArraySrv.response.success) {
      ROS_INFO_STREAM("Transformation success \"" << sc_transform_reference_array_.getService() << "\" with response \"" << TransformArraySrv.response.message << "\".");
      const auto transformed_array = TransformArraySrv.response.array;
      return std::make_tuple(true, transformed_array);

    } else {

      ROS_WARN_STREAM("Transformation failed \"" << sc_transform_reference_.getService() << "\" with response \"" << TransformArraySrv.response.message << "\".");
      return std::make_tuple(false, TransformArraySrv.request.array);
    }

  } else {

    ROS_WARN_STREAM("Failed while calling service \"" << sc_transform_reference_.getService() << "\" with response \"" << TransformArraySrv.response.message << "\".");
    return std::make_tuple(false, TransformArraySrv.request.array);
  }
}
//}


/* distance() //{ */

double MissionManager::distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2){
  return mrs_lib::geometry::dist(vec3_t(waypoint_1.position.x, waypoint_1.position.y,waypoint_1.position.z),
                                 vec3_t(waypoint_2.position.x, waypoint_2.position.y,waypoint_2.position.z));
}
//}

/* updateMissionState() //{ */

void MissionManager::updateMissionState(const mission_state_t& new_state) {
  if (mission_state_.value() == new_state) {
    return;
  }

  previous_mission_state_ = mission_state_.value();
  mission_state_.set(new_state);
  actionPublishFeedback();
}

//}

/* callService() //{ */

template <typename Svc_T>
MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM_THROTTLE(1.0, msg);
    return {false, msg};
  }
}

template <typename Svc_T>
MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc) {
  return callService<Svc_T>(sc, {});
}

MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc, const bool val) {
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}

//}

}  // namespace mrs_mission_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mission_manager::MissionManager, nodelet::Nodelet);
