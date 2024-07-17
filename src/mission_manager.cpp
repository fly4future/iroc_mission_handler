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

#include <atomic>
#include <mutex>

#include <mrs_robot_diagnostics/parsing_functions.h>
#include <mrs_robot_diagnostics/UavState.h>

#include "mrs_mission_manager/enums/mission_state.h"


//}

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
  std::atomic_bool action_finished_ = false;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState> sh_uav_state_;

  // | ----------------------- ROS clients ---------------------- |
  ros::ServiceClient sc_takeoff_;
  ros::ServiceClient sc_land_;
  ros::ServiceClient sc_path_;
  ros::ServiceClient sc_hover_;
  ros::ServiceClient sc_mission_flying_to_start_;
  ros::ServiceClient sc_mission_start_;

  ros::ServiceServer ss_activation_;
  bool               missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

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

  // | ------------------ Additional functions ------------------ |

  result_t actionGoalValidation(const ActionServerGoal& goal);
  void     updateMissionState(const mission_state_t& new_state);
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

  // | --------------------- service clients -------------------- |

  sc_takeoff_ = nh_.serviceClient<std_srvs::Trigger>("svc/takeoff");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/offboard\' -> \'%s\'", sc_takeoff_.getService().c_str());

  sc_land_ = nh_.serviceClient<std_srvs::Trigger>("svc/land");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land\' -> \'%s\'", sc_land_.getService().c_str());

  sc_path_ = nh_.serviceClient<mrs_msgs::PathSrv>("svc/path");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/path\' -> \'%s\'", sc_path_.getService().c_str());

  sc_hover_ = nh_.serviceClient<std_srvs::Trigger>("svc/hover");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/hover\' -> \'%s\'", sc_hover_.getService().c_str());

  sc_mission_flying_to_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_flying_to_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_flying_to_start\' -> \'%s\'", sc_mission_flying_to_start_.getService().c_str());

  sc_mission_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_start\' -> \'%s\'", sc_mission_start_.getService().c_str());

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
              ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executiong terminal action. Calling land");
              auto resp = callService<std_srvs::Trigger>(sc_land_);
              if (!resp.success) {
                ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call land service.");
                return;
              }

              updateMissionState(mission_state_t::LAND);
              break;
            };

              // TODO: add TERMINAL_ACTION_RTL part

            default: {  // in the case of TERMINAL_ACTION_RTL, do the same as for TERMINAL_ACTION_NONE
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
        case mission_state_t::PAUSED_DUE_TO_RC_MODE:
        case mission_state_t::EXECUTING: {
          switch (action_server_goal_.terminal_action) {

            case ActionServerGoal::TERMINAL_ACTION_LAND: {
              ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executiong terminal action. Calling land");
              auto resp = callService<std_srvs::Trigger>(sc_land_);
              if (!resp.success) {
                ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call land service.");
                return;
              }
              updateMissionState(mission_state_t::LAND);
              break;
            };

              // TODO: add TERMINAL_ACTION_RTL part

            default: {  // in the case of TERMINAL_ACTION_RTL, do the same as for TERMINAL_ACTION_NONE
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
          };
          break;
        };

        default:
          break;
      }
    }
  }
}

//}

/* actionPublishFeedback()//{ */

void MissionManager::actionPublishFeedback() {
  std::scoped_lock lock(action_server_mutex_);
  if (mission_manager_server_ptr_->isActive()) {
    mrs_mission_manager::waypointMissionFeedback action_server_feedback;
    action_server_feedback.message = to_string(mission_state_.value());
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
  if (!(goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE || goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
        goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL)) {
    ss << "Unknown terminal_action = \'" << int(goal.terminal_action) << "\', use the predefined ones.";
    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL) {
    ss << "Not implemented terminal action.";
    ROS_ERROR_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }

  mrs_msgs::Path msg_path;
  msg_path.points       = goal.points;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now      = false;
  msg_path.use_heading  = true;
  // do not use the current position for plannin of the path
  msg_path.dont_prepend_current_state = false;

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

  msg_path.header.frame_id = frame_id;

  mrs_msgs::PathSrv::Request srv_path_request;
  srv_path_request.path = msg_path;
  auto resp             = callService<mrs_msgs::PathSrv>(sc_path_, srv_path_request);
  return {resp.success, resp.message};
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
