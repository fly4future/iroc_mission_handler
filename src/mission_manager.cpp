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

  std::atomic_bool is_initialized_ = false;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState> sh_uav_state_;

  // | ----------------------- ROS clients ---------------------- |
  ros::ServiceClient sc_arm_;
  ros::ServiceClient sc_offboard_;
  ros::ServiceClient sc_land_;
  ros::ServiceClient sc_path_;
  ros::ServiceClient sc_mission_pause_;
  ros::ServiceClient sc_mission_flying_to_start_;
  ros::ServiceClient sc_mission_start_;

  ros::ServiceServer ss_activation_;
  bool               missionActivationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  ros::ServiceServer ss_pause_;
  bool               missionPauseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

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
  result_t takeoffAction();

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

  sc_arm_ = nh_.serviceClient<std_srvs::SetBool>("svc/arm");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/arm\' -> \'%s\'", sc_arm_.getService().c_str());

  sc_offboard_ = nh_.serviceClient<std_srvs::Trigger>("svc/offboard");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/offboard\' -> \'%s\'", sc_offboard_.getService().c_str());

  sc_land_ = nh_.serviceClient<std_srvs::Trigger>("svc/land");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land\' -> \'%s\'", sc_land_.getService().c_str());

  sc_path_ = nh_.serviceClient<mrs_msgs::PathSrv>("svc/path");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/path\' -> \'%s\'", sc_path_.getService().c_str());

  sc_mission_pause_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_pause");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_pause\' -> \'%s\'", sc_mission_pause_.getService().c_str());

  sc_mission_flying_to_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_flying_to_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_flying_to_start\' -> \'%s\'", sc_mission_flying_to_start_.getService().c_str());

  sc_mission_start_ = nh_.serviceClient<std_srvs::Trigger>("svc/mission_start");
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_start\' -> \'%s\'", sc_mission_start_.getService().c_str());

  // | --------------------- service servers -------------------- |

  ss_activation_ = nh_.advertiseService("svc_server/mission_activation", &MissionManager::missionActivationServiceCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_activation\' -> \'%s\'", ss_activation_.getService().c_str());

  ss_pause_ = nh_.advertiseService("svc_server/mission_pause", &MissionManager::missionPauseServiceCallback, this);
  ROS_INFO("[IROCBridge]: Created ServiceServer on service \'svc_server/mission_pause\' -> \'%s\'", ss_pause_.getService().c_str());

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
        if (uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Takeoff finished.");
          auto resp = callService<std_srvs::Trigger>(sc_mission_flying_to_start_);
          if (!resp.success) {
            ROS_ERROR_THROTTLE(1.0, "[MissionManager]: Failed to call flying to mission start service.");
            return;
          }
          updateMissionState(mission_state_t::FLYING_TO_START);
          return;
        }
        break;
      };

      case mission_state_t::FLYING_TO_START: {
        if (previous_uav_state == uav_state_t::GOTO && uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Flying to start finished");
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
        if (previous_uav_state == uav_state_t::TRAJECTORY && uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Mission finished.");

          switch (action_server_goal_.terminal_action) {

            case ActionServerGoal::TERMINAL_ACTION_LAND: {
              ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Executiong terminal action. Callin land");
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

      case mission_state_t::LAND: {
        if (uav_state_.value() == uav_state_t::DISARMED) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionManager]: Landing finished.");

          mrs_mission_manager::waypointMissionResult action_server_result;
          action_server_result.success = true;
          action_server_result.message = "Mission finished";
          ROS_INFO("[MissionManager]: Mission finished.");
          mission_manager_server_ptr_->setSucceeded(action_server_result);

          updateMissionState(mission_state_t::IDLE);
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
        auto call_res = takeoffAction();
        res.success   = call_res.success;
        res.message   = call_res.message;

        if (call_res.success) {
          updateMissionState(mission_state_t::TAKEOFF);
        }
        break;
      };

      case mission_state_t::PAUSED: {
        ROS_INFO_STREAM_THROTTLE(1.0, "Calling reactivation.");
        auto resp = callService<std_srvs::Trigger>(sc_mission_start_);
        if (!resp.success) {
          res.success = false;
          res.message = "Failed to call mission reactivation service.";
          ROS_ERROR_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
          break;
        }
        res.success = true;
        res.message = "Mission reactivated";
        updateMissionState(mission_state_t::EXECUTING);
        break;
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

/*  missionPauseServiceCallback()//{ */

bool MissionManager::missionPauseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[MissionManager]: Received mission pause request");
  if (mission_manager_server_ptr_->isActive()) {

    switch (mission_state_.value()) {

      case mission_state_t::EXECUTING: {
        ROS_INFO_STREAM_THROTTLE(1.0, "Calling pausing.");
        auto resp = callService<std_srvs::Trigger>(sc_mission_pause_);
        if (!resp.success) {
          res.success = false;
          res.message = "Failed to call mission pause service.";
          ROS_ERROR_THROTTLE(1.0, "[MissionManager]: %s", res.message.c_str());
          break;
        }
        res.success = true;
        res.message = "Mission paused";
        updateMissionState(mission_state_t::PAUSED);
        break;
      };

      default: {
        res.success = false;
        res.message = "Mission is not in the executing state.";
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

  if (!(uav_state_.value() == uav_state_t::DISARMED || uav_state_.value() == uav_state_t::ARMED)) {
    mrs_mission_manager::waypointMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Mission can be loaded only when the drone is not flying.";
    ROS_ERROR("[MissionManager]: %s", action_server_result.message.c_str());
    mission_manager_server_ptr_->setAborted(action_server_result);
    return;
  }

  const auto result = actionGoalValidation(*new_action_server_goal);

  if (!result.success) {
    mrs_mission_manager::waypointMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = result.message;
    ROS_ERROR("[MissionManager]: %s", result.message.c_str());
    mission_manager_server_ptr_->setAborted(action_server_result);
    return;
  }
  updateMissionState(mission_state_t::MISSION_LOADED);
  action_server_goal_ = *new_action_server_goal;
}

//}

/*  actionCallbackPreempt()//{ */

void MissionManager::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO("[MissionManager]: Preemption toggled for ActionServer.");

  if (mission_manager_server_ptr_->isActive()) {
    mrs_mission_manager::waypointMissionResult action_server_result;
    action_server_result.success = false;
    action_server_result.message = "Preempted by server";
    ROS_WARN_STREAM("[MissionManager]: " << action_server_result.message);
    mission_manager_server_ptr_->setPreempted(action_server_result);
  }
  updateMissionState(mission_state_t::IDLE);
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
    return {false, ss.str()};
  }
  if (!(goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE || goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
        goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL)) {
    ss << "Unknown terminal_action = \'" << int(goal.terminal_action) << "\', use the predefined ones.";
    return {false, ss.str()};
  }
  if (goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL) {
    ss << "Not implemented terminal action.";
    return {false, ss.str()};
  }

  mrs_msgs::Path msg_path;
  msg_path.points       = goal.points;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now      = false;
  msg_path.use_heading  = true;
  // do not use the current position for plannin of the path
  msg_path.dont_prepend_current_state  = true;

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

  return callService<mrs_msgs::PathSrv>(sc_path_, srv_path_request);
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

/* takeoffAction() //{ */

MissionManager::result_t MissionManager::takeoffAction() {
  // firstly, arm the vehicles
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling arm.");
  auto resp = callService(sc_arm_, true);
  if (!resp.success) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MissionManager]: Failed to call arm service.");
    return {false, "Failed to arm."};
  }

  const ros::Duration wait_after_arm(1.0);
  ROS_INFO_STREAM_THROTTLE(1.0, "Waiting " << wait_after_arm.toSec() << "s after arming before swithing to offboard mode.");
  wait_after_arm.sleep();

  ROS_INFO_STREAM_THROTTLE(1.0, "Calling takeoff by switching to the offboard mode");
  resp = callService<std_srvs::Trigger>(sc_offboard_);
  if (!resp.success) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MissionManager]: Failed to call offboard.");
    return {false, "Failed to call offboard."};
  }
  return {true, "Takeoff called."};
}

//}

/* callService() //{ */

template <typename Svc_T>
MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
  if (sc.call(req, res)) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
    return {true, res.message};
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
