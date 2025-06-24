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


//}


using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;
namespace iroc_mission_handler
{

/* class MissionHandler //{ */

class MissionHandler : public nodelet::Nodelet {
public:
  virtual void onInit();


private:
  ros::NodeHandle nh_;

  struct result_t
  {
    bool        success;
    std::string message;
  };

  struct path_segments_t
  {
    mrs_msgs::Path path;
    bool is_valid;
  };

  struct trajectory_t
  {
    mrs_msgs::TrajectoryReference trajectory;
    std::vector<long int> trajectory_idxs;
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
  std::atomic_bool pause_requested_ = false;

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
  void       timerMain(const ros::TimerEvent& event);
  void       timerFeedback(const ros::TimerEvent& event);

  // | --------------------- actionlib stuff -------------------- |
  //

  typedef actionlib::SimpleActionServer<iroc_mission_handler::MissionAction> MissionHandlerServer;
  void                                                                              actionCallbackGoal();
  void                                                                              actionCallbackPreempt();
  std::unique_ptr<MissionHandlerServer>                                             mission_handler_server_ptr_;

  typedef iroc_mission_handler::MissionGoal ActionServerGoal;
  ActionServerGoal                                 action_server_goal_;
  std::recursive_mutex                             action_server_mutex_;
  std::recursive_mutex                             mission_informaton_mutex;

  // | --------------------- mission feedback -------------------- |
  int                               goal_idx_ = 0;
  int                               global_goal_idx_ = 0;
  int                               current_trajectory_idx_;
  int                               current_trajectory_length_;
  int                               total_progress_ = 0;
  double                            finish_estimated_time_of_arrival_; //Estimated Time of Arrival (eta)
  double                            goal_estimated_time_of_arrival_; 
  int                               current_trajectory_goal_idx_;
  mrs_msgs::ReferenceArray          current_path_array_;
  mrs_msgs::TrajectoryReference     current_trajectory_; 
  std::vector<long int>             current_trajectory_idxs_; 

  double                            mission_progress_;
  double                            distance_to_finish_;
  double                            goal_progress_;
  int                               goal_start_ = 0;
  double                            distance_to_goal_;

  double                            tolerance_;
  const double                      _trajectory_samping_period_ = 0.2;
  ros::Time                         trajectory_start_time_;

  // | ------------------ Additional functions ------------------ |

  result_t                                                            actionGoalValidation(const ActionServerGoal& action_server_goal);
  result_t                                                            validateMissionSrv(const mrs_msgs::Path msg) ;
  std::vector<path_segments_t>                                        pathValidation(const mrs_msgs::Path msg) ;
  std::tuple<std::vector<mrs_msgs::Reference>, std::vector<long int>> generateHeadingTrajectory(const mrs_msgs::Reference& last_valid_point, const mrs_msgs::Path& path, double T);
  std::tuple<result_t,trajectory_t>                                   getTrajectoryFromSegments(std::vector<path_segments_t> path_segments);
  void                                                                processMissionInfo(const mrs_msgs::ReferenceArray reference_ist);
  bool                                                                replanMission(void);
  void                                                                updateMissionState(const mission_state_t& new_state);
  std::tuple<bool,mrs_msgs::ReferenceStamped>                         transformReference(mrs_msgs::TransformReferenceSrv transformSrv);
  std::tuple<bool,mrs_msgs::ReferenceArray>                           transformReferenceArray(mrs_msgs::TransformReferenceArraySrv transformArraySrv);
  double                                                              distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2);
  void                                                                controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
  void                                                                actionPublishFeedback(void);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req, typename Svc_T::Response& res);


  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);
};
//}

/* onInit() //{ */

void MissionHandler::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "MissionHandler");

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
    ROS_ERROR("[MissionHandler]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MissionHandler";
  shopts.no_message_timeout = ros::Duration(5.0);
  shopts.timeout_manager    = tim_mgr_;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_state_ = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState>(shopts, "in/uav_state");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in",
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

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &MissionHandler::timerMain, this);
  timer_feedback_ = nh_.createTimer(ros::Rate(feedback_timer_rate), &MissionHandler::timerFeedback, this);

  // | ------------------ action server methods ----------------- |
  //
  mission_handler_server_ptr_ = std::make_unique<MissionHandlerServer>(nh_, ros::this_node::getName(), false);
  mission_handler_server_ptr_->registerGoalCallback(boost::bind(&MissionHandler::actionCallbackGoal, this));
  mission_handler_server_ptr_->registerPreemptCallback(boost::bind(&MissionHandler::actionCallbackPreempt, this));
  mission_handler_server_ptr_->start();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[MissionHandler]: initialized");
  ROS_INFO("[MissionHandler]: --------------------");
  is_initialized_ = true;
}

//}

// | ------------------------- timers  ------------------------ |

/* timerMain() //{ */
void MissionHandler::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  std::scoped_lock lock(action_server_mutex_);
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1, "[MissionHandler]: Waiting for nodelet initialization");
    return;
  }

  const uav_state_t previous_uav_state = uav_state_.value();

  // | -------------------- UAV state parsing ------------------- |
  if (sh_uav_state_.hasMsg()) {
    uav_state_.set(mrs_robot_diagnostics::from_ros<uav_state_t>(sh_uav_state_.getMsg()->state));
  }
  const bool not_idle_or_land = mission_state_.value() != mission_state_t::IDLE && mission_state_.value() != mission_state_t::LAND;

  if (uav_state_.value() == uav_state_t::LAND && not_idle_or_land) {
    ROS_WARN_STREAM("[MissionHandler]: Landing detected. Switching to LAND state.");
    updateMissionState(mission_state_t::LAND);
    return;
  }

  if (mission_state_.value() == mission_state_t::LAND) {
    if (uav_state_.value() == uav_state_t::ARMED || uav_state_.value() == uav_state_t::DISARMED || uav_state_.value() == uav_state_t::OFFBOARD) {
      ROS_INFO_STREAM("[MissionHandler]: Landing finished.");
      if (mission_handler_server_ptr_->isActive()) {
        iroc_mission_handler::MissionResult action_server_result;
        if (action_finished_) {
          action_server_result.name = robot_name_; 
          action_server_result.success = true;
          action_server_result.message = "Mission finished";
          ROS_INFO("[MissionHandler]: Mission finished.");
          mission_handler_server_ptr_->setSucceeded(action_server_result);
        } else {

          action_server_result.name = robot_name_; 
          action_server_result.success = false;
          action_server_result.message = "Mission stopped due to landing.";
          ROS_WARN("[MissionHandler]: Mission stopped due to landing.");
          mission_handler_server_ptr_->setAborted(action_server_result);
        }
      }
      updateMissionState(mission_state_t::IDLE);
      return;
    }
  }

  if (mission_handler_server_ptr_->isActive()) {

    // switch mission to idle if we are in Manual
    if (uav_state_.value() == uav_state_t::MANUAL) {
      iroc_mission_handler::MissionResult action_server_result;
      action_server_result.name = robot_name_; 
      action_server_result.success = false;
      action_server_result.message = "Mission cancelled because drone is under manual control.";
      ROS_INFO("[MissionHandler]: %s", action_server_result.message.c_str());
      mission_handler_server_ptr_->setAborted(action_server_result);
      updateMissionState(mission_state_t::IDLE);
      return;
    }

    // mission paused if we are in RC_mode
    if (uav_state_.value() == uav_state_t::RC_MODE) {
      ROS_INFO_STREAM_THROTTLE(1.0,
                               "[MissionHandler]: Mission is pause due to active MRS Remote mode. Disable the mode, to continue with the mission execution.");
      updateMissionState(mission_state_t::PAUSED_DUE_TO_RC_MODE);
      return;
    }

    switch (mission_state_.value()) {

      case mission_state_t::TAKEOFF: {
        if ((previous_uav_state == uav_state_t::GOTO || previous_uav_state == uav_state_t::TAKEOFF) && uav_state_.value() == uav_state_t::HOVER) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Takeoff finished. Executing mission.");
          auto resp = callService<std_srvs::Trigger>(sc_mission_start_);
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call mission start service.");
            return;
          }
          updateMissionState(mission_state_t::EXECUTING);
          return;
        }
        break;
      };

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
          };

          case ActionServerGoal::TERMINAL_ACTION_RTH: {
            ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Executing terminal action. Calling land home");
            auto resp = callService<std_srvs::Trigger>(sc_land_home_);
            if (!resp.success) {
              ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call land home service.");
              return;
            }

            updateMissionState(mission_state_t::RTH);
            break;
          };

          default: {  
            iroc_mission_handler::MissionResult action_server_result;
            action_server_result.name = robot_name_; 
            action_server_result.success = true;
            action_server_result.message = "Mission finished";
            ROS_INFO("[MissionHandler]: Mission finished.");
            mission_handler_server_ptr_->setSucceeded(action_server_result);

            updateMissionState(mission_state_t::IDLE);
            break;
          };
        }
        break;
      };

      case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
        // mission continue if we are again not in RC_mode
        if (uav_state_.value() != uav_state_t::RC_MODE) {
          ROS_INFO("[MissionHandler]: RC mode disabled. Switching to previous mission mode");
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
  if (mission_handler_server_ptr_->isActive()) {
    switch (mission_state_.value()) {

      case mission_state_t::MISSION_LOADED: {
        if (!mrs_robot_diagnostics::is_flying(uav_state_.value())) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Calling takeoff");
          auto resp   = callService<std_srvs::Trigger>(sc_takeoff_);
          res.success = resp.success;
          res.message = resp.message;
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
            break;
          }
          updateMissionState(mission_state_t::TAKEOFF);
          break;
        } else {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Executing mission.");
          auto resp   = callService<std_srvs::Trigger>(sc_mission_start_);
          res.success = resp.success;
          res.message = resp.message;
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call mission start service.");
            break;
          }
          updateMissionState(mission_state_t::EXECUTING);
          break;
        }
      };

      case mission_state_t::PAUSED: {
        ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Replanning mission from current position");
        const auto resp_plan = replanMission();
        if (!resp_plan) { 
          ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to replan mission.");
          res.success = false;
          res.message = "failed to replan mission";
          break;
        } else {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Replanning mission succesfully.");
          updateMissionState(mission_state_t::MISSION_LOADED);
        }

        ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Executing mission.");
        auto resp   = callService<std_srvs::Trigger>(sc_mission_start_);
        res.success = resp.success;
        res.message = resp.message;
        if (!resp.success) {
          ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call mission start service.");
          break;
        }
        updateMissionState(mission_state_t::EXECUTING);
        break;
      };

      case mission_state_t::PAUSED_DUE_TO_RC_MODE: {
        res.success = false;
        res.message = "Mission is pause due to active MRS Remote mode. Disable the mode, to continue with the mission execution.";
        ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
        break;
      };

      default: {
        res.success = false;
        res.message = "Mission is already activated.";
        ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
        break;
      };
    }
  } else {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
  }
  return true;
}

//}

/*  missionPausingServiceCallback()//{ */
bool MissionHandler::missionPausingServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  std::scoped_lock lock(action_server_mutex_);
  ROS_INFO_STREAM("[MissionHandler]: Received mission pausing request.");
  if (mission_handler_server_ptr_->isActive()) {
    switch (mission_state_.value()) {
      
      case mission_state_t::EXECUTING: {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Mission paused. Hover started.");
          auto resp   = callService<std_srvs::Trigger>(sc_mission_pause_);
          res.success = resp.success;
          res.message = resp.message;
          if (!resp.success) {
            ROS_WARN_THROTTLE(1.0, "[MissionHandler]: Failed to call stop trajectory tracking service.");
            break;
          }
          updateMissionState(mission_state_t::PAUSED);
          break;
      };
                                       
      case mission_state_t::TAKEOFF: {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MissionHandler]: Mission paused. Drone will hover after take off is finished.");
          res.success = true;
          res.message = "Switched to PAUSED state.";
          updateMissionState(mission_state_t::PAUSED);
          break;
      };
      default: {
        res.success = false;
        res.message = "Mission is in the state in which cannot be paused.";
        ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
        break;
      };
    }
  } else {
    res.success = false;
    res.message = "No active mission.";
    ROS_WARN_THROTTLE(1.0, "[MissionHandler]: %s", res.message.c_str());
  }
  return true;
}
//}

// | ----------------- msg callback ---------------- |

/* controlManagerDiagCallback() //{ */

void MissionHandler::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics) {
  std::scoped_lock lock(mission_informaton_mutex);

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  const bool have_goal = diagnostics->tracker_status.have_goal;
  current_trajectory_length_ = diagnostics->tracker_status.trajectory_length;

  if (current_trajectory_length_ == 0 || !mission_info_processed_) {
    return;
  }

  // Get current trajectory ID, and the goal ID (from correspondence check)
  current_trajectory_idx_ = diagnostics->tracker_status.trajectory_idx;
  current_trajectory_goal_idx_ = current_trajectory_idxs_.at(goal_idx_); 

  if (current_trajectory_idx_  >= current_trajectory_idxs_.back() || current_trajectory_length_ == 1) {
    ROS_INFO("[MissionHandler]: Reached last point");
    updateMissionState(mission_state_t::FINISHED);
    distance_to_finish_ = 0.0;
    distance_to_goal_ = 0.0;
    mission_progress_ = 100.0;
    goal_progress_ = 100.0;
    goal_estimated_time_of_arrival_ = 0.0;
    finish_estimated_time_of_arrival_ = 0.0;
    mission_info_processed_ = false;
    return;
  }
  /* Restart distance, as we  will recalculate based on current trajectory idx */
  distance_to_finish_ = 0.0;

  /* Calculating distance from current position to closest goal */
  const mrs_msgs::Reference current_position = current_trajectory_.points.at(current_trajectory_idx_);
  const int closest_goal_idx = current_trajectory_idxs_.at(goal_idx_);
  const mrs_msgs::Reference next_closest_goal = current_trajectory_.points.at(closest_goal_idx);

  distance_to_goal_ = distance (current_position, next_closest_goal);
  distance_to_finish_+= distance_to_goal_;

  /* Calculating distance between remaining goal segments */
  for(size_t i = goal_idx_ ; i < current_trajectory_idxs_.size() - 1; i++) {
    const int current_goal_idx = current_trajectory_idxs_.at(i);
    const mrs_msgs::Reference closest_goal_position = current_trajectory_.points.at(current_goal_idx);
    const int next_goal_idx = current_trajectory_idxs_.at(i + 1);
    mrs_msgs::Reference next_closest_goal_position = current_trajectory_.points.at(next_goal_idx);
    const auto dist = distance (closest_goal_position, next_closest_goal_position);
    distance_to_finish_+= dist;
  }

  const int total_goal_segment = closest_goal_idx - goal_start_;
  /* Calculate goal progress */
  if ( total_goal_segment > 0 ) {
    const int current_progress = (current_trajectory_idx_ - goal_start_);
    const double calculated_goal_progress = (static_cast<double>(current_trajectory_idx_ - goal_start_) / total_goal_segment) * 100;
    goal_progress_ = std::min(calculated_goal_progress, 100.0);
    const double calculated_goal_time = static_cast<double>(closest_goal_idx - current_trajectory_idx_) * _trajectory_samping_period_; 
    goal_estimated_time_of_arrival_ = std::max(calculated_goal_time, 0.0);

  } else {
    goal_progress_ = 0;
  }

  /* Calculate overall mission progress */
  if ( current_trajectory_length_ + total_progress_ > 0 ) {
    // using total progress to accumulate the mission progress when pausing the mission
    const double calculated_mission_progress = (static_cast<double>(current_trajectory_idx_ + total_progress_) / (current_trajectory_length_ + total_progress_)) * 100;
    mission_progress_ = std::min(calculated_mission_progress, 100.0);
    const double calculated_mission_time= static_cast<double>(current_trajectory_length_ - current_trajectory_idx_) * _trajectory_samping_period_;  
    finish_estimated_time_of_arrival_ = std::max(calculated_mission_time, 0.0);
  } else {
    ROS_WARN("[MissionHandler]: Trajectory length is 0, validate if the trajectory was succesfully generated.");
    mission_progress_ = 0;
  }

  if (current_trajectory_idx_ >= current_trajectory_goal_idx_) {
    ROS_INFO("[MissionHandler]: Reached %d waypoint", goal_idx_ + 1);
    /* If last point in ID's from path points */
    goal_start_ = closest_goal_idx; 
    goal_idx_++; //Reached current goal, calculating for next goal
  }
}
//}

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */
void MissionHandler::actionCallbackGoal() {
  std::scoped_lock                                                  lock(action_server_mutex_);
  boost::shared_ptr<const iroc_mission_handler::MissionGoal> new_action_server_goal = mission_handler_server_ptr_->acceptNewGoal();
  ROS_INFO_STREAM("[MissionHandler]: Action server received a new goal: \n" << *new_action_server_goal);

  if (!is_initialized_) {
    iroc_mission_handler::MissionResult action_server_result;
    action_server_result.name = robot_name_;
    action_server_result.success = false;
    action_server_result.message = "Not initialized yet";
    ROS_WARN("[MissionHandler]: not initialized yet");
    mission_handler_server_ptr_->setAborted(action_server_result);
    return;
  }

  const auto result = actionGoalValidation(*new_action_server_goal);

  if (!result.success) {
    iroc_mission_handler::MissionResult action_server_result;
    action_server_result.name = robot_name_;
    action_server_result.success = false;
    action_server_result.message = result.message;
    ROS_WARN("[MissionHandler]: mission aborted");
    mission_handler_server_ptr_->setAborted(action_server_result);
    return;
  }
  action_finished_ = false;
  updateMissionState(mission_state_t::MISSION_LOADED);
  action_server_goal_ = *new_action_server_goal;
}
//}

/*  actionCallbackPreempt()//{ */
void MissionHandler::actionCallbackPreempt() {
  std::scoped_lock lock(action_server_mutex_);
  if (mission_handler_server_ptr_->isActive()) {
    if (mission_handler_server_ptr_->isNewGoalAvailable()) {
      ROS_INFO("[MissionHandler]: Preemption toggled for ActionServer.");
      iroc_mission_handler::MissionResult action_server_result;
      action_server_result.name = robot_name_; 
      action_server_result.success = false;
      action_server_result.message = "Preempted by client";
      ROS_WARN_STREAM("[MissionHandler]: " << action_server_result.message);
      mission_handler_server_ptr_->setPreempted(action_server_result);
      updateMissionState(mission_state_t::IDLE);
    } else {
      ROS_INFO("[MissionHandler]: Cancel toggled for ActionServer.");
      switch (mission_state_.value()) {

        // to implement?
        case mission_state_t::TAKEOFF:

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
          mission_handler_server_ptr_->setAborted(action_server_result);
          ROS_INFO("[MissionHandler]: Mission stopped.");
          updateMissionState(mission_state_t::IDLE);
          break;
        };

        default:
          iroc_mission_handler::MissionResult action_server_result;
          action_server_result.name = robot_name_;
          action_server_result.success = false;
          action_server_result.message = "Mission stopped.";
          mission_handler_server_ptr_->setAborted(action_server_result);
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
  std::scoped_lock mission_lock(mission_informaton_mutex); //Is this right?

  if (mission_handler_server_ptr_->isActive()) {
    iroc_mission_handler::MissionFeedback action_server_feedback;
    action_server_feedback.name = robot_name_;
    action_server_feedback.message = to_string(mission_state_.value());
    action_server_feedback.goal_idx = global_goal_idx_ + goal_idx_; // Global goal idx saves previous reached goals for every pausing
    action_server_feedback.distance_to_closest_goal = distance_to_goal_;
    action_server_feedback.goal_estimated_arrival_time = goal_estimated_time_of_arrival_;
    action_server_feedback.goal_progress = goal_progress_;
    action_server_feedback.distance_to_finish = distance_to_finish_;
    action_server_feedback.finish_estimated_arrival_time = finish_estimated_time_of_arrival_;
    action_server_feedback.mission_progress = mission_progress_;
    mission_handler_server_ptr_->publishFeedback(action_server_feedback);
  }
}
//}

// | -------------------- support functions ------------------- |

/* actionGoalValidation() //{ */
MissionHandler::result_t MissionHandler::actionGoalValidation(const ActionServerGoal& action_server_goal) {
  std::stringstream ss;
  if (!(action_server_goal.frame_id == ActionServerGoal::FRAME_ID_LOCAL || action_server_goal.frame_id == ActionServerGoal::FRAME_ID_LATLON || action_server_goal.frame_id == ActionServerGoal::FRAME_ID_FCU)) {
    ss << "Unknown frame_id = \'" << int(action_server_goal.frame_id) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL || action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AMSL || action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_FCU)) {
    ss << "Unknown height_id = \'" << int(action_server_goal.height_id) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }
  if (!(action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE || action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
        action_server_goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTH)) {
    ss << "Unknown terminal_action = \'" << int(action_server_goal.terminal_action) << "\', use the predefined ones.";
    ROS_WARN_STREAM_THROTTLE(1.0, ss.str());
    return {false, ss.str()};
  }

  for (const auto& point : action_server_goal.points) {
    ROS_DEBUG("[MissionHandler]: Point: x:%f y:%f z:%f h:%f ", point.position.x,
               point.position.y,point.position.z,point.heading);
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
      ROS_WARN_STREAM( ss.str());
      return {false, ss.str()};
    }
  }

  // Saving the AGL height points specified in the goal, this is needed as they will be
  // replaced after doing a transformation with latlon points
  std::vector<double> height_points;
  if (action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL) {
    for (const auto& point : action_server_goal.points) {
      height_points.push_back(point.position.z);
    }
  }

  // Create reference array with received points to transform it into current control frame
  mrs_msgs::ReferenceArray goal_points_array;
  goal_points_array.header.frame_id = frame_id;
  goal_points_array.array = action_server_goal.points;

  /* This could be replaced with TBD ControlManager Service "transformReferenceArray" */
  mrs_msgs::TransformReferenceArraySrv transformSrv_reference_array;
  transformSrv_reference_array.request.to_frame_id ="";
  transformSrv_reference_array.request.array = goal_points_array; 

  auto [res, transformed_array] = transformReferenceArray(transformSrv_reference_array);
  if (!res) {
    ROS_WARN("[MissionHandler]: Failed while transforming the reference array!");
    return {false, "Reference array transformation failed"};
  }

  if (action_server_goal.height_id == ActionServerGoal::HEIGHT_ID_AGL && action_server_goal.frame_id != ActionServerGoal::FRAME_ID_FCU) {
    //Replacing the height points after the transformation, as when receiving LATLON points the transformation also considers the height as AMSL. 
    for (size_t i=0; i < transformed_array.array.size(); i++) {
      transformed_array.array.at(i).position.z = height_points.at(i);
    }
  }

  for (const auto&point : transformed_array.array)  {
    ROS_DEBUG("[MissionHandler]: Transformed point x: %f  y: %f z: %f h: %f", point.position.x,
        point.position.y,
        point.position.z,
        point.heading
        );
  }

  goal_points_array = transformed_array;
  current_path_array_ = goal_points_array;
  mrs_msgs::Path msg_path;
  msg_path.points = goal_points_array.array;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now      = false;
  msg_path.use_heading  = true;
  // do not use the current position for planning of the path
  msg_path.dont_prepend_current_state = false;
  msg_path.header.frame_id = goal_points_array.header.frame_id;  

  /* Validate if path is within safety area */
  const auto result = validateMissionSrv(msg_path);

  if (!result.success) {
    ROS_WARN_STREAM("Trajectory points outside of safety area!");
    return {false, result.message};
  } else {
    ROS_INFO_STREAM("Valid trajectory");
    /* Trajectory is valid, processing information for feedback */
    processMissionInfo(goal_points_array);
    return {result.success, result.message};
  }
}
//}

/* validateMissionSrv() //{ */

MissionHandler::result_t MissionHandler::validateMissionSrv(const mrs_msgs::Path msg) {
  //Validate the path and create segments
  auto path_segments = pathValidation(msg);

  auto [result, trajectory_t] = getTrajectoryFromSegments(path_segments);

  if (!result.success) {
    ROS_WARN_STREAM("Failed to get trajectory from segments: " << result.message);
    return {false, result.message};
  }

  /* Validation of trajectory within safety area */
  current_trajectory_        = trajectory_t.trajectory; 
  // Saving trajectory idx
  current_trajectory_idxs_   = trajectory_t.trajectory_idxs; 
  mrs_msgs::ReferenceArray     waypointArray;
  waypointArray.header       = current_trajectory_.header;
  waypointArray.array        = current_trajectory_.points;

  mrs_msgs::ValidateReferenceArray validateReferenceSrv;
  validateReferenceSrv.request.array = waypointArray;

  ROS_DEBUG_STREAM("[MissionHandler]: Path size: " << msg.points.size()); 
  ROS_DEBUG_STREAM("[MissionHandler]: Trajectory size: " << current_trajectory_.points.size()); 
  ROS_DEBUG_STREAM("[MissionHandler]: Trajectory idxs size: " << current_trajectory_idxs_.size());

  for (const auto& id: current_trajectory_idxs_) {
    ROS_DEBUG_STREAM("[MissionHandler]: Trajectory idx: " << id); 
  }

  if (sc_mission_validation_.call(validateReferenceSrv)) {
    const bool all_success = std::all_of(validateReferenceSrv.response.success.begin(),
        validateReferenceSrv.response.success.end(), [](bool v) { return v; });
    if (all_success) { 
      ROS_INFO_STREAM("Called service \"" << sc_mission_validation_.getService() << "\" with response \""
      << validateReferenceSrv.response.message << "\".");
    } else {
      ROS_WARN_STREAM("Trajectory points outside of safety area, validation from  calling service \""
      << sc_mission_validation_.getService() << "\" with response \"" 
      << validateReferenceSrv.response.message << "\".");

      std::vector<mrs_msgs::Reference> unvalid_points;
      for (auto& point_id : current_trajectory_idxs_) {
          if (!validateReferenceSrv.response.success.at(point_id)) {
            unvalid_points.push_back(current_trajectory_.points.at(point_id));
          }
      }

      for (auto& point : unvalid_points) {
        ROS_WARN_STREAM("[MissionHandler]: Unvalid point: " << point);
      }

      if (unvalid_points.size() == 0) {
        ROS_WARN("[MissionHandler]: The given points are valid, however the generated trajectory seems to be outside of safety area or within anobstacle.");
        return {false,"The given points are valid for: "+ robot_name_+ ", however the generated trajectory seems to be outside of safety area or within an obstacle."};
      } else {
        return {false,"Unvalid trajectory for "+ robot_name_+ ", trajectory is outside of safety area"};
      }
    }
  }

  /* Sending generated trajectory to control manager */
  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = current_trajectory_;

  const bool res = sc_trajectory_reference_.call(srv);

  if (!srv.response.success) {
    ROS_WARN("Service call for trajectory_reference failed,  returned '%s'", srv.response.message.c_str());
    return {false, srv.response.message};
  } else {
    return {true, srv.response.message};
  }
}
//}

/* pathValidation() //{ */
 std::vector<MissionHandler::path_segments_t> MissionHandler::pathValidation(const mrs_msgs::Path msg) {
  trajectory_start_time_ = msg.header.stamp;
  //Validate the path and create segments
  std::map<bool,std::vector<mrs_msgs::Path>> path_segments_map;
  std::vector<path_segments_t> path_segments;
  bool in_valid_segment = true;  

  path_segments_t current_segment_t;

  current_segment_t.path.header = msg.header;
  current_segment_t.path.fly_now = msg.fly_now;
  current_segment_t.path.use_heading = msg.use_heading;

  // Add the first point to start a segment
  current_segment_t.path.points.push_back(msg.points[0]); 
  current_segment_t.is_valid = in_valid_segment;

  // Process points from index 1 to end
  for (size_t i = 1; i < msg.points.size(); i++) {
    double dist = distance(msg.points[i-1], msg.points[i]);

    if (dist > 0.05) {
      // Valid distance between points
      if (!in_valid_segment && !current_segment_t.path.points.empty()) {
        // We were in an invalid segment, save it and start a new valid one
        current_segment_t.is_valid = in_valid_segment;
        path_segments.push_back(current_segment_t);
        current_segment_t.path.points.clear();
        in_valid_segment = true;
      }
    } else {
      // Invalid distance between points
      if (in_valid_segment && !current_segment_t.path.points.empty()) {
        // We were in a valid segment, save it and start a new invalid one
        current_segment_t.is_valid = in_valid_segment;
        path_segments.push_back(current_segment_t);
        current_segment_t.path.points.clear();
        in_valid_segment = false;
      }
    }

    // Add the current point to the segment
    current_segment_t.path.points.push_back(msg.points[i]);
  }

  // Add the last segment if it's not empty
  if (!current_segment_t.path.points.empty()) {
    current_segment_t.is_valid = in_valid_segment;
    path_segments.push_back(current_segment_t);
  }

  // This is a debug log/ to remove after testing
  ROS_DEBUG("[MissionHandler]: Path segments: %zu", path_segments.size());
  // Print path segments
  int segment_idx = 0;
  for (const auto& segment : path_segments) {
    auto valid = segment.is_valid ? "Valid" : "Invalid"; 
    ROS_DEBUG("[MissionHandler]: Segment %d: %s", ++segment_idx, valid);
    for (const auto& point : segment.path.points) {
      ROS_DEBUG("[MissionHandler]: Point: %f, %f, %f Heading: %f", point.position.x, point.position.y, point.position.z, point.heading);
    }
  }

  return path_segments; 
}
//}

/* getTrajectoryFromSegments() //{ */
std::tuple<MissionHandler::result_t,MissionHandler::trajectory_t>
MissionHandler::getTrajectoryFromSegments(std::vector<path_segments_t> path_segments) {
  mrs_msgs::TrajectoryReference    trajectory;
  std::vector<mrs_msgs::Reference> aggregated_points;
  bool first_point = true;
  mrs_msgs::Reference last_invalid_point;
  mrs_msgs::Reference last_valid_point;
  last_valid_point = path_segments.begin()->path.points.back();

  bool previous_segment_invalid = false;
  std::vector<long int> trajectory_idxs;
  long int current_trajectory_size = 0;
  trajectory_t trajectory_s;

  for (auto& segment : path_segments) {

    // Invalid segment
    if (!segment.is_valid) {
      // Generating heading trajectory
      auto [heading_trajectory, heading_trajectory_idxs] = generateHeadingTrajectory(last_valid_point, segment.path, 0.2);
      aggregated_points.insert(aggregated_points.end(), heading_trajectory.begin(), heading_trajectory.end());
      last_invalid_point = heading_trajectory.back();

      // Add trajectory idxs to vector
      for (const auto& idx : heading_trajectory_idxs) {
        auto idx_to_add = current_trajectory_size + idx; 
        trajectory_idxs.push_back(idx_to_add);
      }

      // Update current trajectory size
      current_trajectory_size = aggregated_points.size(); 
      previous_segment_invalid = true;

    // Valid segment 
    } else {

      if (!first_point) {
        // As we only want to prepend the current state once
        segment.path.dont_prepend_current_state = true;
      }

      if (previous_segment_invalid) {
        segment.path.points.insert(segment.path.points.begin(), last_invalid_point);
        previous_segment_invalid = false;
      }

      // Using standard mrs_trajectory generation
      mrs_msgs::GetPathSrv            getPathSrv;
      mrs_msgs::ValidateReferenceArray validateReferenceSrv;
      getPathSrv.request.path = segment.path; 
      auto response = getPathSrv.response; 
      // Calling getPath service
      auto resp = callService<mrs_msgs::GetPathSrv>(sc_get_path_, getPathSrv.request , response);

      if (!resp.success) {
        ROS_INFO_STREAM("Failed to call the service: \"" << sc_get_path_.getService());
        return std::make_tuple(result_t{false, resp.message}, trajectory_t{});
      }

      if(first_point) {
        // Copy the header of the trajectory
        trajectory.header = response.trajectory.header;
        trajectory.header.stamp = ros::Time::now();
        trajectory.input_id = response.trajectory.input_id;
        trajectory.use_heading = response.trajectory.use_heading;
        trajectory.fly_now = response.trajectory.fly_now;
        trajectory.loop = response.trajectory.loop;
        trajectory.dt = response.trajectory.dt;
        first_point = false;
      }

      aggregated_points.insert(aggregated_points.end(), response.trajectory.points.begin(), response.trajectory.points.end());
      // Get trajectory idxs
      auto resp_trajectory_idxs = response.waypoint_trajectory_idxs;
 
      if (!resp_trajectory_idxs.empty()) {
        // Add trajectory idxs to vector
        for (const auto& idx : resp_trajectory_idxs) {
          // Print original idx
          auto idx_to_add = (idx == 0 ? current_trajectory_size + 1  : current_trajectory_size + idx);  
          trajectory_idxs.push_back(idx_to_add);
        }
      } else {
        trajectory_idxs.push_back(current_trajectory_size + 1);
      }

      // Update trajectory size
      current_trajectory_size = aggregated_points.size(); 
      last_valid_point.position = aggregated_points.back().position;  
      last_valid_point.heading = aggregated_points.back().heading; 
    }
  }

  trajectory.points = aggregated_points;
  trajectory_s.trajectory = trajectory;
  trajectory_s.trajectory_idxs = trajectory_idxs;
  trajectory.header.stamp = ros::Time::now();

  // Change the stamp based on the time it took to generate the trajectory
  double path_time_offset = (ros::Time::now() - trajectory_start_time_).toSec();
  ROS_INFO("[MissionHandler]: Trajectory generated took: %f seconds", path_time_offset);

  // Print trajectory
  ROS_DEBUG("[MissionHandler]: Trajectory points: %zu", trajectory.points.size());
  for (const auto& point : trajectory.points) {
    ROS_DEBUG("[MissionHandler]: Point: %f, %f, %f Heading: %f", point.position.x, point.position.y, point.position.z, point.heading);
  }

  return std::make_tuple(result_t{true, "Successfull"}, trajectory_s); 
}
//}

/* generateHeadingTrajectory() //{ */

std::tuple<std::vector<mrs_msgs::Reference>, std::vector<long int>> MissionHandler::generateHeadingTrajectory(const mrs_msgs::Reference& last_valid_point, const mrs_msgs::Path& path, double T = 0.2) {
  std::vector<mrs_msgs::Reference> trajectory;
  std::vector<long int> trajectory_idxs;

  if (path.points.empty()) {
    return {trajectory, trajectory_idxs};
  }

  auto p0 = last_valid_point;
  mrs_msgs::Reference point;
  // Interpolation to the next points within segment
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto& p1 = path.points[i];

    // All positions are the same in invalid segments
    double x = p0.position.x;  
    double y = p0.position.y;
    double z = p0.position.z;
    double h0 = p0.heading;
    double h1 = p1.heading;

    // Only interpolate heading if it's different
    if (std::abs(sradians::diff(h0, h1)) > 1e-6) {
      // Calculate number of samples for heading interpolation
      
      auto heading_diff = sradians::diff(h0, h1);
      ROS_DEBUG("[MissionHandler]: h1 %f, h2 %f diff:%f",h0, h1, heading_diff );
      //absolute value of heading diff
      int num_heading_samples = static_cast<int>(std::ceil(std::abs(heading_diff) / T)); 

      for (int j = 1; j <= num_heading_samples; ++j) {
        double t = static_cast<double>(j) / num_heading_samples;
        point.position.x = x;
        point.position.y = y;
        point.position.z = z;
        point.heading    = h0 + t * (std::abs(heading_diff));
        trajectory.push_back(point);
      }
    }
    trajectory_idxs.push_back(trajectory.size()-1);
    p0 = p1;
  }

  //Print trajectory
  for (const auto& point : trajectory) {
    ROS_DEBUG("[MissionHandler]: Trajectory point: %f, %f, %f Heading: %f", point.position.x, point.position.y, point.position.z, point.heading);
    
  }
  return {trajectory, trajectory_idxs};
}
//}

/* processMissionInfo() //{ */

void MissionHandler::processMissionInfo(const mrs_msgs::ReferenceArray reference_array) {
  std::scoped_lock lock(mission_informaton_mutex);
  // Clear member variables used in feedback
  goal_idx_ = 0;
  distance_to_finish_ = 0.0;
  distance_to_goal_ = 0.0;
  mission_progress_ = 0.0;
  goal_progress_ = 0.0;
  mission_info_processed_ = true;
}
//}

/* replanMission() //{ */

bool MissionHandler::replanMission() {
  mrs_msgs::ReferenceArray remaining_path_array;
  // Copy remaining points from current path, starting from the current goal idx 
  remaining_path_array.array.insert(remaining_path_array.array.end(),
      current_path_array_.array.begin() + goal_idx_,
      current_path_array_.array.end());

  for (const auto& point : current_path_array_.array) {
    ROS_DEBUG("[MissionHandler]: Traversed point: x:%f y:%f z:%f h:%f ", point.position.x,point.position.y,point.position.z,point.heading);
  }

  for (const auto& point : remaining_path_array.array) {
    ROS_DEBUG("[MissionHandler]: Remaining point: x:%f y:%f z:%f h:%f ", point.position.x,point.position.y,point.position.z,point.heading);
  }

  mrs_msgs::Path msg_path;
  msg_path.points = remaining_path_array.array;
  msg_path.header.stamp = ros::Time::now();
  msg_path.fly_now      = false;
  msg_path.use_heading  = true;
  // do not use the current position for planning of the path
  msg_path.dont_prepend_current_state = false;
  msg_path.header.frame_id = current_path_array_.header.frame_id;  

  const auto result = validateMissionSrv(msg_path);

  if (!result.success) {
    ROS_WARN("Service call for trajectory_reference failed,  returned '%s'", result.message.c_str());
    return false;
  } else {
    global_goal_idx_ += goal_idx_; //Saving the reached goals
    current_path_array_ = remaining_path_array; //If a new pause is triggered, will continue with current path
    total_progress_ += current_trajectory_idx_; //Accumulate current progress
    processMissionInfo(remaining_path_array);
    return true;
  }
}
//}

/* transformReference() //{ */
std::tuple<bool,mrs_msgs::ReferenceStamped> MissionHandler::transformReference(mrs_msgs::TransformReferenceSrv transformSrv){
  mrs_msgs::ReferenceStamped waypoint_out;
  if (sc_transform_reference_.call(transformSrv)) {
    if (transformSrv.response.success) {
      waypoint_out = transformSrv.response.reference;
      return std::make_tuple(true, waypoint_out);
    } else {
      ROS_WARN_STREAM("Transformation failed \"" << sc_transform_reference_.getService() << "\" with response \"" << transformSrv.response.message << "\".");
      return std::make_tuple(false, waypoint_out);
    }
  } else {
    ROS_WARN_STREAM("Failed while calling service \"" << sc_transform_reference_.getService() << "\" with response \"" << transformSrv.response.message << "\".");
    return std::make_tuple(false, waypoint_out);
  }
}
//}

/* transformReferenceArray() //{ */
std::tuple<bool,mrs_msgs::ReferenceArray> MissionHandler::transformReferenceArray(mrs_msgs::TransformReferenceArraySrv TransformArraySrv){
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
double MissionHandler::distance(const mrs_msgs::Reference& waypoint_1, const mrs_msgs::Reference& waypoint_2){
  return mrs_lib::geometry::dist(vec3_t(waypoint_1.position.x, waypoint_1.position.y,waypoint_1.position.z),
                                 vec3_t(waypoint_2.position.x, waypoint_2.position.y,waypoint_2.position.z));
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

/* callService() //{ */

template <typename Svc_T>
MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
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

template <typename Svc_T>
MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, typename Svc_T::Request req, typename Svc_T::Response& res) {
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
template <typename Svc_T>
MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc) {
  return callService<Svc_T>(sc, {});
}

MissionHandler::result_t MissionHandler::callService(ros::ServiceClient& sc, const bool val) {
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}
//}

}  // namespace iroc_mission_handler

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::MissionHandler, nodelet::Nodelet);
