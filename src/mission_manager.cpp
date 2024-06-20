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

  class MissionManager : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      ros::NodeHandle nh_;

      struct result_t
      {
        bool success;
        std::string message;
      };

      typedef mrs_robot_diagnostics::tracker_state_t tracker_state_t;
      typedef mrs_robot_diagnostics::uav_state_t uav_state_t;

      enum_helpers::enum_updater<uav_state_t> uav_state_ = {"UAV STATE", uav_state_t::UNKNOWN};
      enum_helpers::enum_updater<mission_state_t> mission_state_ = {"MISSION STATE", mission_state_t::IDLE};

      std::atomic_bool is_initialized_ = false;

      // | ---------------------- ROS subscribers --------------------- |
      std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

      mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavState> sh_uav_state_;

      // | ----------------------- ROS clients ---------------------- |
      ros::ServiceClient sc_arm_;
      ros::ServiceClient sc_offboard_;
      ros::ServiceClient sc_land_;
      ros::ServiceClient sc_path_;

      // | ----------------------- main timer ----------------------- |

      ros::Timer timer_main_;
      ros::Timer timer_feedback_;
      void timerMain(const ros::TimerEvent& event);
      void timerFeedback(const ros::TimerEvent& event);

      // | --------------------- actionlib stuff -------------------- |
      //

      typedef actionlib::SimpleActionServer<mrs_mission_manager::waypointMissionAction> MissionManagerServer;
      void                                                                              actionCallbackGoal();
      void                                                                              actionCallbackPreempt();
      std::unique_ptr<MissionManagerServer>                                             mission_manager_server_ptr_;

      typedef mrs_mission_manager::waypointMissionGoal  ActionServerGoal;
      ActionServerGoal                                  action_server_goal_;
      std::recursive_mutex                              action_server_mutex_;

      // | ------------------ Additional functions ------------------ |

      result_t actionGoalValidation(const ActionServerGoal &goal);

      // some helper method overloads
      template <typename Svc_T>
      result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

      template <typename Svc_T>
      result_t callService(ros::ServiceClient& sc);

      result_t callService(ros::ServiceClient& sc, const bool val);

  };
  //}

  /* onInit() //{ */

  void MissionManager::onInit()
  {

    /* obtain node handle */
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    /* load parameters */
    mrs_lib::ParamLoader param_loader(nh_, "MissionManager");

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");
    const auto feedback_timer_rate = param_loader.loadParam2<double>("feedback_timer_rate");

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[MissionManager]: Could not load all parameters!");
      ros::shutdown();
    }

    // | ----------------------- subscribers ---------------------- |

    tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh_;
    shopts.node_name = "MissionManager";
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.timeout_manager = tim_mgr_;
    shopts.threadsafe = true;
    shopts.autostart = true;
    shopts.queue_size = 10;
    shopts.transport_hints = ros::TransportHints().tcpNoDelay();

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

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &MissionManager::timerMain, this);
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

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timerMain() //{ */
  void MissionManager::timerMain([[maybe_unused]] const ros::TimerEvent& event)
  {
    std::scoped_lock lock(action_server_mutex_);
    if (!is_initialized_) {
      ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
      return;
    }
    const uav_state_t previous_uav_state = uav_state_.value();

    // | -------------------- UAV state parsing ------------------- |
    if (sh_uav_state_.hasMsg())
    {
      previous_uav_state = uav_state_.value();
      uav_state_.set(mrs_robot_diagnostics::from_ros<uav_state_t>(sh_uav_state_.getMsg()->state));
    }

    if (mission_manager_server_ptr_->isActive()) {

      switch (mission_state_.value()) 
      {
        case mission_state_t::IDLE: {
            break;
          };
        default:
          break;
      }
    }

  }
  //}

  /* timerFeedback() //{ */
  void MissionManager::timerFeedback([[maybe_unused]] const ros::TimerEvent& event)
  {
    if (!is_initialized_) {
      ROS_WARN_THROTTLE(1, "[MissionManager]: Waiting for nodelet initialization");
      return;
    }
    actionPublishFeedback();

  }
  //}

  // | ---------------------- action server callbacks --------------------- |

  /*  actionCallbackGoal()//{ */

  void MissionManager::actionCallbackGoal() {
    std::scoped_lock lock(action_server_mutex_);
    boost::shared_ptr<const mrs_mission_manager::waypointMissionGoal> new_action_server_goal = mission_manager_server_ptr_->acceptNewGoal();
    ROS_INFO_STREAM("[MissionManager]: Action server received a new goal: \n" << *new_action_server_goal);

    if (!is_initialized_) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Not initialized yet";
      ROS_ERROR("[MrsActionlibInterface]: not initialized yet");
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }

    if (!(uav_state_.state() == uav_state_t::DISARMED || 
        uav_state_.state() == uav_state_t::ARMED)) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Mission can be loaded only when the drone is not flying.";
      ROS_ERROR("[MrsActionlibInterface]: %s", result.message.c_str());
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }

    const auto result = actionGoalValidation(*new_action_server_goal);

    if (!result.success) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = result.message;
      ROS_ERROR("[MrsActionlibInterface]: %s", result.message.c_str());
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }
    mission_state_.set(mission_state_t::MISSION_LOADED);
    action_server_goal_ = *new_action_server_goal;
  }

  //}

  /*  actionCallbackPreempt()//{ */

  void MissionManager::actionCallbackPreempt() {
    std::scoped_lock lock(action_server_mutex_);
    ROS_INFO("[MrsActionlibInterface]: Preemption toggled for ActionServer.");

    if (mission_manager_server_ptr_->isActive()) {
      mrs_mission_manager::waypointMissionResult  action_server_result;
      action_server_result.success   = false;
      action_server_result.message   = "Preempted by server";
      ROS_WARN_STREAM("[MrsActionlibInterface]: " << action_server_result.message);
      mission_manager_server_ptr_->setPreempted(action_server_result);
    }
  }

  //}
  
  /* actionPublishFeedback()//{ */
  void MissionManager::actionPublishFeedback() {
    std::scoped_lock lock(action_server_mutex_);
    if (mission_manager_server_ptr_->isActive()) {
      mrs_mission_manager::waypointMissionFeedback action_server_feedback;
      action_server_feedback.message             = to_string(mission_state_);
      mission_manager_server_ptr_->publishFeedback(action_server_feedback);
    }
  }
  //}

  // | -------------------- support functions ------------------- |

  /* actionGoalValidation() //{ */
  
  MissionManager::result_t MissionManager::actionGoalValidation(const ActionServerGoal &goal){
    std::stringstream ss;
    if (!(goal.frame_id == ActionServerGoal::FRAME_ID_LOCAL || goal.frame_id == ActionServerGoal::FRAME_ID_LATLON)){
      ss << "Unknown frame_id = \'" << goal.frame_id << "\', use the predefined ones.";
      return {false, ss.str()};
    }
    if (!(
          goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_NONE || 
          goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND ||
          goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL 
        )){
      ss << "Unknown terminal_action = \'" << goal.terminal_action << "\', use the predefined ones.";
      return {false, ss.str()};
    }
    if (goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_LAND || 
        goal.terminal_action == ActionServerGoal::TERMINAL_ACTION_RTL){
      ss << "Not implemented terminal action.";
      return {false, ss.str()};
    }

    mrs_msgs::Path msg_path;
    msg_path.points = goal.points;
    msg_path.header.stamp = ros::Time::now();
    msg_path.header.frame_id = goal.frame_id;
    msg_path.fly_now = false;
    msg_path.use_heading = true;

    mrs_msgs::PathSrv::Request srv_path_request;
    srv_path_request.path = msg_path;

    return callService<mrs_msgs::PathSrv>(sc_path_, srv_path_request);
  }
  
  //}

/* callService() //{ */

template <typename Svc_T>
MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc, typename Svc_T::Request req)
{
  typename Svc_T::Response res;
  if (sc.call(req, res))
  {
    ROS_INFO_STREAM("Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
    return {true, res.message};
  }
  else
  {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

template <typename Svc_T>
MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc)
{
  return callService<Svc_T>(sc, {});
}

MissionManager::result_t MissionManager::callService(ros::ServiceClient& sc, const bool val)
{
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}

//}

}  // namespace mrs_mission_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mission_manager::MissionManager, nodelet::Nodelet);
