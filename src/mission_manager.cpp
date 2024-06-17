/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <actionlib/server/simple_action_server.h>
#include <mrs_mission_manager/waypointMissionAction.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <atomic>
#include <mutex>

#include <mrs_robot_diagnostics/parsing_functions.h>

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
      enum_helpers::enum_updater<tracker_state_t> tracker_state_ = {"TRACKER STATE", tracker_state_t::NULL_TRACKER};
      enum_helpers::enum_updater<mission_state_t> mission_state_ = {"MISSION STATE", mission_state_t::IDLE};

      std::atomic_bool is_initialized_ = false;

      // | ---------------------- ROS subscribers --------------------- |
      std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

      mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;

      // | ----------------------- main timer ----------------------- |

      ros::Timer timer_main_;
      void timerMain(const ros::TimerEvent& event);

      // | --------------------- actionlib stuff -------------------- |
      //

      typedef actionlib::SimpleActionServer<mrs_mission_manager::waypointMissionAction> MissionManagerServer;
      void                                                                              actionCallbackGoal();
      void                                                                              actionCallbackPreempt();
      std::unique_ptr<MissionManagerServer>                                             mission_manager_server_ptr_;

      typedef mrs_mission_manager::waypointMissionGoal  ActionServerGoal;
      ActionServerGoal                                  action_server_goal_;
      std::mutex                                        action_server_mutex_;

      // | ------------------ Additional functions ------------------ |

      result_t actionGoalValidation(const ActionServerGoal &goal);
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
    mrs_lib::ParamLoader param_loader(nh_, "StateMonitor");

    std::string custom_config_path;

    param_loader.loadParam("custom_config", custom_config_path);

    if (custom_config_path != "")
    {
      param_loader.addYamlFile(custom_config_path);
    }

    param_loader.addYamlFileFromParam("config");

    const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

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

    sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "in/control_manager_diagnostics");
    sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "in/hw_api_status");

    // | ------------------------- timers ------------------------- |

    timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &MissionManager::timerMain, this);

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

    // | -------------------- UAV state parsing ------------------- |
    const bool got_all_messages = sh_hw_api_status_.hasMsg() && sh_control_manager_diagnostics_.hasMsg();
    if (got_all_messages) {
      const auto hw_api_status = sh_hw_api_status_.getMsg();
      const auto control_manager_diagnostics = sh_control_manager_diagnostics_.getMsg();
      
      const auto new_state = mrs_robot_diagnostics::parse_uav_state(hw_api_status, control_manager_diagnostics);
      uav_state_.set(new_state);
    }

    if (mission_manager_server_ptr_->isActive()) {
      switch (mission_state_) 
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

  // | ---------------------- action server callbacks --------------------- |

  /*  actionCallbackGoal()//{ */

  void MissionManager::actionCallbackGoal() {
    std::scoped_lock lock(action_server_mutex_);
    boost::shared_ptr<const mrs_mission_manager::waypointMissionGoal> action_server_goal = mission_manager_server_ptr_->acceptNewGoal();
    ROS_INFO_STREAM("[MissionManager]: Action server received a new goal: \n" << *action_server_goal);

    if (!is_initialized_) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Not initialized yet";
      ROS_ERROR("[MrsActionlibInterface]: not initialized yet");
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }

    const auto result = actionGoalValidation(*action_server_goal);

    if (!result.success) {
      mrs_mission_manager::waypointMissionResult action_server_result;
      action_server_result.success = false;
      action_server_result.message = result.message;
      ROS_ERROR("[MrsActionlibInterface]: %s", result.message.c_str());
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }

    action_server_goal_ = *action_server_goal;
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
  /* void MissionManager::actionPublishFeedback(int state, const std::vector<mrs_msgs::Reference>& trajectory) { */
  /*   if (action_server_->isActive()) { */
  /*     mrs_ewok::ewokFeedback action_server_feedback; */
  /*     action_server_feedback.state_id               = state; */
  /*     action_server_feedback.state                  = state_names[state]; */
  /*     action_server_feedback.trajectory_point_count = (int)trajectory.size(); */
  /*     action_server_->publishFeedback(action_server_feedback); */
  /*   } */
  /* } */
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
    return {true, "Goal is OK."};
  }
  
  //}

}  // namespace mrs_mission_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mission_manager::MissionManager, nodelet::Nodelet);
