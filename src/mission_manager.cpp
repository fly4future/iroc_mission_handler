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

    std::atomic_bool have_new_goal_ = false;
    std::atomic_bool is_initialized_ = false;

    // | --------------------- actionlib stuff -------------------- |
    //

    typedef actionlib::SimpleActionServer<mrs_mission_manager::waypointMissionAction> MissionManagerServer;
    void                                                                              actionCallbackGoal();
    void                                                                              actionCallbackPreempt();
    std::unique_ptr<MissionManagerServer>                                             mission_manager_server_ptr_;

    static const mrs_mission_manager::waypointMissionGoal _ACTION_SERVER_GOAL_;
    mrs_mission_manager::waypointMissionGoal              action_server_goal_;
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

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[MissionManager]: Could not load all parameters!");
      ros::shutdown();
    }
    mission_manager_server_ptr_ = std::make_unique<MissionManagerServer>(nh_, ros::this_node::getName(), false);
    mission_manager_server_ptr_->registerGoalCallback(boost::bind(&MissionManager::actionCallbackGoal, this));
    mission_manager_server_ptr_->registerPreemptCallback(boost::bind(&MissionManager::actionCallbackPreempt, this));
    mission_manager_server_ptr_->start();

    // | --------------------- finish the init -------------------- |

    ROS_INFO("[MissionManager]: initialized");
    ROS_INFO("[MissionManager]: --------------------");
    is_initialized_ = true;
  }

  // | ---------------------- action server callbacks --------------------- |

  /*  actionCallbackGoal()//{ */

  void MissionManager::actionCallbackGoal() {

    have_new_goal_ = false;

    boost::shared_ptr<const mrs_mission_manager::waypointMissionGoal> temp = mission_manager_server_ptr_->acceptNewGoal();

    ROS_INFO("[MissionManager]: got a new goal from the action server");

    if (!is_initialized_) {
      mrs_mission_manager::waypointMissionResult  action_server_result;
      action_server_result.success = false;
      action_server_result.message = "Not initialized yet";
      ROS_ERROR("[MrsActionlibInterface]: not initialized yet");
      mission_manager_server_ptr_->setAborted(action_server_result);
      return;
    }


    // TODO: check that th goal is valid

    have_new_goal_ = true;
    action_server_goal_ = *temp;
  }

  //}

  /*  actionCallbackPreempt()//{ */

  void MissionManager::actionCallbackPreempt() {

    if (mission_manager_server_ptr_->isActive()) {

      ROS_INFO_ONCE("[MissionManager]: preempted");

      // abort the mission
      mission_manager_server_ptr_->setPreempted();
    }
  }

//}

  //}

}  // namespace mrs_mission_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mission_manager::MissionManager, nodelet::Nodelet);
