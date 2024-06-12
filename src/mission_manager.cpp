/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

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


    // | --------------------- finish the init -------------------- |

    ROS_INFO("[MissionManager]: initialized");
    ROS_INFO("[MissionManager]: --------------------");
  }

  //}

}  // namespace mrs_mission_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_mission_manager::MissionManager, nodelet::Nodelet);
