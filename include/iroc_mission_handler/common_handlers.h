#pragma once

#include <ros/ros.h>
#include <mrs_lib/subscribe_handler.h>

namespace iroc_mission_handler {

/**
 * \brief Struct to hold common handlers for subtasks
 * This struct is used to pass common parameters to subtask executors.
 */
struct CommonHandlers {
  ros::NodeHandle nh;
  mrs_lib::SubscribeHandlerOptions sh_opts;
};

} // namespace iroc_mission_handler