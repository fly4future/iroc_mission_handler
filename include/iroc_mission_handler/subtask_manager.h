#pragma once

#include <ros/ros.h>
#include <mrs_lib/subscribe_handler.h>
#include "iroc_mission_handler/Subtask.h"
#include "iroc_mission_handler/subtask_executors.h"

namespace iroc_mission_handler {

/**
 * \brief Manager class for all subtask executors
 */
class SubtaskManager {
 public:
  SubtaskManager(const ros::NodeHandle& nh, const mrs_lib::SubscribeHandlerOptions& sh_opts);

  /**
   * \brief Execute a subtask
   * \param subtask The subtask message to start
   * \param id The index of the subtask
   * \return A tuple containing success status and the subtask ID if successful or an error message if failed.
   */
  std::tuple<bool, std::string> startSubtask(const Subtask& subtask, const int id = 0);

  /**
   * \brief Check if a subtask has completed
   * \param id The ID of the subtask to check
   * \param progress Reference to store the progress value (0.0-1.0)
   * \return True if the subtask has completed, false if still running
   */
  bool isSubtaskCompleted(const int id, double& progress);

  /**
   * \brief Check if all subtasks have completed
   * \return True if all active subtasks are completed, false otherwise
   */
  bool areAllSubtasksCompleted();

  /**
   * \brief Stop a running subtask
   * \param id The ID of the subtask to stop
   * \return True if the subtask was stopped successfully
   */
  bool stopSubtask(const int id);

  /**
   * \brief Stop all running subtasks
   */
  void stopAllSubtasks();

 private:
  ros::NodeHandle nh_;
  mrs_lib::SubscribeHandlerOptions sh_opts_;
  bool is_initialized_ = false;
  std::map<int, std::unique_ptr<SubtaskExecutorBase>> active_subtasks_;
};

} // namespace iroc_mission_handler
