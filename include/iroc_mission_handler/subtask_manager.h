#pragma once

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "iroc_mission_handler/common_handlers.h"
#include "iroc_mission_handler/subtask_executor_interface.h"
#include "iroc_mission_handler/Subtask.h"

#include <mutex>

namespace iroc_mission_handler {

/**
 * \brief Manager class for handling multiple subtask executors using plugins
 *
 * This class manages the lifecycle of subtask executors loaded as plugins.
 * It provides functionality to create, start, monitor, and stop subtasks dynamically.
 */
class SubtaskManager {
 public:
  /**
   * \brief Constructor
   *
   * \param nh ROS NodeHandle
   */
  SubtaskManager(ros::NodeHandle& nh);

  /**
   * \brief Check if all active subtasks have completed
   *
   * \return True if all subtasks are completed or no subtasks are active
   */
  bool areAllSubtasksCompleted();

  /**
   * \brief Check if any critical subtasks have failed
   *
   * \return True if any critical subtask has failed
   */
  bool areCriticalSubtasksFailed();

  /**
   * \brief Create a subtask executor using the plugin system
   *
   * \param subtasks Vector of subtasks to create
   *
   * \return True if all non-critical subtasks were created successfully
   */
  bool createSubtasks(const std::vector<Subtask>& subtasks);

  /**
   * \brief Check if a subtask has completed
   *
   * \param progress Reference to store current progress (0.0-1.0)
   *
   * \return True if the subtask is completed
   */
  bool isCurrentSubtaskCompleted(double& progress);

  /**
   * \brief Start all active subtasks
   *
   * \return True if all subtasks were started successfully
   */
  bool startAllSubtasks();

  /**
   * \brief Start the next subtask in the queue
   *
   * \return True if the next subtask was started successfully
   */
  bool startNextSubtask();

  /**
   * \brief Validate waypoint subtasks before mission execution
   *
   * \param subtasks Vector of subtasks to validate
   *
   * \return Tuple of (success, error_message)
   */
  std::tuple<bool, std::string> validateSubtasks(const std::vector<Subtask>& subtasks);

 private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool has_started_subtasks_ = false;
  int current_subtask_id_ = -1;

  // Plugin loader for subtask executors
  std::unique_ptr<pluginlib::ClassLoader<SubtaskExecutor>> plugin_loader_;
  std::map<std::string, std::string> plugin_addresses_;

  // Map of active subtask executors
  std::unordered_map<int, boost::shared_ptr<SubtaskExecutor>> active_subtasks_;

  // Thread safety
  std::mutex mutex_;
};
} // namespace iroc_mission_handler
