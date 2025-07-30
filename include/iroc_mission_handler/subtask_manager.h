#pragma once

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "iroc_mission_handler/common_handlers.h"

#include "subtask_executor_base.h"
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
   * \param common_handlers CommonHandlers object for managing ROS communication
   */
  SubtaskManager(const CommonHandlers& common_handlers);

  /**
   * \brief Create a subtask executor using the plugin system
   *
   * \param subtask The subtask definition containing type and parameters
   * \param id Unique ID for this subtask instance (auto-generated if < 0)
   *
   * \return True if the executor was created successfully
   */
  bool createSubtask(const Subtask& subtask, int id = -1);

  /**
   * \brief Start execution of a subtask
   *
   * \param id The ID of the subtask to start
   *
   * \return Tuple of (success, message)
   */
  std::tuple<bool, std::string> startSubtask(const int id = 0);

  /**
   * \brief Check if a subtask has completed
   *
   * \param id The ID of the subtask to check
   * \param progress Reference to store current progress (0.0-1.0)
   *
   * \return True if the subtask is completed
   */
  bool isSubtaskCompleted(const int id, double& progress);

  /**
   * \brief Check if all active subtasks have completed
   *
   * \return True if all subtasks are completed or no subtasks are active
   */
  bool areAllSubtasksCompleted();

  /**
   * \brief Stop a specific subtask
   *
   * \param id The ID of the subtask to stop
   *
   * \return True if the subtask was stopped successfully
   */
  bool stopSubtask(const int id);

  /**
   * \brief Stop all active subtasks
   */
  void stopAllSubtasks();

  /**
   * \brief Get the number of active subtasks
   *
   * \return Number of active subtasks
   */
  size_t getActiveSubtaskCount() const;

  /**
   * \brief Get list of available executor types
   *
   * \return Vector of executor type names
   */
  std::vector<std::string> getAvailableExecutorTypes() const;

 private:
  CommonHandlers common_handlers_;
  bool is_initialized_ = false;

  // Plugin loader for subtask executors
  std::unique_ptr<pluginlib::ClassLoader<SubtaskExecutorBase>> plugin_loader_;
  std::map<std::string, XmlRpc::XmlRpcValue> plugin_configs_;

  // Map of active subtask executors
  std::unordered_map<int, boost::shared_ptr<SubtaskExecutorBase>> active_subtasks_;

  // Thread safety
  std::mutex mutex_;
};
} // namespace iroc_mission_handler
