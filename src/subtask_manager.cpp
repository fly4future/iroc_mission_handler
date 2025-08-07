#include "iroc_mission_handler/subtask_manager.h"

namespace iroc_mission_handler {

SubtaskManager::SubtaskManager(const CommonHandlers& common_handlers) : common_handlers_(common_handlers) {

  // | ----------------------- Load parameters ---------------------- |
  mrs_lib::ParamLoader param_loader(common_handlers.nh, "SubtaskManager");

  param_loader.addYamlFileFromParam("config");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.setPrefix("mission_handler/subtask_manager/");
  std::vector<std::string> available_executors;
  if (!param_loader.loadParam("available_executors", available_executors)) {
    ROS_ERROR("[SubtaskManager]: Failed to load subtask executor configurations");
    ros::shutdown();
    return;
  }

  for (const auto& executor : available_executors) {
    if (!param_loader.loadParam("executors/" + executor + "/address", plugin_addresses_[executor])) {
      ROS_ERROR("[SubtaskManager]: Failed to load address for subtask executor: %s", executor.c_str());
      ros::shutdown();
      return;
    }
    ROS_DEBUG("[SubtaskManager]: Loaded subtask executor '%s' with address '%s'", executor.c_str(), plugin_addresses_[executor].c_str());
  }

  // | ----------------------- Initialize plugin loader ---------------------- |
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<SubtaskExecutor>>("iroc_mission_handler", "iroc_mission_handler::SubtaskExecutor");
  is_initialized_ = true;
}

bool SubtaskManager::areAllSubtasksCompleted() {
  for (const auto& [id, executor] : active_subtasks_) {
    double progress = 0.0;
    if (!executor->isCompleted(progress)) {
      ROS_DEBUG("[SubtaskManager]: Subtask %d is still running with progress: %f", id, progress);
      return false; // Found a running subtask
    }
  }

  ROS_DEBUG("[SubtaskManager]: All subtasks are completed");
  return true; // All subtasks are completed
}

bool SubtaskManager::areCriticalSubtasksFailed() {
  std::scoped_lock lock(mutex_);

  for (const auto& [id, executor] : active_subtasks_) {
    if (executor->isFailed() && executor->shouldStopMissionOnFailure()) {
      ROS_ERROR("[SubtaskManager]: Critical subtask %d failed of type: %s.", id, executor->getType().c_str());
      return true; // Found a critical subtask that failed
    }
  }

  return false;
}

bool SubtaskManager::createSubtasks(const std::vector<Subtask>& subtasks) {
  std::scoped_lock lock(mutex_);

  active_subtasks_.clear();
  current_subtask_id_ = -1; // Reset current subtask ID

  for (size_t id = 0; id < subtasks.size(); ++id) {
    const auto& subtask = subtasks[id];

    try {
      auto it = plugin_addresses_.find(subtask.type);
      if (it == plugin_addresses_.end()) {
        if (subtask.stop_on_failure) {
          ROS_ERROR("[SubtaskManager]: Subtask type '%s' is not registered in plugin configurations", subtask.type.c_str());
          return false;
        } else {
          ROS_WARN("[SubtaskManager]: Subtask type '%s' is not registered in plugin configurations, skipping", subtask.type.c_str());
          continue; // Skip this subtask if it is not critical
        }
      }

      active_subtasks_[id] = plugin_loader_->createInstance(it->second);

      if (!active_subtasks_[id]->initialize(subtask, common_handlers_)) {
        if (subtask.stop_on_failure) {
          ROS_ERROR("[SubtaskManager]: Failed to initialize subtask executor for type '%s' with ID %ld", subtask.type.c_str(), id);
          return false;
        } else {
          ROS_WARN("[SubtaskManager]: Failed to initialize subtask executor for type '%s' with ID %ld, skipping", subtask.type.c_str(), id);
          active_subtasks_.erase(id);
          continue; // Skip this subtask if it is not critical
        }
      }
    } catch (const pluginlib::PluginlibException& e) {
      if (subtask.stop_on_failure) {
        ROS_ERROR("[SubtaskManager]: Plugin creation failed for subtask type '%s': %s", subtask.type.c_str(), e.what());
        return false;
      } else {
        ROS_WARN("[SubtaskManager]: Plugin creation failed for subtask type '%s': %s, skipping", subtask.type.c_str(), e.what());
        continue; // Skip this subtask if it is not critical
      }
    }

    ROS_DEBUG("[SubtaskManager]: Created subtask executor for type: %s with ID: %ld", subtask.type.c_str(), id);
  }

  return true;
}

bool SubtaskManager::isCurrentSubtaskCompleted(double& progress) {
  auto it = active_subtasks_.find(current_subtask_id_);
  if (it == active_subtasks_.end()) {
    progress = 0.0; // No current subtask, consider progress as 0
    return true;
  }

  if (!it->second->hasStarted()) {
    ROS_DEBUG("[SubtaskManager]: Current subtask '%s' has not started yet", it->second->getType().c_str());
    progress = 0.0; // Not started, progress is 0
    return false;
  }

  if (!it->second->shouldWaitForCompletion()) {
    ROS_DEBUG("[SubtaskManager]: Current subtask '%s' does not require waiting for completion", it->second->getType().c_str());
    return true;
  }

  return it->second->isCompleted(progress);
}

bool SubtaskManager::startAllSubtasks() {
  std::scoped_lock lock(mutex_);

  for (const auto& [id, executor] : active_subtasks_) {
    if (!executor->hasStarted()) {
      if (!executor->start()) {
        if (executor->shouldStopMissionOnFailure()) {
          ROS_ERROR("[SubtaskManager]: Failed to start subtask executor for ID: %d", id);
          return false;
        } else {
          ROS_WARN("[SubtaskManager]: Failed to start subtask executor for ID: %d, skipping", id);
          continue; // Skip this subtask if it is not critical
        }
      }

      ROS_DEBUG("[SubtaskManager]: Started subtask: %d", id);
      current_subtask_id_ = id; // Update current subtask ID
    }
  }

  return true;
}

bool SubtaskManager::startNextSubtask() {
  std::scoped_lock lock(mutex_);

  current_subtask_id_++;
  auto it = active_subtasks_.find(current_subtask_id_);
  if (it == active_subtasks_.end()) {
    ROS_DEBUG("[SubtaskManager]: No more subtasks to start");
    return false; // No more subtasks to start
  }

  // Execute the subtask
  auto executor_ptr = it->second;
  if (!executor_ptr->hasStarted()) {
    if (!executor_ptr->start()) {
      if (executor_ptr->shouldStopMissionOnFailure()) {
        ROS_ERROR("[SubtaskManager]: Failed to start subtask executor for ID: %d", current_subtask_id_);
        return false;
      } else {
        ROS_WARN("[SubtaskManager]: Failed to start subtask executor for ID: %d, skipping", current_subtask_id_);
        active_subtasks_.erase(current_subtask_id_);
      }
    }
  }

  return true;
}

std::tuple<bool, std::string> SubtaskManager::validateSubtasks(const std::vector<Subtask>& subtasks, bool parallel_execution) {
  std::scoped_lock lock(mutex_);

  if (!is_initialized_) {
    return std::make_tuple(false, "SubtaskManager not initialized");
  }

  std::stringstream error_messages;
  bool has_errors = false;

  // Validate each subtask in the waypoint
  for (const auto& subtask : subtasks) {
    auto it = plugin_addresses_.find(subtask.type);
    if (it == plugin_addresses_.end()) {
      error_messages << "Subtask type '" << subtask.type << "' is not registered in plugin configurations. ";
      has_errors = true;
      continue;
    }

    // Validate subtask parameters
    if (subtask.type.empty()) {
      error_messages << "Subtask type cannot be empty. ";
      has_errors = true;
    }

    // Try to create a temporary executor to validate parameters (simplified)
    try {
      auto temp_executor = plugin_loader_->createInstance(it->second);
      // Note: parameter validation will happen during initialize phase
      // if (!temp_executor->validateParameters(subtask.parameters)) {
      //   error_messages << "Invalid parameters '" << subtask.parameters << "' for subtask type '" << subtask.type << "'. ";
      //   has_errors = true;
      // }
    } catch (const pluginlib::PluginlibException& e) {
      error_messages << "Plugin creation failed for subtask type '" << subtask.type << "': " << e.what() << ". ";
      has_errors = true;
    }
  }

  // Validate parallel execution logic
  if (parallel_execution) {
    std::unordered_set<std::string> seen;
    for (const auto& subtask : subtasks) {
      if (seen.count(subtask.type)) {
        error_messages << "Subtask type '" << subtask.type << "' is duplicated in parallel execution. ";
        has_errors = true;
      } else {
        seen.insert(subtask.type);
      }
    }
  }

  if (has_errors) {
    return std::make_tuple(false, error_messages.str());
  }

  ROS_INFO("[SubtaskManager]: All waypoint subtasks validated successfully");
  return std::make_tuple(true, "All subtasks validated successfully");
}

} // namespace iroc_mission_handler
