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

  ROS_INFO("[SubtaskManager]: Initialized");
  is_initialized_ = true;
}

bool SubtaskManager::createSubtask(const Subtask& subtask, int id) {
  std::scoped_lock lock(mutex_);

  if (!is_initialized_) {
    ROS_ERROR("[SubtaskManager]: Not initialized, cannot create subtask");
    return false;
  }

  if (id < 0) {
    id = static_cast<int>(active_subtasks_.size());
    ROS_DEBUG("[SubtaskManager]: No ID provided, using next available ID: %d", id);
  }

  try {
    auto it = plugin_addresses_.find(subtask.type);
    if (it == plugin_addresses_.end()) {
      ROS_ERROR("[SubtaskManager]: Subtask type '%s' is not registered in plugin configurations", subtask.type.c_str());
      return false;
    }

    active_subtasks_[id] = plugin_loader_->createInstance(it->second);

    if (!active_subtasks_[id]->initialize(common_handlers_, subtask.parameters)) {
      ROS_ERROR("[SubtaskManager]: Failed to initialize subtask executor for type '%s' with ID %d", subtask.type.c_str(), id);
      active_subtasks_.erase(id);
      return false;
    }
  } catch (const pluginlib::PluginlibException& e) {
    ROS_ERROR("[SubtaskManager]: Exception while creating subtask for type '%s' with ID %d: %s", subtask.type.c_str(), id, e.what());
    return false;
  }

  ROS_DEBUG("[SubtaskManager]: Created subtask executor for type: %s with ID: %d", subtask.type.c_str(), id);
  return true;
}

std::tuple<bool, std::string> SubtaskManager::startSubtask(const int id) {
  std::scoped_lock lock(mutex_);

  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN("[SubtaskManager]: Cannot start subtask, not found: %d", id);
    return std::make_tuple(false, "Subtask not found");
  }

  auto& executor_ptr = it->second;
  if (!executor_ptr) {
    ROS_ERROR("[SubtaskManager]: Subtask executor is not initialized for ID: %d", id);
    return std::make_tuple(false, "Subtask executor not initialized");
  }

  // Execute the subtask
  bool success = executor_ptr->start();
  if (!success) {
    ROS_ERROR("[SubtaskManager]: Failed to start subtask %d", id);
    active_subtasks_.erase(it);
    return std::make_tuple(false, "Failed to start subtask");
  }

  ROS_INFO("[SubtaskManager]: Started subtask: %d", id);
  return std::make_tuple(true, "Subtask started successfully with ID: " + std::to_string(id));
}

bool SubtaskManager::isSubtaskCompleted(const int id, double& progress) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN("[SubtaskManager]: Subtask not found: %d", id);
    return true; // Consider completed if not found
  }

  bool completed = it->second->isCompleted(progress);
  if (completed) {
    // Clean up completed subtask
    active_subtasks_.erase(it);
    ROS_DEBUG("[SubtaskManager]: Subtask completed and removed: %d, progress: %f", id, progress);
  }

  return completed;
}

bool SubtaskManager::areAllSubtasksCompleted() {
  if (active_subtasks_.empty()) {
    ROS_DEBUG("[SubtaskManager]: No active subtasks");
    return true;
  }

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

bool SubtaskManager::stopSubtask(const int id) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN("[SubtaskManager]: Cannot stop subtask, not found: %d", id);
    return false;
  }

  bool success = it->second->stop();
  if (!success) {
    ROS_ERROR("[SubtaskManager]: Failed to stop subtask: %d", id);
    return false;
  }

  active_subtasks_.erase(it);
  ROS_INFO("[SubtaskManager]: Stopped subtask: %d", id);
  return success;
}

void SubtaskManager::stopAllSubtasks() {
  ROS_INFO("[SubtaskManager]: Stopping all %zu active subtasks", active_subtasks_.size());

  for (auto& [id, executor] : active_subtasks_) {
    bool success = executor->stop();
    if (!success) {
      ROS_ERROR("[SubtaskManager]: Failed to stop subtask: %d", id);
      continue;
    }

    ROS_DEBUG("[SubtaskManager]: Stopped subtask: %d", id);
  }

  active_subtasks_.clear();
}

} // namespace iroc_mission_handler
