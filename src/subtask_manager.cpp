#include "iroc_mission_handler/subtask_manager.h"

namespace iroc_mission_handler {

SubtaskManager::SubtaskManager(const CommonHandlers& common_handlers) : common_handlers_(common_handlers) {
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<SubtaskExecutorBase>>("iroc_mission_handler", "iroc_mission_handler::SubtaskExecutorBase");

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
    active_subtasks_[id] = plugin_loader_->createInstance(subtask.type);
    active_subtasks_[id]->initialize(common_handlers_, subtask.parameters);
  } catch (const std::exception& e) {
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
