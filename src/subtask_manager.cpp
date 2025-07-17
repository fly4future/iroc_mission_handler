#include "iroc_mission_handler/subtask_manager.h"

namespace iroc_mission_handler {

// | ---------------------- SubtaskManager ---------------------- |

SubtaskManager::SubtaskManager(const ros::NodeHandle& nh, const mrs_lib::SubscribeHandlerOptions& sh_opts) : nh_(nh), sh_opts_(sh_opts) {
  ROS_INFO("[SubtaskManager]: Initialized");
  is_initialized_ = true;
}

std::tuple<bool, std::string> SubtaskManager::executeSubtask(const Subtask& subtask, const int id) {
  // Create the appropriate executor
  std::unique_ptr<SubtaskExecutorBase> executor_ptr = nullptr;
  switch (subtask.type) {
    case Subtask::TYPE_WAIT:
      executor_ptr = std::make_unique<WaitSubtaskExecutor>(nh_);
      break;

    case Subtask::TYPE_GIMBAL:
      executor_ptr = std::make_unique<GimbalSubtaskExecutor>(nh_, sh_opts_);
      break;

    default:
      ROS_ERROR("[SubtaskManager]: Unknown subtask type: %d", static_cast<int>(subtask.type));
      return std::make_tuple(false, "Unknown subtask type");
  }

  // Execute the subtask
  bool success = executor_ptr->execute(subtask.parameters);
  if (!success) {
    ROS_ERROR("[SubtaskManager]: Failed to execute subtask %d of type: %d", id, static_cast<int>(subtask.type));
    return std::make_tuple(false, "Failed to execute subtask");
  }

  // Store the active subtask
  active_subtasks_[id] = std::move(executor_ptr);

  ROS_INFO("[SubtaskManager]: Started subtask: %d of type: %d", id, static_cast<int>(subtask.type));
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
  ROS_INFO("[SubtaskManager]: Stopping all %ld active subtasks", active_subtasks_.size());

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
