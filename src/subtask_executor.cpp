#include "iroc_mission_handler/subtask_executor.h"

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

      // case Subtask::TYPE_GIMBAL:
      //   executor_ptr = std::make_unique<GimbalSubtaskExecutor>(nh_);
      //   break;

    default:
      ROS_ERROR_STREAM("[SubtaskManager]: Unknown subtask type: " << static_cast<int>(subtask.type));
      return std::make_tuple(false, "Unknown subtask type");
  }

  // Execute the subtask
  bool success = executor_ptr->execute(subtask.parameters);
  if (!success) {
    ROS_ERROR_STREAM("[SubtaskManager]: Failed to execute subtask " << id << " of type: " << static_cast<int>(subtask.type));
    return std::make_tuple(false, "Failed to execute subtask");
  }

  // Store the active subtask
  active_subtasks_[id] = std::move(executor_ptr);

  ROS_INFO_STREAM("[SubtaskManager]: Started subtask: " << id);
  return std::make_tuple(true, "Subtask started successfully with ID: " + std::to_string(id));
}

bool SubtaskManager::isSubtaskCompleted(const int id, double& progress) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN_STREAM("[SubtaskManager]: Subtask not found: " << id);
    return true; // Consider completed if not found
  }

  bool completed = it->second->isCompleted(progress);
  if (completed) {
    // Clean up completed subtask
    active_subtasks_.erase(it);
    ROS_DEBUG_STREAM("[SubtaskManager]: Subtask completed and removed: " << id);
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
      ROS_DEBUG_STREAM("[SubtaskManager]: Subtask " << id << " is still running with progress: " << progress);
      return false; // Found a running subtask
    }
  }

  ROS_DEBUG("[SubtaskManager]: All subtasks are completed");
  return true; // All subtasks are completed
}

bool SubtaskManager::stopSubtask(const int id) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN_STREAM("[SubtaskManager]: Cannot stop subtask, not found: " << id);
    return false;
  }

  bool success = it->second->stop();
  if (!success) {
    ROS_ERROR_STREAM("[SubtaskManager]: Failed to stop subtask: " << id);
    return false;
  }

  active_subtasks_.erase(it);
  ROS_INFO_STREAM("[SubtaskManager]: Stopped subtask: " << id);
  return success;
}

void SubtaskManager::stopAllSubtasks() {
  ROS_INFO_STREAM("[SubtaskManager]: Stopping all " << active_subtasks_.size() << " active subtasks");

  for (auto& [id, executor] : active_subtasks_) {
    bool success = executor->stop();
    if (!success) {
      ROS_ERROR_STREAM("[SubtaskManager]: Failed to stop subtask: " << id);
      continue;
    }

    ROS_DEBUG_STREAM("[SubtaskManager]: Stopped subtask: " << id);
  }

  active_subtasks_.clear();
}

// | -------------------- WaitSubtaskExecutor -------------------- |

WaitSubtaskExecutor::WaitSubtaskExecutor(ros::NodeHandle& nh) : nh_(nh) {
  ROS_DEBUG("[WaitSubtaskExecutor]: Initialized");
}

bool WaitSubtaskExecutor::execute(const std::string& parameters) {
  // Parse duration from parameters string
  if (!parseNumber(parameters, duration_)) {
    ROS_ERROR_STREAM("[WaitSubtaskExecutor]: Invalid duration parameter: " << parameters);
    return false;
  }

  if (duration_ <= 0.0) {
    ROS_ERROR_STREAM("[WaitSubtaskExecutor]: Duration must be positive: " << duration_);
    return false;
  }

  start_time_ = ros::Time::now();
  running_ = true;

  ROS_INFO_STREAM("[WaitSubtaskExecutor]: Started waiting for " << duration_ << " seconds");
  return true;
}

bool WaitSubtaskExecutor::isCompleted(double& progress) {
  if (!running_) {
    progress = 1.0;
    return true;
  }

  double elapsed = (ros::Time::now() - start_time_).toSec();
  progress = std::min(elapsed / duration_, 1.0);

  if (elapsed >= duration_) {
    running_ = false;
    progress = 1.0;
    ROS_INFO("[WaitSubtaskExecutor]: Wait completed");
    return true;
  }

  return false;
}

bool WaitSubtaskExecutor::stop() {
  if (running_) {
    running_ = false;
    ROS_INFO("[WaitSubtaskExecutor]: Wait stopped");
  }
  return true;
}

// | -------------------- GimbalSubtaskExecutor -------------------- |
// GimbalSubtaskExecutor::GimbalSubtaskExecutor(ros::NodeHandle& nh) : nh_(nh) {
//   gimbal_pub_ = nh_.advertise<std_msgs::String>("gimbal_command", 10);
//   ROS_DEBUG("[GimbalSubtaskExecutor]: Initialized");
// }

// bool GimbalSubtaskExecutor::execute(const std::string& parameters) {
//   // Parse gimbal control parameters from the parameters string
//   if (!parseNumber(parameters, target_pitch_) || !parseNumber(parameters, target_yaw_) || !parseNumber(parameters, target_roll_)) {
//     ROS_ERROR_STREAM("[GimbalSubtaskExecutor]: Invalid gimbal parameters: " << parameters);
//     return false;
//   }

//   // Create and publish the gimbal command
//   std_msgs::String command;
//   command.data = "Gimbal command: " + std::to_string(target_pitch_) + ", " + std::to_string(target_yaw_) + ", " + std::to_string(target_roll_);
//   gimbal_pub_.publish(command);

//   ROS_INFO_STREAM("[GimbalSubtaskExecutor]: Executing gimbal command: " << command.data);
//   return true;
// }

// bool GimbalSubtaskExecutor::isCompleted(double& progress) {
//   // Check if the gimbal command has completed (dummy implementation)
//   progress = 1.0;
//   return true;
// }

// bool GimbalSubtaskExecutor::stop() {
//   // Stop the gimbal command (dummy implementation)
//   ROS_INFO("[GimbalSubtaskExecutor]: Stopped gimbal command");
//   return true;
// }

} // namespace iroc_mission_handler
