#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "iroc_mission_handler/Subtask.h"

namespace iroc_mission_handler {
/**
 * @brief Abstract base class for all subtask executors
 */
class SubtaskExecutorBase {
 public:
  virtual ~SubtaskExecutorBase() = default;

  /**
   * @brief Execute the subtask
   * @param parameters JSON string with task-specific parameters
   * @return True if execution started successfully
   */
  virtual bool execute(const std::string& parameters) = 0;

  /**
   * @brief Check if the subtask has completed
   * @param progress Reference to store the progress value (0.0-1.0)
   * @return True if the subtask has completed
   */
  virtual bool isCompleted(double& progress) = 0;

  /**
   * @brief Stop the execution of the subtask
   * @return True if the subtask was stopped successfully
   */
  virtual bool stop() = 0;
};

/**
 * @brief Wait subtask executor - simply waits for a specified duration
 */
class WaitSubtaskExecutor : public SubtaskExecutorBase {
 public:
  WaitSubtaskExecutor(ros::NodeHandle& nh);
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  ros::Time       start_time_;
  double          duration_ = 0.0;
  bool            running_  = false;
};

/**
 * @brief Gimbal subtask executor - controls a gimbal device
 */
class GimbalSubtaskExecutor : public SubtaskExecutorBase {
 public:
  GimbalSubtaskExecutor(ros::NodeHandle& nh);
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  ros::Publisher  gimbal_pub_;
  ros::Time       start_time_;
  double          duration_ = 0.0;
  bool            running_  = false;
};

/**
 * @brief Generic sensor subtask executor - controls various sensors
 */
class SensorSubtaskExecutor : public SubtaskExecutorBase {
 public:
  SensorSubtaskExecutor(ros::NodeHandle& nh);
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle                       nh_;
  std::map<std::string, ros::Publisher> sensor_publishers_;
  ros::Time                             start_time_;
  double                                duration_ = 0.0;
  bool                                  running_  = false;
  std::string                           current_sensor_;
};

/**
 * @brief Manager class for all subtask executors
 */
class SubtaskManager {
 public:
  SubtaskManager(ros::NodeHandle& nh);

  /**
   * @brief Execute a subtask
   * @param subtask The subtask message to execute
   * @return True if execution started successfully
   */
  bool executeSubtask(const Subtask& subtask);

  /**
   * @brief Check if a subtask has completed
   * @param id The ID of the subtask to check
   * @param progress Reference to store the progress value (0.0-1.0)
   * @return True if the subtask has completed, false if still running
   */
  bool isSubtaskCompleted(const std::string& id, double& progress);

  /**
   * @brief Stop a running subtask
   * @param id The ID of the subtask to stop
   * @return True if the subtask was stopped successfully
   */
  bool stopSubtask(const std::string& id);

  /**
   * @brief Stop all running subtasks
   */
  void stopAllSubtasks();

 private:
  ros::NodeHandle                                             nh_;
  std::map<std::string, std::unique_ptr<SubtaskExecutorBase>> active_subtasks_;

  // Factory methods to create subtask executors
  std::unique_ptr<SubtaskExecutorBase> createSubtaskExecutor(uint8_t task_type);
};

// --------------------- WaitSubtaskExecutor --------------------- //

WaitSubtaskExecutor::WaitSubtaskExecutor(ros::NodeHandle& nh) : nh_(nh) {}

bool WaitSubtaskExecutor::execute(const std::string& parameters) {
  try {
    auto params = nlohmann::json::parse(parameters);
    duration_   = params.value("duration", 5.0); // Default 5 seconds if not specified

    start_time_ = ros::Time::now();
    running_    = true;
    ROS_INFO("Starting wait subtask for %.2f seconds", duration_);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to parse wait subtask parameters: %s", e.what());
    return false;
  }
}

bool WaitSubtaskExecutor::isCompleted(double& progress) {
  if (!running_) {
    progress = 1.0;
    return true;
  }

  double elapsed = (ros::Time::now() - start_time_).toSec();
  progress       = std::min(elapsed / duration_, 1.0);

  if (elapsed >= duration_) {
    running_ = false;
    return true;
  }

  return false;
}

bool WaitSubtaskExecutor::stop() {
  running_ = false;
  return true;
}

// --------------------- GimbalSubtaskExecutor --------------------- //

GimbalSubtaskExecutor::GimbalSubtaskExecutor(ros::NodeHandle& nh) : nh_(nh) {
  // Create a generic gimbal publisher - could be different based on your gimbal type
  gimbal_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("gimbal/command", 10);
}

bool GimbalSubtaskExecutor::execute(const std::string& parameters) {
  try {
    auto                        params = nlohmann::json::parse(parameters);
    std_msgs::Float64MultiArray gimbal_msg;

    // Fill the message based on parameters
    if (params.contains("pan")) {
      gimbal_msg.data.push_back(params["pan"].get<double>());
    } else {
      gimbal_msg.data.push_back(0.0);
    }

    if (params.contains("tilt")) {
      gimbal_msg.data.push_back(params["tilt"].get<double>());
    } else {
      gimbal_msg.data.push_back(0.0);
    }

    if (params.contains("roll")) {
      gimbal_msg.data.push_back(params["roll"].get<double>());
    } else {
      gimbal_msg.data.push_back(0.0);
    }

    // Optional duration parameter for movement time
    duration_ = params.value("duration", 2.0); // Default 2 seconds for movement

    // Publish the command
    gimbal_pub_.publish(gimbal_msg);

    start_time_ = ros::Time::now();
    running_    = true;
    ROS_INFO("Starting gimbal subtask with pan=%.2f, tilt=%.2f, roll=%.2f, duration=%.2f", gimbal_msg.data[0], gimbal_msg.data[1], gimbal_msg.data[2],
             duration_);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to parse gimbal subtask parameters: %s", e.what());
    return false;
  }
}

bool GimbalSubtaskExecutor::isCompleted(double& progress) {
  if (!running_) {
    progress = 1.0;
    return true;
  }

  double elapsed = (ros::Time::now() - start_time_).toSec();
  progress       = std::min(elapsed / duration_, 1.0);

  if (elapsed >= duration_) {
    running_ = false;
    return true;
  }

  return false;
}

bool GimbalSubtaskExecutor::stop() {
  // Optionally send a stop command to gimbal
  std_msgs::Float64MultiArray stop_msg;
  // Keep current position, just stop moving
  stop_msg.data = {0.0, 0.0, 0.0};
  gimbal_pub_.publish(stop_msg);

  running_ = false;
  return true;
}

// --------------------- SensorSubtaskExecutor --------------------- //

SensorSubtaskExecutor::SensorSubtaskExecutor(ros::NodeHandle& nh) : nh_(nh) {
  // Sensors can be added dynamically as needed
}

bool SensorSubtaskExecutor::execute(const std::string& parameters) {
  try {
    auto params = nlohmann::json::parse(parameters);

    if (!params.contains("sensor_type")) {
      ROS_ERROR("Sensor subtask missing required 'sensor_type' parameter");
      return false;
    }

    std::string sensor_type = params["sensor_type"];
    current_sensor_         = sensor_type;

    // Create a publisher for this sensor type if it doesn't exist
    if (sensor_publishers_.find(sensor_type) == sensor_publishers_.end()) {
      std::string topic = sensor_type + "/command";

      // Different sensor types might need different message types
      if (sensor_type == "camera") {
        sensor_publishers_[sensor_type] = nh_.advertise<std_msgs::Bool>(topic, 10);
      } else if (sensor_type == "lidar") {
        sensor_publishers_[sensor_type] = nh_.advertise<std_msgs::Bool>(topic, 10);
      } else {
        sensor_publishers_[sensor_type] = nh_.advertise<std_msgs::String>(topic, 10);
      }
    }

    // Handle different sensor types
    if (sensor_type == "camera") {
      std_msgs::Bool cam_msg;
      cam_msg.data = params.value("enable", true);
      sensor_publishers_[sensor_type].publish(cam_msg);
    } else if (sensor_type == "lidar") {
      std_msgs::Bool lidar_msg;
      lidar_msg.data = params.value("enable", true);
      sensor_publishers_[sensor_type].publish(lidar_msg);
    } else {
      // Generic string command for other sensors
      std_msgs::String generic_msg;
      generic_msg.data = params.value("command", "start");
      sensor_publishers_[sensor_type].publish(generic_msg);
    }

    duration_   = params.value("duration", 0.0); // 0 = instantaneous unless specified
    start_time_ = ros::Time::now();
    running_    = (duration_ > 0.0);

    ROS_INFO("Starting %s sensor subtask with duration=%.2f", sensor_type.c_str(), duration_);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to parse sensor subtask parameters: %s", e.what());
    return false;
  }
}

bool SensorSubtaskExecutor::isCompleted(double& progress) {
  if (!running_) {
    progress = 1.0;
    return true;
  }

  double elapsed = (ros::Time::now() - start_time_).toSec();
  progress       = std::min(elapsed / duration_, 1.0);

  if (elapsed >= duration_) {
    running_ = false;
    return true;
  }

  return false;
}

bool SensorSubtaskExecutor::stop() {
  if (!current_sensor_.empty() && sensor_publishers_.find(current_sensor_) != sensor_publishers_.end()) {
    if (current_sensor_ == "camera" || current_sensor_ == "lidar") {
      std_msgs::Bool stop_msg;
      stop_msg.data = false;
      sensor_publishers_[current_sensor_].publish(stop_msg);
    } else {
      std_msgs::String stop_msg;
      stop_msg.data = "stop";
      sensor_publishers_[current_sensor_].publish(stop_msg);
    }
  }

  running_ = false;
  return true;
}

// --------------------- SubtaskManager --------------------- //

SubtaskManager::SubtaskManager(ros::NodeHandle& nh) : nh_(nh) {}

std::unique_ptr<SubtaskExecutorBase> SubtaskManager::createSubtaskExecutor(uint8_t task_type) {
  switch (task_type) {
    case Subtask::TASK_WAIT:
      return std::make_unique<WaitSubtaskExecutor>(nh_);
    case Subtask::TASK_GIMBAL:
      return std::make_unique<GimbalSubtaskExecutor>(nh_);
    case Subtask::TASK_SENSOR:
      return std::make_unique<SensorSubtaskExecutor>(nh_);
    case Subtask::TASK_CUSTOM:
      // Custom executors could be added here
      ROS_WARN("Custom subtask type not implemented yet");
      return nullptr;
    default:
      ROS_ERROR("Unknown subtask type: %d", task_type);
      return nullptr;
  }
}

bool SubtaskManager::executeSubtask(const Subtask& subtask) {
  // Don't execute if already running
  if (active_subtasks_.find(subtask.id) != active_subtasks_.end()) {
    ROS_WARN("Subtask with ID %s is already running", subtask.id.c_str());
    return false;
  }

  // Create appropriate executor based on task type
  auto executor = createSubtaskExecutor(subtask.task_type);
  if (!executor) {
    return false;
  }

  // Execute the subtask
  if (executor->execute(subtask.parameters)) {
    // Store the executor for future reference
    active_subtasks_[subtask.id] = std::move(executor);
    return true;
  }

  return false;
}

bool SubtaskManager::isSubtaskCompleted(const std::string& id, double& progress) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    ROS_WARN("No active subtask with ID %s", id.c_str());
    progress = 1.0; // Assume completed if not found
    return true;
  }

  bool completed = it->second->isCompleted(progress);
  if (completed) {
    // Remove from active tasks if completed
    active_subtasks_.erase(it);
  }

  return completed;
}

bool SubtaskManager::stopSubtask(const std::string& id) {
  auto it = active_subtasks_.find(id);
  if (it == active_subtasks_.end()) {
    return false;
  }

  bool result = it->second->stop();
  active_subtasks_.erase(it);
  return result;
}

void SubtaskManager::stopAllSubtasks() {
  for (auto& [id, executor] : active_subtasks_) {
    executor->stop();
  }
  active_subtasks_.clear();
}
} // namespace iroc_mission_handler
