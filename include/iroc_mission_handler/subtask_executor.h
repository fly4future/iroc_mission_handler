#pragma once

#include <ros/ros.h>

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

}  // namespace iroc_mission_handler
