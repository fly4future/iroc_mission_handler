#pragma once

#include <ros/ros.h>
#include <mrs_lib/subscribe_handler.h>
#include "iroc_mission_handler/Subtask.h"

namespace iroc_mission_handler {

/**
 * \brief Abstract base class for all subtask executors
 */
class SubtaskExecutorBase {
 public:
  virtual ~SubtaskExecutorBase() = default;

  /**
   * \brief Execute the subtask
   * \param parameters String with task-specific parameters (format depends on task type)
   * \return True if execution started successfully
   */
  virtual bool execute(const std::string& parameters) = 0;

  /**
   * \brief Check if the subtask has completed
   * \param progress Reference to store the progress value (0.0-1.0)
   * \return True if the subtask has completed
   */
  virtual bool isCompleted(double& progress) = 0;

  /**
   * \brief Stop the execution of the subtask
   * \return True if the subtask was stopped successfully
   */
  virtual bool stop() = 0;

 protected:
  /**
   * \brief Helper function to parse a number from a string
   * \param str The string to parse
   * \param value Reference to store the parsed value
   * \return True if parsing was successful, false otherwise
   */
  bool parseNumber(const std::string& str, int& value) {
    try {
      value = std::stoi(str);
      return true;
    } catch (const std::invalid_argument& e) {
      return false;
    }
  }

  bool parseNumber(const std::string& str, double& value) {
    try {
      value = std::stod(str);
      return true;
    } catch (const std::invalid_argument& e) {
      return false;
    }
  }
};

/**
 * \brief Manager class for all subtask executors
 */
class SubtaskManager {
 public:
  SubtaskManager(const ros::NodeHandle& nh, const mrs_lib::SubscribeHandlerOptions& sh_opts);

  /**
   * \brief Execute a subtask
   * \param subtask The subtask message to execute
   * \param idx The index of the subtask
   * \return A tuple containing success status and the subtask ID if successful or an error message if failed.
   */
  std::tuple<bool, std::string> executeSubtask(const Subtask& subtask, const int id = 0);

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

  // Factory methods to create subtask executors
  std::unique_ptr<SubtaskExecutorBase> createSubtaskExecutor(const Subtask& subtask);
};

/**
 * \brief Wait subtask executor - simply waits for a specified duration
 */
class WaitSubtaskExecutor : public SubtaskExecutorBase {
 public:
  WaitSubtaskExecutor(ros::NodeHandle& nh);
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  ros::Time start_time_;
  double duration_ = 0.0;
  bool running_ = false;
};

/**
 * \brief Gimbal subtask executor - controls a gimbal device
 */
class GimbalSubtaskExecutor : public SubtaskExecutorBase {
 public:
  GimbalSubtaskExecutor(ros::NodeHandle& nh);
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  ros::Publisher gimbal_pub_;
  ros::Time start_time_;
  double duration_ = 0.0;
  bool running_ = false;

  // Gimbal control parameters
  double target_pitch_ = 0.0;
  double target_yaw_ = 0.0;
  double target_roll_ = 0.0;
};

} // namespace iroc_mission_handler
