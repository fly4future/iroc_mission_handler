#pragma once

#include <ros/ros.h>
#include <mrs_lib/param_loader.h>

#include "iroc_mission_handler/common_handlers.h"
#include "iroc_mission_handler/Subtask.h"
#include "iroc_mission_handler/enums/subtask_state.h"

namespace iroc_mission_handler {

/**
 * \brief Abstract base class for all subtask executors
 *
 * This class defines the interface that all subtask executors must implement.
 * It uses the plugin system from ROS (pluginlib) to allow dynamic loading
 * of different executor types at runtime.
 */
class SubtaskExecutor {
 public:
  virtual ~SubtaskExecutor() = default;
  /**
   * \brief Initialize the executor with ROS components
   *
   * \param common_handlers Common ROS handlers (NodeHandle, SubscribeHandlerOptions, etc.)
   * \param parameters String with task-specific runtime parameters
   *
   * \return True if initialization was successful
   */
  bool initialize(const Subtask& subtask, const CommonHandlers& common_handlers) {
    if (initialized_) {
      ROS_WARN("[SubtaskExecutor]: Already initialized");
      return true;
    }

    // Subtask validation
    subtask_ = std::make_shared<Subtask>(subtask);
    if (!subtask_) {
      ROS_ERROR("[SubtaskExecutor]: Failed to create subtask instance");
      return false;
    }

    if (!validateSubtaskConfig(*subtask_)) {
      ROS_ERROR("[SubtaskExecutor]: Invalid subtask configuration");
      return false;
    }

    // Start initialization and parameter validation
    if (!initializeImpl(common_handlers, subtask_->parameters)) {
      ROS_ERROR("[SubtaskExecutor]: Failed to initialize derived class implementation");
      return false;
    }

    initialized_ = true;
    ROS_INFO("[SubtaskExecutor]: Initialization completed successfully");
    return true;
  }

  /**
   * \brief Execute the subtask
   *
   * \return True if execution started successfully
   */
  bool start() {
    if (!initialized_) {
      ROS_ERROR("[SubtaskExecutor]: Executor not initialized");
      return false;
    }

    // Check if the executor is in a valid state to start
    if (state_ != subtask_state_t::IDLE && state_ != subtask_state_t::FAILED) {
      ROS_WARN("[SubtaskExecutor]: Cannot start, executor is not in IDLE or FAILED state");
      return false;
    }

    // Retry logic
    bool success = startImpl();
    for (uint8_t attempt = 1; attempt <= subtask_->max_retries && !success; ++attempt) {
      ros::Duration(subtask_->retry_delay).sleep(); // Wait before retrying
      ROS_WARN("[SubtaskExecutor]: Retry %d/%d for subtask '%s'", attempt, subtask_->max_retries, subtask_->type.c_str());
      success = startImpl();
    }

    if (!success) {
      ROS_ERROR("[SubtaskExecutor]: Failed to start subtask executor");
      return false;
    }

    state_ = subtask_state_t::RUNNING;
    ROS_INFO("[SubtaskExecutor]: Subtask started successfully");
    return true;
  }

  /**
   * \brief Check if the subtask has completed
   *
   * \param progress Reference to store the progress value (0.0-1.0)
   *
   * \return True if the subtask has completed
   */
  bool isCompleted(double& progress) {
    if (!initialized_) {
      ROS_ERROR("[SubtaskExecutor]: Executor not initialized");
      progress = 0.0;
      return false;
    }

    bool completed = checkCompletion(progress);
    if (completed) {
      if (progress >= 1.0) {
        state_ = subtask_state_t::COMPLETED;
        ROS_INFO("[SubtaskExecutor]: Subtask completed successfully");
      } else {
        state_ = subtask_state_t::FAILED;
        ROS_ERROR("[SubtaskExecutor]: Subtask '%s' failed with progress: %f, stop_on_failure: %s", subtask_->type.c_str(), progress,
                  subtask_->stop_on_failure ? "true" : "false");
      }
    }
    return completed;
  }

  /**
   * \brief Check if the subtask is currently running
   *
   * \return True if the subtask is running
   */
  bool isFailed() const {
    return state_ == subtask_state_t::FAILED;
  }

  /**
   * \brief Check if the subtask has already started. It can be running or completed.
   *
   * \return True if the subtask has started
   */
  bool hasStarted() const {
    return state_ != subtask_state_t::IDLE;
  }

  /**
   * \brief Check if this subtask should stop the mission on failure
   *
   * \return True if the subtask has stop_on_failure flag enabled
   */
  bool shouldStopMissionOnFailure() const {
    if (!subtask_) {
      return false;
    }
    return subtask_->stop_on_failure;
  }

  /**
   * \brief Get the wait_for_completion flag
   *
   * \return True if the mission should wait for this subtask to complete
   */
  bool shouldWaitForCompletion() const {
    if (!subtask_) {
      return true; // Default to wait
    }

    return !subtask_->continue_without_waiting;
  }

  /**
   * \brief Stop the execution of the subtask
   *
   * \return True if the subtask was stopped successfully
   */
  virtual bool stop() = 0;

  /**
   * \brief Get the type of the subtask executor
   *
   * \return The type of the subtask executor
   */
  std::string getType() const {
    if (!subtask_) {
      return "unknown";
    }
    return subtask_->type;
  }

 protected:
  /**
   * \brief Initialize the executor with ROS components (to be implemented by derived classes)
   *
   * \param subtask The subtask definition
   * \param common_handlers Common ROS handlers
   * \param parameters String with task-specific runtime parameters
   *
   * \return True if initialization was successful
   */
  virtual bool initializeImpl(const CommonHandlers& common_handlers, const std::string& parameters) = 0;

  /**
   * \brief Start the subtask execution (to be implemented by derived classes)
   *
   * \return True if the subtask started successfully
   */
  virtual bool startImpl() = 0;

  /**
   * \brief Check completion status (to be implemented by derived classes)
   *
   * \param progress Reference to store the progress value (0.0-1.0)
   *
   * \return True if the subtask has completed
  /**
   * \brief Check if the subtask has completed
   *
   * \param progress Reference to store the progress value (0.0-1.0)
   * \return True if completed (either successfully or failed)
   */
  virtual bool checkCompletion(double& progress) = 0;

  /**
   * \brief Helper function to parse a number from a string
   *
   * \param str The string to parse
   * \param value Reference to store the parsed value
   *
   * \return True if parsing was successful, false otherwise
   */
  bool parseParams(const std::string& str, int& value) const {
    try {
      value = std::stoi(str);
      return true;
    } catch (const std::invalid_argument&) {
      return false;
    }
  }

  bool parseParams(const std::string& str, double& value) const {
    try {
      value = std::stod(str);
      return true;
    } catch (const std::invalid_argument&) {
      return false;
    }
  }

  template <typename T>
  bool parseParams(const std::string& str, std::vector<T>& vec) const {
    vec.clear();

    std::string cleaned_str = str;
    cleaned_str.erase(std::remove(cleaned_str.begin(), cleaned_str.end(), '['), cleaned_str.end());
    cleaned_str.erase(std::remove(cleaned_str.begin(), cleaned_str.end(), ']'), cleaned_str.end());
    cleaned_str.erase(std::remove(cleaned_str.begin(), cleaned_str.end(), ' '), cleaned_str.end());

    std::stringstream ss(cleaned_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      T value;
      if (!parseParams(item, value)) {
        ROS_ERROR_STREAM("[SubtaskExecutorBase]: Failed to parse parameter: " << item);
        return false;
      }
      vec.push_back(value);
    }
    return true;
  }

 private:
  bool initialized_ = false;
  subtask_state_t state_ = subtask_state_t::IDLE;
  std::shared_ptr<Subtask> subtask_; // Pointer to the subtask this executor is handling

  /**
   * \brief Validate subtask configuration
   */
  bool validateSubtaskConfig(const Subtask& subtask) const {
    if (subtask.max_retries > 10) {
      ROS_WARN("[SubtaskExecutor]: Max retries (%d) is unusually high", subtask.max_retries);
    }

    if (subtask.retry_delay < 0.0) {
      ROS_ERROR("[SubtaskExecutor]: Retry delay cannot be negative: %f", subtask.retry_delay);
      return false;
    }

    if (subtask.retry_delay > 60.0) {
      ROS_WARN("[SubtaskExecutor]: Retry delay (%f) is unusually long", subtask.retry_delay);
    }

    return true;
  }
};

} // namespace iroc_mission_handler
