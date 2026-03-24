#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>

#include "iroc_mission_handler/msg/subtask.hpp"
#include "iroc_mission_handler/enums/subtask_state.h"

namespace iroc_mission_handler
{

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
   * \param nh ROS NodeHandle
   * \param parameters String with task-specific runtime parameters
   *
   * \return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, const iroc_mission_handler::msg::Subtask &subtask) {

    if (initialized_) {
      RCLCPP_WARN(node->get_logger(), "[SubtaskExecutor]: Already initialized");
      return true;
    }

    node_  = node;
    clock_ = node->get_clock();

    // Subtask validation
    subtask_ = std::make_shared<iroc_mission_handler::msg::Subtask>(subtask);
    if (!subtask_) {
      RCLCPP_WARN(node->get_logger(), "[SubtaskExecutor]: Invalid subtask configuration");
      return false;
    }

    if (!validateSubtaskConfig(*subtask_)) {
      RCLCPP_WARN(node->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
      return false;
    }

    // Start initialization and parameter validation
    if (!initializeImpl(node, subtask_->parameters)) {
      RCLCPP_WARN(node->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
      return false;
    }

    initialized_ = true;
    RCLCPP_INFO(node->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
    return true;
  }

  /**
   * \brief Execute the subtask
   *
   * \return True if execution started successfully
   */
  bool start() {
    if (!initialized_) {
      // node_ and subtask_ are only assigned inside initialize(), so they are
      // null here. Use a named logger to avoid a null-pointer dereference.
      RCLCPP_WARN(rclcpp::get_logger("SubtaskExecutor"),
                  "[SubtaskExecutor]: Executor not initialized — call initialize() before start()");
      return false;
    }

    // Check if the executor is in a valid state to start
    if (state_ != subtask_state_t::IDLE && state_ != subtask_state_t::FAILED) {
      RCLCPP_WARN(node_->get_logger(), "[SubtaskExecutor]: Cannot start executor is in either FAILED or IDLE state for subtask type: %s, parameters: %s",
                  subtask_->type.c_str(), subtask_->parameters.c_str());
      return false;
    }

    // Retry logic
    bool success = startImpl();
    for (uint8_t attempt = 1; attempt <= subtask_->max_retries && !success; ++attempt) {
      clock_->sleep_for(std::chrono::duration<double>(subtask_->retry_delay));

      RCLCPP_WARN(node_->get_logger(), "[SubtaskExecutor]: Retry %d/%d for subtask type: %s, parameters: %s", attempt, subtask_->max_retries,
                  subtask_->type.c_str(), subtask_->parameters.c_str());
      success = startImpl();
    }

    if (!success) {
      RCLCPP_WARN(node_->get_logger(), "[SubtaskExecutor]: Failed to start subtask type: %s, parameters: %s after %d attempts", subtask_->type.c_str(),
                  subtask_->parameters.c_str(), subtask_->max_retries);
      state_ = subtask_state_t::FAILED;
      return false;
    }

    state_ = subtask_state_t::RUNNING;
    RCLCPP_INFO(node_->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
    return true;
  }

  /**
   * \brief Check if the subtask has completed
   *
   * \param progress Reference to store the progress value (0.0-1.0)
   *
   * \return True if the subtask has completed
   */
  bool isCompleted(double &progress) {
    if (!initialized_ || state_ == subtask_state_t::IDLE) {
      progress = 0.0;
      return false;
    }

    bool completed = checkCompletion(progress);
    if (completed) {
      if (progress >= 1.0) {
        state_ = subtask_state_t::COMPLETED;
        RCLCPP_INFO(node_->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
      } else {
        state_ = subtask_state_t::FAILED;
        RCLCPP_WARN(node_->get_logger(), "[SubtaskExecutor]: Subtask type: %s, parameters: %s", subtask_->type.c_str(), subtask_->parameters.c_str());
      }
    }

    if (state_ == subtask_state_t::FAILED) {
      return true; // Return true even if failed, to allow checking progress
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
   * \param nh NodeHandle for ROS communication
   * \param parameters String with task-specific runtime parameters
   *
   * \return True if initialization was successful
   */
  virtual bool initializeImpl(rclcpp::Node::SharedPtr node, const std::string &parameters) = 0;

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
   */
  virtual bool checkCompletion(double &progress) = 0;

  /**
   * \brief Helper function to parse a number from a string
   *
   * \param str The string to parse
   * \param value Reference to store the parsed value
   *
   * \return True if parsing was successful, false otherwise
   */
  bool parseParams(const std::string &str, int &value) const {
    try {
      value = std::stoi(str);
      return true;
    }
    catch (const std::invalid_argument &) {
      return false;
    }
    catch (const std::out_of_range &) {
      return false;
    }
  }

  bool parseParams(const std::string &str, double &value) const {
    try {
      value = std::stod(str);
      return true;
    }
    catch (const std::invalid_argument &) {
      return false;
    }
    catch (const std::out_of_range &) {
      return false;
    }
  }

  template <typename T>
  bool parseParams(const std::string &str, std::vector<T> &vec) const {
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
        RCLCPP_WARN_STREAM(node_->get_logger(), "[SubtaskExecutor]: Failed to parse parameter: " << item);
        return false;
      }
      vec.push_back(value);
    }
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  bool initialized_      = false;
  subtask_state_t state_ = subtask_state_t::IDLE;
  std::shared_ptr<iroc_mission_handler::msg::Subtask> subtask_; // Pointer to the subtask this executor is handling

  /**
   * \brief Validate subtask configuration
   */
  bool validateSubtaskConfig(const iroc_mission_handler::msg::Subtask &subtask) const {
    if (subtask.max_retries > 10) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[SubtaskExecutor]: Max retries (" << subtask.max_retries << ") is unusually high for subtask type: "
                                                                                 << subtask.type << ", parameters: " << subtask.parameters);
    }

    if (subtask.retry_delay < 0.0) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[SubtaskExecutor]: Retry delay cannot be negative: " << subtask.retry_delay << " for subtask type: "
                                                                                                    << subtask.type << ", parameters: " << subtask.parameters);
      return false;
    }

    if (subtask.retry_delay > 60.0) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[SubtaskExecutor]: Retry delay (" << subtask.retry_delay << ") is unusually long for subtask type: "
                                                                                 << subtask.type << ", parameters: " << subtask.parameters);
    }

    return true;
  }
};

} // namespace iroc_mission_handler
