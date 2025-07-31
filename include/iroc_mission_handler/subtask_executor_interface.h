#pragma once

#include <ros/ros.h>
#include <mrs_lib/param_loader.h>

#include "iroc_mission_handler/common_handlers.h"

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
  virtual bool initialize(const CommonHandlers& common_handlers, const std::string& parameters) = 0;

  /**
   * \brief Execute the subtask
   *
   * \return True if execution started successfully
   */
  virtual bool start() = 0;

  /**
   * \brief Check if the subtask has completed
   *
   * \param progress Reference to store the progress value (0.0-1.0)
   *
   * \return True if the subtask has completed
   */
  virtual bool isCompleted(double& progress) = 0;

  /**
   * \brief Stop the execution of the subtask
   *
   * \return True if the subtask was stopped successfully
   */
  virtual bool stop() = 0;

 protected:
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

  /**
   * \brief Check if executor is properly initialized
   */
  bool isInitialized() const {
    return initialized_;
  }

  /**
   * \brief Set initialization status
   */
  void setInitialized(bool status) {
    initialized_ = status;
  }

 private:
  bool initialized_ = false;
};

} // namespace iroc_mission_handler
