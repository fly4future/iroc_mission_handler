#pragma once

#include <ros/ros.h>
#include <mrs_lib/subscribe_handler.h>

#include <mutex>

// Gimbal subtask executor
#include <std_msgs/Float32MultiArray.h>
#include <mrs_msgs/Vec4.h>

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
  bool isCompleted() {
    double progress = 0.0;
    return isCompleted(progress);
  }

  /**
   * \brief Helper function to parse a number from a string
   * \param str The string to parse
   * \param value Reference to store the parsed value
   * \return True if parsing was successful, false otherwise
   */
  bool parseParams(const std::string& str, int& value) {
    try {
      value = std::stoi(str);
      return true;
    } catch (const std::invalid_argument&) {
      return false;
    }
  }

  bool parseParams(const std::string& str, double& value) {
    try {
      value = std::stod(str);
      return true;
    } catch (const std::invalid_argument& e) {
      return false;
    }
  }

  template <typename T>
  bool parseParams(const std::string& str, std::vector<T>& vec) {
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
        ROS_ERROR_STREAM("Failed to parse parameter: " << item);
        return false;
      }
      vec.push_back(value);
    }
    return true;
  }
};

/**
 * \brief Wait subtask executor - simply waits for a specified duration
 */
class WaitSubtaskExecutor : public SubtaskExecutorBase {
 public:
  /**
   * \brief Constructor for WaitSubtaskExecutor
   * \param nh ROS NodeHandle for communication
   * \param freq Frequency at which to check the elapsed time (default is 1.0 Hz)
   * \note The frequency is used to set the timer for checking the elapsed time
   *       and can be adjusted based on the required precision.
   */
  WaitSubtaskExecutor(const ros::NodeHandle& nh, const double freq = 1.0);

  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Time start_time_;
  double duration_ = 0.0;
  double elapsed_time_ = 0.0;

  /**
   * \brief Timer callback to check if the wait duration has elapsed
   * \param event Timer event
   */
  void timerCallback([[maybe_unused]] const ros::TimerEvent& event);
};

/**
 * \brief Gimbal subtask executor - controls a gimbal device
 */
class GimbalSubtaskExecutor : public SubtaskExecutorBase {
 public:
  // Tolerance for orientation matching
  static constexpr double GIMBAL_ORIENTATION_TOLERANCE = 0.01;

  /**
   * \brief Constructor for GimbalSubtaskExecutor
   * \param nh ROS NodeHandle for communication
   * \param sh_opts SubscribeHandler options for subscribing to gimbal orientation updates
   * \note The SubscribeHandler will not be started automatically, so `autostart` in `sh_opts` is not taken into account.
   */
  GimbalSubtaskExecutor(const ros::NodeHandle& nh, const mrs_lib::SubscribeHandlerOptions& sh_opts);

  /**
   * \brief Execute the gimbal subtask
   * \param parameters String with gimbal control parameters (format: "roll,pitch,yaw")
   * \return True if execution started successfully
   * \note The parameters string should contain the target pitch, yaw, and roll angles separated by commas.
   *       Example: "1.57,0.707,0.0"
   *       If the string is not in the correct format, an error will be logged and the method will return false.
   */
  bool execute(const std::string& parameters) override;
  bool isCompleted(double& progress) override;
  bool stop() override;

 private:
  ros::NodeHandle nh_;
  mrs_lib::SubscribeHandlerOptions sh_opts_;
  std::string base_topic_ = "/uav1/servo_camera";

  mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray> sh_current_orientation_;
  ros::ServiceClient sc_set_gimbal_orientation_;

  double progress_ = 0.0;
  std::mutex mutex_;

  // Gimbal control parameters
  double target_pitch_ = 0.0;
  double target_yaw_ = 0.0;
  double target_roll_ = 0.0;

  double initial_pitch_ = 0.0;
  double initial_yaw_ = 0.0;
  double initial_roll_ = 0.0;

  /**
   * \brief Callback for receiving current gimbal orientation. It will be used to check if the gimbal has reached the target orientation.
   *
   * \param msg The received message containing the current orientation
   */
  void orientationCallback(const std_msgs::Float32MultiArray::ConstPtr msg);
};

} // namespace iroc_mission_handler
