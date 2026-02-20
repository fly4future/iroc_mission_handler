#pragma once

#include <std_msgs/msg/float32_multi_array.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_lib/service_client_handler.h>

#include <mutex>

#include "iroc_mission_handler/subtask_executor_interface.h"

namespace iroc_mission_handler {
namespace executors {
namespace basic_executors {

/**
 * \brief Gimbal subtask executor - controls a gimbal device
 *
 * This executor controls a gimbal by sending orientation commands and monitoring
 * the current orientation to determine when the target position is reached.
 *
 * Parameters format: "roll,pitch,yaw"
 * Example: "[0.0,1.57,0.707]" (roll=0, pitch=90°, yaw=45°)
 */
class GazeboGimbalExecutor : public SubtaskExecutor {
 public:
  GazeboGimbalExecutor()          = default;
  virtual ~GazeboGimbalExecutor() = default;

  bool stop() override;

 protected:
  bool initializeImpl(rclcpp::Node::SharedPtr node, const std::string& parameters) override;
  bool startImpl() override;
  bool checkCompletion(double& progress) override;

 private:
  rclcpp::Node::SharedPtr node_;

  mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray> sh_current_orientation_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>           sc_set_gimbal_orientation_;

  // Tolerance for orientation matching
  double _orientation_tolerance_;
  double _max_movement_time_; // This prevents infinite waiting if the gimbal does not reach the target orientation

  double       progress_ = 0.0;
  rclcpp::Time start_time_;
  std::mutex   mutex_;

  // Gimbal control parameters
  double target_roll_  = 0.0;
  double target_pitch_ = 0.0;
  double target_yaw_   = 0.0;

  double initial_roll_  = 0.0;
  double initial_pitch_ = 0.0;
  double initial_yaw_   = 0.0;

  /**
   * \brief Callback for receiving current gimbal orientation
   * \param msg The received message containing the current orientation
   */
  void orientationCallback(std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
};

} // namespace basic_executors
} // namespace executors
} // namespace iroc_mission_handler
