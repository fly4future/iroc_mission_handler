#pragma once

#include <std_msgs/Float32MultiArray.h>
#include <mrs_msgs/Vec4.h>

#include <mutex>

#include "iroc_mission_handler/subtask_executor_interface.h"

namespace iroc_mission_handler {
namespace executors {

/**
 * \brief Gimbal subtask executor - controls a gimbal device
 *
 * This executor controls a gimbal by sending orientation commands and monitoring
 * the current orientation to determine when the target position is reached.
 *
 * Parameters format: "roll,pitch,yaw"
 * Example: "[0.0,1.57,0.707]" (roll=0, pitch=90°, yaw=45°)
 */
class GimbalExecutor : public SubtaskExecutor {
 public:
  // Tolerance for orientation matching
  static constexpr double GIMBAL_ORIENTATION_TOLERANCE = 0.01;

  GimbalExecutor() = default;
  virtual ~GimbalExecutor() = default;

  bool initialize(const CommonHandlers& common_handlers, const std::string& parameters) override;

  bool start() override;
  bool stop() override;

  bool isCompleted(double& progress) override;

 private:
  ros::NodeHandle nh_;
  mrs_lib::SubscribeHandlerOptions sh_opts_;

  mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray> sh_current_orientation_;
  ros::ServiceClient sc_set_gimbal_orientation_;

  double progress_ = 0.0;
  std::mutex mutex_;

  // Gimbal control parameters
  double target_roll_ = 0.0;
  double target_pitch_ = 0.0;
  double target_yaw_ = 0.0;

  double initial_roll_ = 0.0;
  double initial_pitch_ = 0.0;
  double initial_yaw_ = 0.0;

  /**
   * \brief Callback for receiving current gimbal orientation
   * \param msg The received message containing the current orientation
   */
  void orientationCallback(const std_msgs::Float32MultiArray::ConstPtr msg);
};

} // namespace executors
} // namespace iroc_mission_handler

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::executors::GimbalExecutor, iroc_mission_handler::SubtaskExecutor)