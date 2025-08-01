#include "iroc_mission_handler/executors/gimbal.h"

namespace iroc_mission_handler {
namespace executors {

bool GimbalExecutor::initialize(const CommonHandlers& common_handlers, const std::string& parameters) {
  // Load parameters
  mrs_lib::ParamLoader param_loader(common_handlers.nh, "SubtaskManager");
  param_loader.addYamlFileFromParam("executor_config");

  _orientation_tolerance_ = param_loader.loadParam2<double>("gimbal/orientation_tolerance", 0.01);
  _max_movement_time_ = param_loader.loadParam2<double>("gimbal/max_movement_time", 30.0);

  nh_ = common_handlers.nh;
  sh_opts_ = common_handlers.sh_opts;
  sh_opts_.autostart = false; // We will start manually

  // Parse gimbal control parameters from the parameters string
  std::vector<double> angles;
  if (!parseParams(parameters, angles) || angles.size() != 3) {
    ROS_ERROR_STREAM("[GimbalExecutor]: Invalid parameters format: " << parameters);
    return false;
  }

  // Set target angles
  target_roll_ = angles[0];
  target_pitch_ = angles[1];
  target_yaw_ = angles[2];

  // Initialize subscriber and service client
  sh_current_orientation_ = mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray>(sh_opts_, "in/servo_camera/orientation", // Remapped
                                                                                   &GimbalExecutor::orientationCallback, this);

  sc_set_gimbal_orientation_ = nh_.serviceClient<mrs_msgs::Vec4>("svc/servo_camera/set_orientation");

  setInitialized(true);
  ROS_DEBUG_STREAM("[GimbalExecutor]: Initialized with target angles - Roll: " << target_roll_ << ", Pitch: " << target_pitch_ << ", Yaw: " << target_yaw_);
  return true;
}

bool GimbalExecutor::start() {
  if (!isInitialized()) {
    ROS_ERROR("[GimbalExecutor]: Executor not initialized");
    return false;
  }

  // Wait for service to be available
  if (!sc_set_gimbal_orientation_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("[GimbalExecutor]: Gimbal orientation service not available");
    return false;
  }

  // Create and send gimbal command
  mrs_msgs::Vec4 srv;
  srv.request.goal[0] = target_roll_;
  srv.request.goal[1] = target_pitch_;
  srv.request.goal[2] = target_yaw_;

  if (!sc_set_gimbal_orientation_.call(srv)) {
    ROS_ERROR("[GimbalExecutor]: Failed to call gimbal orientation service");
    return false;
  }

  if (!srv.response.success) {
    ROS_ERROR_STREAM("[GimbalExecutor]: Gimbal service rejected command: " << srv.response.message);
    return false;
  }

  // Start orientation monitoring
  std::scoped_lock lock(mutex_);
  sh_current_orientation_.start();
  start_time_ = ros::Time::now();
  progress_ = 0.0;

  ROS_INFO_STREAM("[GimbalExecutor]: Started gimbal command - Roll: " << target_roll_ << ", Pitch: " << target_pitch_ << ", Yaw: " << target_yaw_);
  return true;
}

bool GimbalExecutor::isCompleted(double& progress) {
  if (!isInitialized()) {
    ROS_ERROR("[GimbalExecutor]: Executor not initialized");
    progress = 0.0;
    return false;
  }

  std::scoped_lock lock(mutex_);
  progress = progress_;
  return progress_ >= 1.0;
}

bool GimbalExecutor::stop() {
  if (!isInitialized()) {
    ROS_WARN("[GimbalExecutor]: Executor not initialized, nothing to stop");
    return true;
  }

  sh_current_orientation_.stop();
  ROS_INFO("[GimbalExecutor]: Stopped gimbal executor");
  return true;
}

void GimbalExecutor::orientationCallback(const std_msgs::Float32MultiArray::ConstPtr msg) {
  std::scoped_lock lock(mutex_);

  if (progress_ >= 1.0) {
    ROS_DEBUG("[GimbalExecutor]: Already completed, stopping orientation monitoring");
    sh_current_orientation_.stop();
    return;
  } else if (ros::Time::now() - start_time_ > ros::Duration(_max_movement_time_)) {
    ROS_WARN("[GimbalExecutor]: Maximum movement time exceeded, stopping orientation monitoring");
    sh_current_orientation_.stop();
    return;
  }

  if (msg->data.size() < 3) {
    ROS_WARN("[GimbalExecutor]: Received incomplete orientation data");
    return;
  }

  double current_roll = msg->data[0];
  double current_pitch = msg->data[1];
  double current_yaw = msg->data[2];

  // Initialize starting position if this is the first callback
  if (progress_ == 0.0) {
    initial_roll_ = current_roll;
    initial_pitch_ = current_pitch;
    initial_yaw_ = current_yaw;
  }

  // Calculate progress for each axis
  double roll_den = std::abs(target_roll_ - initial_roll_);
  double pitch_den = std::abs(target_pitch_ - initial_pitch_);
  double yaw_den = std::abs(target_yaw_ - initial_yaw_);

  double roll_progress = (roll_den > 1e-6) ? std::abs(current_roll - initial_roll_) / roll_den : 1.0;
  double pitch_progress = (pitch_den > 1e-6) ? std::abs(current_pitch - initial_pitch_) / pitch_den : 1.0;
  double yaw_progress = (yaw_den > 1e-6) ? std::abs(current_yaw - initial_yaw_) / yaw_den : 1.0;

  // Clamp progress values to [0, 1]
  roll_progress = std::min(roll_progress, 1.0);
  pitch_progress = std::min(pitch_progress, 1.0);
  yaw_progress = std::min(yaw_progress, 1.0);

  // Calculate overall progress as the average of individual axis progress
  progress_ = (roll_progress + pitch_progress + yaw_progress) / 3.0;

  // Check if target orientation is reached within tolerance
  if (std::abs(current_roll - target_roll_) < _orientation_tolerance_ &&   // Roll
      std::abs(current_pitch - target_pitch_) < _orientation_tolerance_ && // Pitch
      std::abs(current_yaw - target_yaw_) < _orientation_tolerance_) {     // Yaw
    progress_ = 1.0;
    ROS_INFO("[GimbalExecutor]: Target orientation reached");
  }

  ROS_DEBUG_STREAM("[GimbalExecutor]: Current: [" << current_roll << ", " << current_pitch << ", " << current_yaw << "] Target: [" << target_roll_ << ", "
                                                  << target_pitch_ << ", " << target_yaw_ << "] Progress: " << progress_);
}

} // namespace executors
} // namespace iroc_mission_handler
