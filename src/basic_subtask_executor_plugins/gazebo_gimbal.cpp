#include "iroc_mission_handler/basic_subtask_executor_plugins/gazebo_gimbal.h"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::executors::basic_executors::GazeboGimbalExecutor, iroc_mission_handler::SubtaskExecutor)

namespace iroc_mission_handler
{
namespace executors
{
namespace basic_executors
{

bool GazeboGimbalExecutor::initializeImpl(rclcpp::Node::SharedPtr node, const std::string &parameters) {
  node_ = node;
  mrs_lib::ParamLoader param_loader(node_, "GazeboGimbalExecutor");

  // Load custom configuration if provided
  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("executor_config");

  // Load parameters
  param_loader.setPrefix("mission_handler/subtask_executors/");
  _orientation_tolerance_ = param_loader.loadParam2<double>("gimbal/orientation_tolerance", 0.01);
  _max_movement_time_     = param_loader.loadParam2<double>("gimbal/max_movement_time", 30.0);

  // Parse gimbal control parameters from the parameters string
  std::vector<double> angles;
  if (!parseParams(parameters, angles) || angles.size() != 3) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[GazeboGimbalExecutor]: Invalid parameters format: " << parameters);
    return false;
  }

  // Set target angles
  target_roll_  = angles[0];
  target_pitch_ = angles[1];
  target_yaw_   = angles[2];

  // Initialize subscriber and service client
  mrs_lib::SubscriberHandlerOptions sh_opts(node_);
  sh_opts.node_name          = "GazeboGimbalExecutor";
  sh_opts.no_message_timeout = rclcpp::Duration::from_seconds(5.0);
  sh_opts.threadsafe         = true;
  sh_opts.autostart          = false;
  sh_opts.qos                = rclcpp::SystemDefaultsQoS();

  sh_current_orientation_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray>(
      sh_opts, "in/servo_camera/orientation", [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { orientationCallback(msg); });

  sc_set_gimbal_orientation_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>(node_, "svc/servo_camera/set_orientation");

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[GazeboGimbalExecutor]: Initialized with target angles - Roll: " << target_roll_ << ", Pitch: " << target_pitch_
                                                                                                             << ", Yaw: " << target_yaw_);
  return true;
}

bool GazeboGimbalExecutor::startImpl() {
  // Wait for service to be available
  if (!sc_set_gimbal_orientation_.waitForService(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[GazeboGimbalExecutor]: Gimbal orientation service not available");
    return false;
  }

  // Create and send gimbal command
  auto req     = std::make_shared<mrs_msgs::srv::Vec4::Request>();
  req->goal[0] = target_roll_;
  req->goal[1] = target_pitch_;
  req->goal[2] = target_yaw_;
  req->goal[3] = 0.0;

  const auto resp = sc_set_gimbal_orientation_.callSync(req);
  if (!resp.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "[GazeboGimbalExecutor]: Failed to call gimbal orientation service");
    return false;
  }

  if (!resp.value()->success) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[GazeboGimbalExecutor]: Gimbal orientation service failed: " << resp.value()->message);
    return false;
  }

  // Start orientation monitoring
  std::scoped_lock lock(mutex_);
  start_time_ = node_->now();
  sh_current_orientation_.start();
  progress_ = 0.0;

  RCLCPP_INFO_STREAM(node_->get_logger(), "[GazeboGimbalExecutor]: Started gimbal command - Roll: " << target_roll_ << ", Pitch: " << target_pitch_
                                                                                                    << ", Yaw: " << target_yaw_
                                                                                                    << ". Start time: " << start_time_.seconds());
  return true;
}

bool GazeboGimbalExecutor::checkCompletion(double &progress) {
  std::scoped_lock lock(mutex_);
  progress = progress_;

  if ((node_->now() - start_time_).seconds() > _max_movement_time_) {
    return true; // Consider it completed if max movement time exceeded
  }

  return progress_ >= 1.0;
}

bool GazeboGimbalExecutor::stop() {
  sh_current_orientation_.stop();
  RCLCPP_INFO(node_->get_logger(), "[GazeboGimbalExecutor]: Stopped gimbal executor");
  return true;
}

void GazeboGimbalExecutor::orientationCallback(std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
  std::scoped_lock lock(mutex_);

  if (progress_ >= 1.0) {
    RCLCPP_DEBUG(node_->get_logger(), "[GazeboGimbalExecutor]: Already completed, stopping orientation monitoring");
    sh_current_orientation_.stop();
    return;
  } else if ((node_->now() - start_time_).seconds() > _max_movement_time_) {
    RCLCPP_WARN(node_->get_logger(), "[GazeboGimbalExecutor]: Maximum movement time exceeded, stopping orientation monitoring");
    sh_current_orientation_.stop();
    return;
  }

  if (msg->data.size() < 3) {
    RCLCPP_WARN(node_->get_logger(), "[GazeboGimbalExecutor]: Received incomplete orientation data");
    return;
  }

  double current_roll  = msg->data[0];
  double current_pitch = msg->data[1];
  double current_yaw   = msg->data[2];

  // Initialize starting position if this is the first callback
  if (progress_ == 0.0) {
    initial_roll_  = current_roll;
    initial_pitch_ = current_pitch;
    initial_yaw_   = current_yaw;
  }

  // Calculate progress for each axis
  double roll_den  = std::abs(target_roll_ - initial_roll_);
  double pitch_den = std::abs(target_pitch_ - initial_pitch_);
  double yaw_den   = std::abs(target_yaw_ - initial_yaw_);

  double roll_progress  = (roll_den > 1e-6) ? std::abs(current_roll - initial_roll_) / roll_den : 1.0;
  double pitch_progress = (pitch_den > 1e-6) ? std::abs(current_pitch - initial_pitch_) / pitch_den : 1.0;
  double yaw_progress   = (yaw_den > 1e-6) ? std::abs(current_yaw - initial_yaw_) / yaw_den : 1.0;

  // Clamp progress values to [0, 1]
  roll_progress  = std::min(roll_progress, 1.0);
  pitch_progress = std::min(pitch_progress, 1.0);
  yaw_progress   = std::min(yaw_progress, 1.0);

  // Calculate overall progress as the average of individual axis progress
  progress_ = (roll_progress + pitch_progress + yaw_progress) / 3.0;

  // Check if target orientation is reached within tolerance
  if (std::abs(current_roll - target_roll_) < _orientation_tolerance_ &&   // Roll
      std::abs(current_pitch - target_pitch_) < _orientation_tolerance_ && // Pitch
      std::abs(current_yaw - target_yaw_) < _orientation_tolerance_) {     // Yaw
    progress_ = 1.0;
    RCLCPP_INFO(node_->get_logger(), "[GazeboGimbalExecutor]: Target orientation reached");
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[GazeboGimbalExecutor]: Current: [" << current_roll << ", " << current_pitch << ", " << current_yaw << "] Target: ["
                                                                                << target_roll_ << ", " << target_pitch_ << ", " << target_yaw_
                                                                                << "] Progress: " << progress_);
}

} // namespace basic_executors
} // namespace executors
} // namespace iroc_mission_handler
