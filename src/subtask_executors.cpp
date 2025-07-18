#include "iroc_mission_handler/subtask_executors.h"

namespace iroc_mission_handler {
// | -------------------- WaitSubtaskExecutor -------------------- |

WaitSubtaskExecutor::WaitSubtaskExecutor(const ros::NodeHandle& nh, const double freq) : nh_(nh) {
  ros::Rate rate(freq);
  timer_ = nh_.createTimer(rate, &WaitSubtaskExecutor::timerCallback, this, false, false); // Create a timer that will not start immediately

  ROS_DEBUG("[WaitSubtaskExecutor]: Initialized");
}

void WaitSubtaskExecutor::timerCallback([[maybe_unused]] const ros::TimerEvent& event) {
  double elapsed_time = (ros::Time::now() - start_time_).toSec();

  if (elapsed_time >= duration_) {
    timer_.stop();
    ROS_INFO("[WaitSubtaskExecutor]: Wait completed");
  }

  elapsed_time_ = elapsed_time;
  ROS_DEBUG_STREAM("[WaitSubtaskExecutor]: Elapsed time: " << elapsed_time_);
}

bool WaitSubtaskExecutor::execute(const std::string& parameters) {
  // Parse duration from parameters string
  if (!parseParams(parameters, duration_)) {
    ROS_ERROR_STREAM("[WaitSubtaskExecutor]: Invalid duration parameter: " << parameters);
    return false;
  }

  if (duration_ <= 0.0) {
    ROS_ERROR_STREAM("[WaitSubtaskExecutor]: Duration must be positive: " << duration_);
    return false;
  }

  start_time_ = ros::Time::now();
  timer_.start(); // Start the timer

  ROS_INFO_STREAM("[WaitSubtaskExecutor]: Started waiting for " << duration_ << " seconds");
  return true;
}

bool WaitSubtaskExecutor::isCompleted(double& progress) {
  if (duration_ <= 0.0) {
    ROS_ERROR("[WaitSubtaskExecutor]: Duration is not set or invalid");
    return false;
  }

  progress = std::min(elapsed_time_ / duration_, 1.0);
  if (elapsed_time_ >= duration_) {
    return true;
  }

  return false;
}

bool WaitSubtaskExecutor::stop() {
  if (timer_.hasStarted()) {
    ROS_INFO("[WaitSubtaskExecutor]: Stopping wait");
  } else {
    ROS_WARN("[WaitSubtaskExecutor]: Wait was not started, nothing to stop");
  }

  timer_.stop();
  return true;
}

// | -------------------- GimbalSubtaskExecutor -------------------- |

GimbalSubtaskExecutor::GimbalSubtaskExecutor(const ros::NodeHandle& nh, const mrs_lib::SubscribeHandlerOptions& sh_opts) : nh_(nh) {
  sh_opts_ = sh_opts;
  sh_opts_.autostart = false; // We will start the SubscribeHandler manually
  sh_current_orientation_ = mrs_lib::SubscribeHandler<std_msgs::Float32MultiArray>(sh_opts_, "in/servo_camera/orientation",
                                                                                   &GimbalSubtaskExecutor::orientationCallback, this); // Gimbal topic

  sc_set_gimbal_orientation_ = nh_.serviceClient<mrs_msgs::Vec4>("svc/servo_camera/set_orientation");

  ROS_DEBUG("[GimbalSubtaskExecutor]: Initialized");
}

bool GimbalSubtaskExecutor::execute(const std::string& parameters) {
  // Parse gimbal control parameters from the parameters string
  std::vector<double> angles;
  if (!parseParams(parameters, angles) || angles.size() != 3) {
    ROS_ERROR_STREAM("[GimbalSubtaskExecutor]: Invalid parameters format: " << parameters);
    return false;
  }

  // Set target angles
  target_roll_ = angles[0];
  target_pitch_ = angles[1];
  target_yaw_ = angles[2];

  // Create and publish the gimbal command
  mrs_msgs::Vec4::Request req;
  req.goal[0] = target_roll_;
  req.goal[1] = target_pitch_;
  req.goal[2] = target_yaw_;

  mrs_msgs::Vec4::Response res;
  if (sc_set_gimbal_orientation_.call(req, res)) {
    if (!res.success) {
      ROS_ERROR_STREAM("[GimbalSubtaskExecutor]: Failed to set gimbal orientation: " << res.message);
      return false;
    }

    ROS_INFO_STREAM("[GimbalSubtaskExecutor]: Executing gimbal command: " << req.goal[0] << ", " << req.goal[1] << ", " << req.goal[2]);

    sh_current_orientation_.start();
    return true;
  } else {
    ROS_ERROR("[GimbalSubtaskExecutor]: Failed to call service to set gimbal orientation");
    return false;
  }
}

bool GimbalSubtaskExecutor::isCompleted(double& progress) {
  std::scoped_lock lock(mutex_);
  progress = progress_;

  if (progress_ >= 1.0) {
    ROS_DEBUG("[GimbalSubtaskExecutor]: Gimbal command already completed");
    return true;
  }
  return false;
}

bool GimbalSubtaskExecutor::stop() {
  // Stop the gimbal command (dummy implementation)
  ROS_INFO("[GimbalSubtaskExecutor]: Stopped gimbal command");
  sh_current_orientation_.stop();
  return true;
}

void GimbalSubtaskExecutor::orientationCallback(const std_msgs::Float32MultiArray::ConstPtr msg) {
  std::scoped_lock lock(mutex_);

  if (progress_ >= 1.0) {
    ROS_DEBUG("[GimbalSubtaskExecutor]: Gimbal command already completed, skipping orientation update");
    sh_current_orientation_.stop();
    return;
  }

  // Check if the gimbal has reached the target orientation
  if (msg->data.size() >= 3) {
    if (progress_ == 0.0) {
      initial_roll_ = msg->data[0];
      initial_pitch_ = msg->data[1];
      initial_yaw_ = msg->data[2];
    }

    double current_roll = msg->data[0];
    double current_pitch = msg->data[1];
    double current_yaw = msg->data[2];

    double roll_den = std::abs(target_roll_ - initial_roll_);
    double pitch_den = std::abs(target_pitch_ - initial_pitch_);
    double yaw_den = std::abs(target_yaw_ - initial_yaw_);

    double roll_progress = (roll_den > 1e-6) ? std::abs(current_roll - initial_roll_) / roll_den : 1.0;
    double pitch_progress = (pitch_den > 1e-6) ? std::abs(current_pitch - initial_pitch_) / pitch_den : 1.0;
    double yaw_progress = (yaw_den > 1e-6) ? std::abs(current_yaw - initial_yaw_) / yaw_den : 1.0;

    // Calculate overall progress as the average of individual axis progress
    progress_ = (roll_progress + pitch_progress + yaw_progress) / 3.0;

    if (std::abs(current_pitch - target_pitch_) < GIMBAL_ORIENTATION_TOLERANCE && // Pitch
        std::abs(current_yaw - target_yaw_) < GIMBAL_ORIENTATION_TOLERANCE &&     // Yaw
        std::abs(current_roll - target_roll_) < GIMBAL_ORIENTATION_TOLERANCE) {   // Roll

      // Mark as completed
      progress_ = 1.0;
      ROS_INFO("[GimbalSubtaskExecutor]: Gimbal reached target orientation");
    }

    ROS_DEBUG("[GimbalSubtaskExecutor]: Current orientation: %f, %f, %f | Target: %f, %f, %f | Progress: %f", current_roll, current_pitch, current_yaw,
              target_roll_, target_pitch_, target_yaw_, progress_);
  } else {
    ROS_WARN("[GimbalSubtaskExecutor]: Received incomplete orientation data");
  }
}

} // namespace iroc_mission_handler
