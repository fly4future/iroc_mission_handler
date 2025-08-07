#include "iroc_mission_handler/executors/wait.h"

namespace iroc_mission_handler {
namespace executors {

bool WaitExecutor::initializeImpl(const CommonHandlers& common_handlers, const std::string& parameters) {
  // Load parameters
  mrs_lib::ParamLoader param_loader(common_handlers.nh, "SubtaskManager");
  param_loader.addYamlFileFromParam("executor_config");

  double min_duration = param_loader.loadParam2<double>("wait/min_duration", 1.0);
  double max_duration = param_loader.loadParam2<double>("wait/max_duration", 300.0);
  double frequency = param_loader.loadParam2<double>("wait/timer_rate", 10.0);

  if (min_duration <= 0.0) {
    ROS_ERROR("[WaitExecutor]: Invalid min_duration, must be greater than 0.0");
    return false;
  }
  if (max_duration <= 0.0 || max_duration < min_duration) {
    ROS_ERROR("[WaitExecutor]: Invalid max_duration, must be greater than 0.0 and greater than min_duration");
    return false;
  }
  if (frequency <= 0.0) {
    ROS_ERROR("[WaitExecutor]: Invalid timer_rate, must be greater than 0.0");
    return false;
  }

  // Parse duration from parameters string
  if (!parseParams(parameters, duration_)) {
    ROS_ERROR_STREAM("[WaitExecutor]: Failed to parse duration from parameters: " << parameters);
    return false;
  }

  // Check if duration is valid
  if (duration_ < min_duration || duration_ > max_duration) {
    ROS_ERROR_STREAM("[WaitExecutor]: Duration must be between " << min_duration << " and " << max_duration << " seconds, got: " << duration_);
    return false;
  }

  // Create timer (will be started in `start()` method)
  ros::Rate rate(frequency);
  timer_ = common_handlers.nh.createTimer(rate, &WaitExecutor::timerCallback, this, false, false);

  ROS_DEBUG_STREAM("[WaitExecutor]: Initialized with duration: " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::startImpl() {
  start_time_ = ros::Time::now();
  elapsed_time_ = 0.0;
  timer_.start();

  ROS_INFO_STREAM("[WaitExecutor]: Started waiting for " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::checkCompletion(double& progress) {
  if (duration_ <= 0.0) {
    ROS_ERROR("[WaitExecutor]: Duration is not set or invalid");
    progress = 0.0;
    return false;
  }

  progress = std::min(elapsed_time_ / duration_, 1.0);
  return elapsed_time_ >= duration_;
}

bool WaitExecutor::stop() {
  if (timer_.hasStarted()) {
    timer_.stop();
    ROS_INFO("[WaitExecutor]: Stopped wait execution");
  } else {
    ROS_WARN("[WaitExecutor]: Wait was not started, nothing to stop");
  }

  return true;
}

void WaitExecutor::timerCallback([[maybe_unused]] const ros::TimerEvent& event) {
  elapsed_time_ = (ros::Time::now() - start_time_).toSec();

  if (elapsed_time_ >= duration_) {
    timer_.stop();
    ROS_INFO("[WaitExecutor]: Wait completed");
  }

  ROS_DEBUG_STREAM("[WaitExecutor]: Elapsed time: " << elapsed_time_ << "/" << duration_);
}

} // namespace executors
} // namespace iroc_mission_handler
