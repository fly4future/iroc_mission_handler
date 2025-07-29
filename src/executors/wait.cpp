#include "iroc_mission_handler/executors/wait.h"

namespace iroc_mission_handler {
namespace executors {

bool WaitExecutor::initialize(const CommonHandlers& common_handlers, const std::string& parameters) {
  if (!validateParameters(parameters)) {
    ROS_ERROR_STREAM("[WaitExecutor]: Invalid parameters: " << parameters);
    return false;
  }

  // Parse duration from parameters string
  if (!parseParams(parameters, duration_)) {
    ROS_ERROR_STREAM("[WaitExecutor]: Failed to parse duration from parameters: " << parameters);
    return false;
  }

  if (duration_ <= 0.0) {
    ROS_ERROR_STREAM("[WaitExecutor]: Duration must be positive: " << duration_);
    return false;
  }

  // Create timer (will be started in start() method)
  ros::Rate rate(10.0);
  timer_ = common_handlers.nh.createTimer(rate, &WaitExecutor::timerCallback, this, false, false);

  setInitialized(true);
  ROS_DEBUG_STREAM("[WaitExecutor]: Initialized with duration: " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::validateParameters(const std::string& parameters) const {
  double test_duration;
  return parseParams(parameters, test_duration) && test_duration > 0.0;
}

bool WaitExecutor::start() {
  if (!isInitialized()) {
    ROS_ERROR("[WaitExecutor]: Executor not initialized");
    return false;
  }

  start_time_ = ros::Time::now();
  elapsed_time_ = 0.0;
  timer_.start();

  ROS_INFO_STREAM("[WaitExecutor]: Started waiting for " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::isCompleted(double& progress) {
  if (!isInitialized()) {
    ROS_ERROR("[WaitExecutor]: Executor not initialized");
    progress = 0.0;
    return false;
  }

  if (duration_ <= 0.0) {
    ROS_ERROR("[WaitExecutor]: Duration is not set or invalid");
    progress = 0.0;
    return false;
  }

  progress = std::min(elapsed_time_ / duration_, 1.0);
  return elapsed_time_ >= duration_;
}

bool WaitExecutor::stop() {
  if (!isInitialized()) {
    ROS_WARN("[WaitExecutor]: Executor not initialized, nothing to stop");
    return true;
  }

  if (timer_.hasStarted()) {
    timer_.stop();
    ROS_INFO("[WaitExecutor]: Stopped wait execution");
  } else {
    ROS_WARN("[WaitExecutor]: Wait was not started, nothing to stop");
  }

  return true;
}

void WaitExecutor::timerCallback(const ros::TimerEvent& event) {
  elapsed_time_ = (ros::Time::now() - start_time_).toSec();

  if (elapsed_time_ >= duration_) {
    timer_.stop();
    ROS_INFO("[WaitExecutor]: Wait completed");
  }

  ROS_DEBUG_STREAM("[WaitExecutor]: Elapsed time: " << elapsed_time_ << "/" << duration_);
}

} // namespace executors
} // namespace iroc_mission_handler
