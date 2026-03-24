#include "iroc_mission_handler/basic_subtask_executor_plugins/wait.h"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::executors::basic_executors::WaitExecutor, iroc_mission_handler::SubtaskExecutor)

namespace iroc_mission_handler {
namespace executors {
namespace basic_executors {

bool WaitExecutor::initializeImpl(rclcpp::Node::SharedPtr node, const std::string& parameters) {
  node_ = node;
  mrs_lib::ParamLoader param_loader(node_, "WaitExecutor");

  // Load custom configuration if provided
  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("executor_config");

  // Load parameters
  param_loader.setPrefix("mission_handler/subtask_executors/");
  double min_duration = param_loader.loadParam2<double>("wait/min_duration", 1.0);
  double max_duration = param_loader.loadParam2<double>("wait/max_duration", 300.0);
  double frequency    = param_loader.loadParam2<double>("wait/timer_rate", 10.0);

  if (min_duration <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitExecutor]: Invalid min_duration, must be greater than 0.0");
    return false;
  }
  if (max_duration <= 0.0 || max_duration < min_duration) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitExecutor]: Invalid max_duration, must be greater than 0.0 and greater than min_duration");
    return false;
  }
  if (frequency <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitExecutor]: Invalid timer_rate, must be greater than 0.0");
    return false;
  }

  // Parse duration from parameters string
  if (!parseParams(parameters, duration_)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[WaitExecutor]: Failed to parse duration from parameters: " << parameters);
    return false;
  }

  // Check if duration is valid.
  // std::isnan() guard is required because IEEE 754 NaN comparisons always
  // return false, so "NaN < min || NaN > max" evaluates to false and NaN
  // would silently pass without the explicit check.
  if (std::isnan(duration_) || duration_ < min_duration || duration_ > max_duration) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[WaitExecutor]: Duration must be between " << min_duration << " and " << max_duration << " seconds, got: " << duration_);
    return false;
  }

  // Store timer period for later use in startImpl()
  timer_period_ = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / frequency));

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[WaitExecutor]: Initialized with duration: " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::startImpl() {
  start_time_   = node_->now();
  elapsed_time_ = 0.0;
  timer_        = node_->create_wall_timer(timer_period_, [this]() { timerCallback(); });

  RCLCPP_INFO_STREAM(node_->get_logger(), "[WaitExecutor]: Started waiting for " << duration_ << " seconds");
  return true;
}

bool WaitExecutor::checkCompletion(double& progress) {
  if (duration_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitExecutor]: Duration is not set or invalid");
    progress = 0.0;
    return false;
  }

  progress = std::min(elapsed_time_ / duration_, 1.0);
  return elapsed_time_ >= duration_;
}

bool WaitExecutor::stop() {
  // node_ is only set in initializeImpl(), so guard against it being null
  // (e.g., stop() called before initialize() during mission rollback).
  auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("WaitExecutor");

  if (timer_) {
    timer_->cancel();
    RCLCPP_INFO(logger, "[WaitExecutor]: Stopped wait execution");
  } else {
    RCLCPP_WARN(logger, "[WaitExecutor]: Wait was not started, nothing to stop");
  }

  return true;
}

void WaitExecutor::timerCallback() {
  elapsed_time_ = (node_->now() - start_time_).seconds();

  if (elapsed_time_ >= duration_) {
    timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "[WaitExecutor]: Wait completed");
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[WaitExecutor]: Elapsed time: " << elapsed_time_ << "/" << duration_);
}

} // namespace basic_executors
} // namespace executors
} // namespace iroc_mission_handler
