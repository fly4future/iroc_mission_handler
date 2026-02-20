#pragma once

#include "iroc_mission_handler/subtask_executor_interface.h"

namespace iroc_mission_handler {
namespace executors {
namespace basic_executors {

/**
 * \brief Wait subtask executor - simply waits for a specified duration
 *
 * This executor implements a simple wait functionality that blocks execution
 * for a specified amount of time. It's useful for creating delays in mission sequences.
 *
 * Parameters format: "duration_in_seconds"
 * Example: "5.0" (wait for 5 seconds)
 */
class WaitExecutor : public SubtaskExecutor {
 public:
  WaitExecutor()          = default;
  virtual ~WaitExecutor() = default;

  bool stop() override;

 protected:
  bool initializeImpl(rclcpp::Node::SharedPtr node, const std::string& parameters) override;
  bool startImpl() override;
  bool checkCompletion(double& progress) override;

 private:
  rclcpp::Node::SharedPtr     node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::nanoseconds     timer_period_;

  rclcpp::Time start_time_;
  double       duration_     = 0.0;
  double       elapsed_time_ = 0.0;

  /**
   * \brief Timer callback to check if the wait duration has elapsed
   */
  void timerCallback();
};

} // namespace basic_executors
} // namespace executors
} // namespace iroc_mission_handler
