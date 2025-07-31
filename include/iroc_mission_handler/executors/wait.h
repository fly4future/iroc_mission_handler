#pragma once

#include "iroc_mission_handler/subtask_executor_interface.h"

namespace iroc_mission_handler {
namespace executors {

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
  WaitExecutor() = default;
  virtual ~WaitExecutor() = default;

  bool initialize(const CommonHandlers& common_handlers, const std::string& parameters) override;

  bool start() override;
  bool stop() override;

  bool isCompleted(double& progress) override;

 private:
  ros::Timer timer_;

  ros::Time start_time_;
  double duration_ = 0.0;
  double elapsed_time_ = 0.0;

  /**
   * \brief Timer callback to check if the wait duration has elapsed
   * \param event Timer event
   */
  void timerCallback(const ros::TimerEvent& event);
};

} // namespace executors
} // namespace iroc_mission_handler

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_mission_handler::executors::WaitExecutor, iroc_mission_handler::SubtaskExecutor)