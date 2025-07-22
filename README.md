# iroc_mission_handler

ROS (Robot Operating System) action server component that implements drone flight control. Specifically, it handles requests to navigate a drone through a defined sequence of waypoints. It allows users to define a series of waypoints and then controls the drone to visit each point in order.

## State machine

The state machine for the iroc_mission_handler is designed to manage the drone's flight path through a series of waypoints but taking into account the subtasks involved in reaching each trajectory final point.

```mermaid
---
title: State Machine Diagram
---
stateDiagram-v2
  [*] --> Idle
  Idle --> ML: Action server goal received
  ML --> Executing: Start mission service

  Executing --> Paused: Pause mission service
  Paused --> Executing: Start mission service
  Executing --> Idle: Stop mission service

  state if_state <<choice>>
  Executing --> if_state: Segment finished
  if_state --> Executing: subtasks <= 0
  if_state --> ST : subtasks > 0
  ST --> Executing: Subtask finished
  ST --> ST: Not finished
  note left of ST: Subtasks are initiated sequentially<br>but can run in parallel<br>if the subtask executor supports it.

  Executing --> Finished: No remaining segments
  Finished --> Idle: Terminal action finished


  ML: Mission Loaded
  ST: Executing Subtask
```
