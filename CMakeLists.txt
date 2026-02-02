cmake_minimum_required(VERSION 3.28)
project(iroc_mission_handler)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[iroc_fleet_manager]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# Set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")
  add_definitions("-O1")
  add_definitions("-g")
endif()

## --------------------------------------------------------------
## |                    Find Dependencies                       |
## --------------------------------------------------------------

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  pluginlib
  std_msgs
  geometry_msgs
  builtin_interfaces
  action_msgs
  mrs_msgs
  mrs_lib
  mrs_robot_diagnostics
  rosidl_default_generators
)

set(LIBRARIES
  IROCMissionHandler
  # TODO to update later 
  # IROCMissionHandler_BasicSubtaskExecutors
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                      Generate Interfaces                   |
## --------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Subtask.msg"
  "msg/Waypoint.msg"
  "action/Mission.action"
  DEPENDENCIES 
    builtin_interfaces 
    std_msgs 
    geometry_msgs 
    mrs_msgs 
)

## --------------------------------------------------------------
## |                      Interface Library                     |
## --------------------------------------------------------------

add_library(IrocMissionHandler_IrocMissionHandler INTERFACE)
add_library(iroc_mission_handler::iroc_mission_handler ALIAS IrocMissionHandler_IrocMissionHandler)

set_target_properties(IrocMissionHandler_IrocMissionHandler
  PROPERTIES
    EXPORT_NAME iroc_mission_handler 
    OUTPUT_NAME iroc_mission_handler 
)

target_sources(IrocMissionHandler_IrocMissionHandler
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/iroc_mission_handler/subtask_executor_interface.h"
    "include/iroc_mission_handler/subtask_manager.h"
    "include/iroc_mission_handler/enums/mission_state.h"
    "include/iroc_mission_handler/enums/subtask_state.h"
)

target_link_libraries(IrocMissionHandler_IrocMissionHandler
  INTERFACE
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# ## | ------------ IROCFleetManager ------------ |
#
# add_library(IROCFleetManager SHARED
#   src/fleet_manager.cpp
#   # src/utils/json_var_parser.cpp
# )
#
# target_link_libraries(IROCFleetManager
#   PRIVATE
#     iroc_fleet_manager::iroc_fleet_manager
#     rclcpp::rclcpp
#     rclcpp_action::rclcpp_action
#     rclcpp_components::component
#     pluginlib::pluginlib
#     mrs_lib::mrs_lib
#     mrs_robot_diagnostics::mrs_robot_diagnostics
#     ${mrs_msgs_TARGETS}
# )
#
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(IROCFleetManager PRIVATE ${cpp_typesupport_target})
#
# rclcpp_components_register_nodes(IROCFleetManager "iroc_fleet_manager::IROCFleetManager")
# target_compile_definitions(IROCFleetManager PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Mission Handler
add_library(IROCMissionHandler SHARED
  src/mission_handler.cpp
)

target_link_libraries(IROCMissionHandler
  PRIVATE
    iroc_mission_handler::iroc_mission_handler
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_components::component
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_robot_diagnostics::mrs_robot_diagnostics
    ${mrs_msgs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(IROCMissionHandler PRIVATE ${cpp_typesupport_target})

rclcpp_components_register_nodes(IROCMissionHandler "iroc_mission_handler::IROCMissionHandler")

target_compile_definitions(IROCMissionHandler PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})


# TODO to update later
# # Executors
# add_library(IROCMissionHandler_BasicSubtaskExecutors
#   src/basic_subtask_executor_plugins/gazebo_gimbal.cpp
#   src/basic_subtask_executor_plugins/wait.cpp
# )
# add_dependencies(IROCMissionHandler_BasicSubtaskExecutors
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# target_link_libraries(IROCMissionHandler_BasicSubtaskExecutors
#   ${catkin_LIBRARIES}
# )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    IrocMissionHandler_IrocMissionHandler
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

## --------------------------------------------------------------
## |                           Export                           |
## --------------------------------------------------------------

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
