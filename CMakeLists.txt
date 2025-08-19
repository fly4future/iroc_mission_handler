cmake_minimum_required(VERSION 3.15.0)
project(iroc_mission_handler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  nodelet
  mrs_msgs
  std_srvs
  mrs_lib
  actionlib_msgs
  mrs_robot_diagnostics
  message_generation
)

set(LIBRARIES
  IROCMissionHandler
  IROCMIssionHandler_BasicSubtaskExecutors
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

add_action_files(
  DIRECTORY action FILES
  Mission.action
)

add_message_files(
  DIRECTORY msg
  FILES
  Subtask.msg
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs mrs_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

# Executors
add_library(IROCMIssionHandler_BasicSubtaskExecutors
  src/executors/gimbal.cpp
  src/executors/wait.cpp
)
add_dependencies(IROCMIssionHandler_BasicSubtaskExecutors
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(IROCMIssionHandler_BasicSubtaskExecutors
  ${catkin_LIBRARIES}
)

# Mission Handler
add_library(IROCMissionHandler
  src/mission_handler.cpp
  src/subtask_manager.cpp
)
add_dependencies(IROCMissionHandler
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(IROCMissionHandler
  ${catkin_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
