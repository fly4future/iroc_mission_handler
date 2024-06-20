cmake_minimum_required(VERSION 3.15.0)
project(mrs_mission_manager)

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
  genmsg
  mrs_robot_diagnostics
  )

set(LIBRARIES
  MRSMissionManager
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

add_action_files(
  DIRECTORY action 
  FILES waypointMission.action
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

## | ------------ MRSRobotDiagnostics_StateMonitor ------------ |

## Declare a C++ library
add_library(MRSMissionManager
  src/mission_manager.cpp
  )

add_dependencies(MRSMissionManager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MRSMissionManager
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
