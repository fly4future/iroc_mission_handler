#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iroc_mission_handler/waypointMissionAction.h>


typedef iroc_mission_handler::waypointMissionGoal ActionServerGoal;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_mission_handler");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<iroc_mission_handler::waypointMissionAction> ac("waypoint_mission_action", true);

  ROS_INFO("[MissionHandlerTestClient]: Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("[MissionHandlerTestClient]: Action server started, sending goal.");

  // send a goal to the action
  std::vector<mrs_msgs::Reference> points;
  mrs_msgs::Reference             goal;

  //LOCAL POINTS
  /* goal.position.x = 20; */
  /* goal.position.y = 0; */
  /* goal.position.z = 3; */
  /* goal.heading    = 2; */
  /* points.push_back(goal); */
  /* goal.position.x = -10; */
  /* goal.position.y = 0; */
  /* goal.position.z = 3; */
  /* goal.heading    = 1.5; */
  /* points.push_back(goal); */

  //LATLON POINTS
  goal.position.x = 47.397745;
  goal.position.y = 8.545596;
  goal.position.z = 3;
  goal.heading    = 2;
  points.push_back(goal);
  goal.position.x = 47.397638;
  goal.position.y = 8.545487;
  goal.position.z = 3;
  goal.heading    = 1.5;
  points.push_back(goal);
  goal.position.x = 47.397724;
  goal.position.y = 8.545204;
  goal.position.z = 3;
  goal.heading    = 1.5;
  points.push_back(goal);
  goal.position.x = 47.397851;
  goal.position.y = 8.545342;
  goal.position.z = 3;
  goal.heading    = 1.5;
  points.push_back(goal);
  goal.position.x = 47.397711;
  goal.position.y = 8.545375;
  goal.position.z = 3;
  goal.heading    = 1.5;
  points.push_back(goal);


  ActionServerGoal action_client_goal;
  action_client_goal.frame_id = ActionServerGoal::FRAME_ID_LATLON;
  action_client_goal.height_id = ActionServerGoal::HEIGHT_ID_AGL;
  action_client_goal.points = points;
  action_client_goal.terminal_action = ActionServerGoal::TERMINAL_ACTION_LAND;
  ac.sendGoal(action_client_goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[MissionHandlerTestClient]: Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("[MissionHandlerTestClient]: Action did not finish before the time out.");
    ac.cancelAllGoals();
  }

  return EXIT_SUCCESS;
}
