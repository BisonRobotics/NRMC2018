#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <central_drive_control/central_driveAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "central_drive_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<central_drive_control::central_driveAction> ac("central_drive", true);

  ROS_INFO("Waiting for action server to startz.");
  // wait for the action server to start
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goalz.");
  // send a goal to the action
  central_drive_control::central_driveGoal goal;

  // this could be the setpoint for the backhoe?
  goal.order = 30;  // this should be a number from 0 to 6000
  ac.sendGoal(goal);

  // wait for the action to return
  // timeout to see if action was performed.
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finishedz: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time outz.");

  // exit
  return 0;
}
