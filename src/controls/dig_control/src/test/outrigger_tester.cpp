#include <dig_control/OutriggerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "outrigger_tester");

  actionlib::SimpleActionClient<dig_control::OutriggerAction> deployClient("deploy_riggers",
                                                                           true);  // true -> don't need ros::spin()
  deployClient.waitForServer();

  actionlib::SimpleActionClient<dig_control::OutriggerAction> retractClient("retract_riggers",
                                                                            true);  // true -> don't need ros::spin()
  retractClient.waitForServer();

  dig_control::OutriggerGoal goal;
  // Fill in goal here

  ROS_INFO("SENDING GOAL");
  deployClient.sendGoal(goal);
  deployClient.waitForResult(ros::Duration(5.0));
  ROS_INFO("DONE WAITING");
  if (deployClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The outriggers are now deployed");
  printf("Current State: %s\n", deployClient.getState().toString().c_str());

  ROS_INFO("SENDING GOAL");
  retractClient.sendGoal(goal);
  retractClient.waitForResult(ros::Duration(5.0));
  ROS_INFO("DONE WAITING");
  if (retractClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The outriggers are now retracted");
  printf("Current State: %s\n", retractClient.getState().toString().c_str());

  ROS_INFO("SENDING GOAL");
  deployClient.sendGoal(goal);
  deployClient.waitForResult(ros::Duration(5.0));
  ROS_INFO("DONE WAITING");
  if (deployClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The outriggers are now deployed");
  printf("Current State: %s\n", deployClient.getState().toString().c_str());

  ROS_INFO("SENDING GOAL");
  retractClient.sendGoal(goal);
  retractClient.waitForResult(ros::Duration(5.0));
  ROS_INFO("DONE WAITING");
  if (retractClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The outriggers are now retracted");
  printf("Current State: %s\n", retractClient.getState().toString().c_str());

  return 0;
}