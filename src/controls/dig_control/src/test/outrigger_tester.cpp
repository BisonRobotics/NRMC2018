#include <dig_control/OutriggerAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "outrigger_tester");
  actionlib::SimpleActionClient<dig_control::outriggerAction> client("deploy_riggers", true); // true -> don't need ros::spin()
  client.waitForServer();
  dig_control::outriggersGoal goal;
  // Fill in goal here

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}