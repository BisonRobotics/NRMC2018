#include <ros/ros.h>
#include <wheel_control/distanceAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<wheel_control::distanceAction> Server;

void execute(const wheel_control::distanceGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_controller");
  ros::NodeHandle n;
  Server server(n, "drive a distance", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
