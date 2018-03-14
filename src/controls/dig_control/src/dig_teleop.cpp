#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "wheel_params/wheel_params.h"
#include "vesc_access/vesc_access.h"

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {

  }
  else
  {

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 10, callback);
  ros::Rate loop_rate(20);
  while (ros::ok ())
  {

  }
}