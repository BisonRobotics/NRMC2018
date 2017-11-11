#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_interface/teleop_interface.h>

float velocity_left = 0.0f;
float velocity_right = 0.0f;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {
    velocity_left = joy->axes[1];
    velocity_right = joy->axes[4];
  }
  else
  {
    velocity_left = 0.0f;
    velocity_right = 0.0f;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotteleop");
  ros::NodeHandle n;
  ROS_INFO("GOT HERE");
  ros::Subscriber joy_sub = n.subscribe("joy", 10, callback);
  ros::Rate loop_rate(100);
  ROS_INFO("DECLARED LOOP RATE");
  TeleopInterface tele(0.8f);

  ROS_INFO("Declared teleop Interface");
  while (ros::ok())
  {
    tele.update(velocity_left, velocity_right);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
