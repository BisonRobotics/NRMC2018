#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_safety/teleop_safety.h"

TeleopSafety teleopSafety;



void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  float linear=0.0f;
  float central=0.0f;
  float outriggers=0.0f;
  bool sifter_toggle = false;
  bool large_conveyor_toggle = false;
  if (joy->buttons[4])              // left bumper is safety
  {
    linear = joy->axes[1]; // left analog up/down
    central = joy->axes[3]; // right analog up/down
    sifter_toggle = joy->buttons[0] != 0; // A button
    large_conveyor_toggle = joy->buttons[1] != 0; // B button

  }
  else if (joy->buttons[5])
  {
    outriggers = joy->axes[1]; // if only the right bumper is down, the left analog up/down is outriggers
  }
  teleopSafety.moveLinear(linear);
  teleopSafety.moveShoulder(central);
  if (sifter_toggle)
  {
    teleopSafety.toggleSifter();
  }
  if (large_conveyor_toggle)
  {
    teleopSafety.toggleLargeConveyor();
  }
  teleopSafety.moveOutriggers(outriggers);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 10, callback);
  ros::spin ();
}