#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_safety/teleop_safety.h"
#include "backhoe_controller/backhoe_controller.h"
#include "bucket_controller/bucket_controller.h"
#include "wheel_params/wheel_params.h"

BackhoeController *backhoe;
BucketController *bucket;

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

  VescAccess linear_vesc (linear_param, true);
  VescAccess shoulder_vesc (shoulder_param, true);
  VescAccess left_outrigger_vesc (left_outrigger_param);
  VescAccess right_outrigger_vesc (right_outrigger_param);
  VescAccess sifter_vesc (sifter_param);
  VescAccess small_conveyor_vesc (small_conveyor_param);
  VescAccess large_conveyor_vesc (large_conveyor_param);

  BackhoeController backhoe ();
  BucketController bucket ();

  ros::Subscriber joy_sub = n.subscribe("joy", 10, callback);
  ros::Rate r(50);
  ros::Time initial = ros::Time::now ();
  while (ros::ok ())
  {
    r.sleep ();

  }

}