#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "backhoe_controller/backhoe_controller.h"
#include "bucket_controller/bucket_controller.h"
#include "wheel_params/wheel_params.h"
#include "outrigger_controller/outrigger_controller.h"
#include "safety_vesc/backhoe_safety_controller.h"
#include "safety_vesc/linear_safety_controller.h"

BackhoeController *global_backhoe;
BucketController *global_bucket;
OutriggerController *global_outrigger;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  float linear = 0.0f;
  float central = 0.0f;
  bool sifter_toggle = false;
  bool large_conveyor_toggle = false;
  static constexpr float linear_gain = .1;
  static constexpr float central_gain = 1.0f;

  if (joy->buttons[4])  // left bumper is safety
  {
    linear = joy->axes[1];                         // left analog up/down
    central = joy->axes[3];                        // right analog up/down
    sifter_toggle = joy->buttons[0] != 0;          // A button
    large_conveyor_toggle = joy->buttons[1] != 0;  // B button

    if (joy->buttons[2] && !joy->buttons[3])      /* should we toggle */
    {
      global_outrigger->deploy();
    }
    else if (joy->buttons[3] && !joy->buttons[2])
    {
      global_outrigger->retract();
    }
  }

  global_backhoe->setShoulderTorque(central_gain * central);
  global_backhoe->setWristVelocity(linear_gain * linear);

  if (sifter_toggle)
  {
    global_bucket->toggleSifter();
    global_bucket->toggleLittleConveyor();
  }
  if (large_conveyor_toggle)
  {
    global_bucket->toggleBigConveyor();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;

  VescAccess linear_vesc(linear_param, true);
  VescAccess shoulder_vesc(shoulder_param, true);
  VescAccess left_outrigger_vesc(left_outrigger_param);
  VescAccess right_outrigger_vesc(right_outrigger_param);
  VescAccess sifter_vesc(sifter_param);
  VescAccess small_conveyor_vesc(small_conveyor_param);
  VescAccess large_conveyor_vesc(large_conveyor_param);

  bool has_been_init = false;

  LinearSafetyController linearSafety(linear_joint_params, &linear_vesc);
  BackhoeSafetyController backhoeSafety(central_joint_params, &shoulder_vesc);
  BackhoeController backhoe(&backhoeSafety, &linearSafety);

  BucketController bucket(&large_conveyor_vesc, &small_conveyor_vesc, &sifter_vesc);

  OutriggerController outrigger(&left_outrigger_vesc, &right_outrigger_vesc);
  global_backhoe = &backhoe;
  global_bucket = &bucket;
  global_outrigger = &outrigger;

  ros::Subscriber joy_sub = n.subscribe("joy_dig", 10, callback);
  ros::Rate r(10);
  ros::Time initial = ros::Time::now();

  backhoeSafety.init();
  while (ros::ok() && !has_been_init)
  {
    has_been_init = linearSafety.init ();
    r.sleep();
  }

  ROS_INFO ("Init!!");
  float velocity;
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    backhoe.update((initial - ros::Time::now()).toSec());
    outrigger.update((initial - ros::Time::now()).toSec());
    initial = ros::Time::now();
    velocity = small_conveyor_vesc.getTorque();
    velocity = large_conveyor_vesc.getTorque();
    velocity = sifter_vesc.getTorque ();
    velocity = right_outrigger_vesc.getTorque();
    velocity = left_outrigger_vesc.getTorque();
    velocity = shoulder_vesc.getTorque();
    velocity = linear_vesc.getTorque ();
  }
}