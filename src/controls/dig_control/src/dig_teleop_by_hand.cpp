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

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool linear_up = false;
  bool linear_down = false;
  bool central_up = false;
  bool central_down = false;
  bool sifter_toggle = false;
  static constexpr float linear_gain = 6.0;
  static constexpr float central_gain = 5.0f;

  if (joy->buttons[5])  // right bumper is safety
  {
    linear_up = joy->buttons[3];                  // Y is linear up
    linear_down = joy->buttons[2];                // X is linear down
    central_up = joy->buttons[1];                 // B is central up
    central_down = joy->buttons[0];               // A is central down
    sifter_toggle = joy->axes[2] == -1.0f;          //LT is sifter stuff
  }

  if (linear_up && linear_down)
  {
    linear_up = false;
    linear_down = false;
  }

  if (central_up && central_down)
  {
    central_up = false;
    central_down = false;
  }

  if (central_up)
  {
    global_backhoe->setShoulderTorque(central_gain);
  }
  else if (central_down)
  {
    global_backhoe->setShoulderTorque(-central_gain);
  }
  else
  {
    ROS_INFO ("shoulder set in callback");
    global_backhoe->stopShoulder();
  }

  if (linear_up)
  {
    global_backhoe->setWristTorque(linear_gain);
  }
  else if (linear_down)
  {
    global_backhoe->setWristTorque(-linear_gain);
  }
  else
  {
    ROS_INFO ("wrist Set in callback");
    global_backhoe->stopWrist();
  }

  if (sifter_toggle) {
    global_bucket->toggleSifter();
    global_bucket->toggleLittleConveyor();
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

  global_backhoe = &backhoe;
  global_bucket = &bucket;

  ros::Subscriber joy_sub = n.subscribe("joy", 10, callback);

  ros::Rate r(40);
  ros::Time initial = ros::Time::now();

  ROS_INFO ("Initializing");
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
      initial=ros::Time::now();
    r.sleep();
    ros::spinOnce();
    backhoe.update(1.0/40.0);
    ROS_INFO ("linear position: %.4f central position: %.4f ",linearSafety.getPositionEstimate(),backhoeSafety.getPositionEstimate() );
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