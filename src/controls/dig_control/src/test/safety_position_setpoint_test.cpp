#include <ros/ros.h>
#include <backhoe_controller/backhoe_controller.h>
#include <vesc_access/vesc_access.h>
#include "wheel_params/wheel_params.h"

#include "safety_vesc/backhoe_safety_controller.h"
#include "safety_vesc/linear_safety_controller.h"

#define GROUND_ALPHA .2

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_setpoint_tester");

  ros::NodeHandle globalNode;
  ros::Rate rate(50);

  iVescAccess *backhoeShoulderVesc;
  iVescAccess *backhoeWristVesc;

  backhoeShoulderVesc = new VescAccess(shoulder_param, true);
  backhoeWristVesc = new VescAccess(linear_param, true);

  BackhoeSafetyController backhoeSafety(central_joint_params, backhoeShoulderVesc);
  backhoeSafety.init();



  LinearSafetyController linearSafety(linear_joint_params, backhoeWristVesc);
  bool isLinearInit = false;
  while (ros::ok() && !isLinearInit)
  {
    isLinearInit = linearSafety.init();
    rate.sleep();
  }

  BackhoeController backhoeC(&backhoeSafety, &linearSafety);

  backhoeC.update(.02);
  bool setOnce = false;
  double ground_metric =10;
  double prev_backhoe_position = backhoeSafety.getPositionEstimate();

  while (ros::ok())
  {
    backhoeC.update(.02);
    ROS_INFO("backhoe controller says CD at %.4f", backhoeSafety.getPositionEstimate());
    ROS_INFO("backhoe controller says LA at %.4f", linearSafety.getPositionEstimate());
	ROS_INFO("backhoe torque says : %.4f", backhoeShoulderVesc->getTorque());

    ground_metric = GROUND_ALPHA * (prev_backhoe_position - backhoeSafety.getPositionEstimate())/.02 
                    + (1 - GROUND_ALPHA) * ground_metric;
    prev_backhoe_position = backhoeSafety.getPositionEstimate();
    ROS_INFO("ground metric: %.4f", ground_metric);
    if (ground_metric < .05)
    {
      ROS_WARN("HIT GROUND");
    }

    if (!setOnce)
    {
       backhoeC.setShoulderSetpoint(.5);
       setOnce = true;
    }
    else
    {
       if (backhoeC.shoulderAtSetpoint())
       {
          ROS_INFO("backhoe at setpoint!");
       }
    }
    

    ros::spinOnce();
    rate.sleep();

    
  }
}