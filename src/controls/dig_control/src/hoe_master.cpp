#include <backhoe_controller/backhoe_controller.h>
#include <bucket_controller/bucket_controller.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <vesc_access/vesc_access.h>
// TODO backhoe params
#include <visualization_msgs/Marker.h>
#include <sim_robot/sim_outriggers.h>
#include <sim_robot/sim_bucket.h>
#include <sim_robot/sim_backhoe.h>

#include <sensor_msgs/JointState.h>
#include <cmath>

#include "dig_dump_action/dig_dump_action.h"
#include "wheel_params/wheel_params.h"

#include "safety_vesc/backhoe_safety_controller.h"
#include "safety_vesc/linear_safety_controller.h"

#define DIGGING_CONTROL_RATE_HZ 50.0


bool should_initialize = false;

void callback (const std_msgs::Empty::ConstPtr &msg)
{
  should_initialize = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "the_backhoe_master");

  ros::NodeHandle node("~");
  ros::NodeHandle globalNode;
  ros::Rate rate(DIGGING_CONTROL_RATE_HZ);  // should be 50 Hz

  ros::Subscriber initializeSub = globalNode.subscribe("init_digging",100,callback);

  bool simulating;
  if (node.hasParam("simulating_digging"))
  {
    node.getParam("simulating_digging", simulating);
    if (simulating)
    {
      ROS_WARN("\nRUNNING DIGGING AS SIMULATION\n");
    }
    else
    {
      ROS_WARN("\nRUNNING DIGGING AS PHYSICAL\n");
    }
  }
  else
  {
    ROS_ERROR("\n\nsimulating_digging param not defined! aborting.\n\n");
    return -1;
  }

  double backhoeInitialShoulderTheta;
  double backhoeInitialWristTheta;

  iVescAccess *bucketBigConveyorVesc;
  iVescAccess *bucketLittleConveyorVesc;
  iVescAccess *bucketSifterVesc;
  iVescAccess *backhoeShoulderVesc;
  iVescAccess *backhoeWristVesc;
  ros::Time lastTime=ros::Time::now();
  ros::Rate vesc_init_rate (10);
  // these should not be initialized if we are not simulating
  SimBucket *bucketSimulation;

  SimBackhoe *backhoeSimulation;
  if (simulating)
  {
    // SimBucket
    bucketSimulation = new SimBucket();
    bucketBigConveyorVesc = bucketSimulation->getBigConveyorVesc();
    bucketLittleConveyorVesc = bucketSimulation->getLittleConveyorVesc();
    bucketSifterVesc = bucketSimulation->getSifterVesc();
    // SimBackhoe
    backhoeSimulation = new SimBackhoe(2.0, .1, central_joint_params.lower_limit_position, central_joint_params.upper_limit_position,
                                       linear_joint_params.lower_limit_position, linear_joint_params.upper_limit_position);  // shoulder and wrist angle, limits
    backhoeShoulderVesc = backhoeSimulation->getShoulderVesc();
    backhoeWristVesc = backhoeSimulation->getWristVesc();
    // populate inital backhoe position
    backhoeInitialShoulderTheta = 0;
    backhoeInitialWristTheta = 0;
  }
  else
  {
    // Don't use these pointers
    // This is a physical run.
    // You'll cause exceptions.
    bucketSimulation = NULL;
    backhoeSimulation = NULL;
    bool no_except = false;
    while (!no_except  && ros::ok()) {
      try {
        backhoeShoulderVesc = new VescAccess(shoulder_param, true);
        backhoeWristVesc = new VescAccess(linear_param, true);
        bucketLittleConveyorVesc = new VescAccess(small_conveyor_param);
        bucketBigConveyorVesc = new VescAccess(large_conveyor_param);
        bucketSifterVesc = new VescAccess(sifter_param);
        no_except = true;
      } catch (VescException e) {
        ROS_WARN("%s",e.what ());
        no_except = false;
      }
      if (!no_except && (ros::Time::now() - lastTime).toSec() > 10){
        ROS_ERROR ("Vesc exception thrown for more than 10 seconds");
        ros::shutdown ();
      }
        vesc_init_rate.sleep();
    }

    // initialize real vescs here
  }
 LinearSafetyController linearSafety(linear_joint_params, backhoeWristVesc);
  BackhoeSafetyController backhoeSafety(central_joint_params, backhoeShoulderVesc);
  ROS_INFO ("WAITING for init");
  while (ros::ok() && !should_initialize)
  {
    ros::spinOnce();
    rate.sleep();
  }
    
  backhoeSafety.init();

    ROS_INFO ("INITING");

  bool isLinearInit = false;
  while (ros::ok() && !isLinearInit)
  {
    if (simulating)
    {
      backhoeSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
      ROS_DEBUG("Linear at %.4f", backhoeSimulation->getWrTheta());
      ROS_DEBUG("Linear from vesc at %.4f", backhoeSimulation->getWristVesc()->getPotPosition());
      ROS_DEBUG("Linear Limit state %d", backhoeSimulation->getWristVesc()->getLimitSwitchState());
      ROS_DEBUG("Linear vel set to  %.4f", backhoeSimulation->getWristVesc()->getLinearVelocity());
      if (((SimVesc *)backhoeSimulation->getWristVesc())->ableToHitGround())
        ROS_INFO("Linear able to hit ground");
    }
    isLinearInit = linearSafety.init();
    rate.sleep();
    ros::spinOnce();
  }

  backhoeSafety.setPositionSetpoint(CENTRAL_TRANSPORT_ANGLE);
  while (ros::ok () && !backhoeSafety.isAtSetpoint())
  {
    if (simulating)
    {
        bucketSimulation->update(1.0/DIGGING_CONTROL_RATE_HZ);
        backhoeSimulation->update(1.0/DIGGING_CONTROL_RATE_HZ);
    }
    backhoeSafety.update(1.0/DIGGING_CONTROL_RATE_HZ);
    rate.sleep();
    ros::spinOnce();
  }
  BucketController bucketC(bucketBigConveyorVesc, bucketLittleConveyorVesc, bucketSifterVesc);
  BackhoeController backhoeC(&backhoeSafety, &linearSafety);

  bool bucket_init = false;
  while(ros::ok() && bucket_init)
  {
    bucket_init = bucketC.init();
    rate.sleep();
    ros::spinOnce();
  }


  // pass vescs (sim or physical) to controllers

  ROS_INFO("Init'd");

  DigDumpAction ddAct(&backhoeC, &bucketC, argc, argv);

  ros::Publisher JsPub = globalNode.advertise<sensor_msgs::JointState>("joint_states", 100);

  while (ros::ok())
  {
    if (simulating)  // update simulations if neccesary
    {
      bucketSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
      backhoeSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
    }
    // update controllers

    backhoeC.update(1.0 / DIGGING_CONTROL_RATE_HZ);

    // display output if simulating
    sensor_msgs::JointState robotAngles;
    if (simulating)
    {
      robotAngles.header.stamp = ros::Time::now();

      robotAngles.name.push_back("central_drive_to_monoboom");

      robotAngles.position.push_back(backhoeSimulation->getShTheta());

      robotAngles.name.push_back("monoboom_actuator");
      robotAngles.position.push_back(backhoeSimulation->getWrTheta());

      JsPub.publish(robotAngles);
      ROS_DEBUG("shoulder joint state published with angle %f \n", backhoeSimulation->getShTheta());
      ROS_DEBUG("wrist joint state published with angle %f \n", backhoeSimulation->getWrTheta());
    }
    else  // display output for physical
    {
    }
    //ROS_INFO("backhoe controller says CD at %.4f", backhoeSafety.getPositionEstimate());
    //ROS_INFO("backhoe controller says LA at %.4f", linearSafety.getPositionEstimate());
    //ROS_INFO("Digdump AS states: %d, %d", ddAct.digging_state, ddAct.dumping_state);

    ros::spinOnce();
    rate.sleep();
  }
}
