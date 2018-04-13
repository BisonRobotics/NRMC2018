#include <backhoe_controller/backhoe_controller.h>
#include <bucket_controller/bucket_controller.h>

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "the_backhoe_master");

  ros::NodeHandle node("~");
  ros::NodeHandle globalNode;
  ros::Rate rate(DIGGING_CONTROL_RATE_HZ); //should be 50 Hz

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
    backhoeSimulation = new SimBackhoe(0, 0, -2, 4, -2, 4);  // shoulder and wrist angle, limits
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

    backhoeShoulderVesc = new VescAccess (shoulder_param, true);
    backhoeWristVesc = new VescAccess (linear_param, true);
    bucketLittleConveyorVesc = new VescAccess (small_conveyor_param);
    bucketBigConveyorVesc = new VescAccess (large_conveyor_param);
    bucketSifterVesc = new VescAccess (sifter_param);
    // initialize real vescs here
 }

  LinearSafetyController linearSafety (linear_joint_params, backhoeWristVesc);
  bool isLinearInit = false;
  while (ros::ok() && !isLinearInit)
  {
    if (simulating)
    {
      backhoeSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
      ROS_INFO("Linear at %.4f", backhoeSimulation->getWrTheta());
      ROS_INFO("Linear from vesc at %.4f", backhoeSimulation->getWristVesc()->getPotPosition());
    }
    isLinearInit = linearSafety.init();
    rate.sleep();
  }


  BackhoeSafetyController backhoeSafety (central_joint_params, backhoeShoulderVesc);
  backhoeSafety.init();
  // pass vescs (sim or physical) to controllers

  ROS_INFO("Init'd");
  BucketController bucketC(bucketBigConveyorVesc, bucketLittleConveyorVesc, bucketSifterVesc);
  BackhoeController backhoeC(&backhoeSafety, &linearSafety);

  DigDumpAction ddAct(&backhoeC, &bucketC);

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

       robotAngles.name.push_back("monoboom_to_backhoe_bucket");
       robotAngles.position.push_back(backhoeSimulation->getWrTheta());

       JsPub.publish(robotAngles);
       ROS_INFO("shoulder joint state published with angle %f \n", backhoeSimulation->getShTheta());
       ROS_INFO("wrist joint state published with angle %f \n", backhoeSimulation->getWrTheta());
    }
    else // display output for physical
    {
      
    }

    ROS_INFO("Digdump AS states: %d, %d", ddAct.digging_state, ddAct.dumping_state);

    ros::spinOnce();
    rate.sleep();
  }
}
