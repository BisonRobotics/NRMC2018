#include <backhoe_controller/backhoe_controller.h>
#include <bucket_controller/bucket_controller.h>

#include <ros/ros.h>
#include <vesc_access/vesc_access.h>
// TODO backhoe params
#include <visualization_msgs/Marker.h>
#include <sim_robot/sim_outriggers.h>
#include <sim_robot/sim_bucket.h>
#include <sim_robot/sim_backhoe.h>

#include "dig_dump_action/dig_dump_action.h"
#include "wheel_params/wheel_params.h"

#include "safety_vesc/backhoe_safety_controller.h"
#include "safety_vesc/linear_safety_controller.h"

#define DIGGING_CONTROL_RATE_HZ 50.0

int main(int argc, char **argv)
{
  ros::init(argc, argv, "the_backhoe_master");

  ros::NodeHandle node("~");
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

  // iVescAccess *outriggerRightVesc;
  // iVescAccess *outriggerLeftVesc;
  iVescAccess *bucketBigConveyorVesc;
  iVescAccess *bucketLittleConveyorVesc;
  iVescAccess *bucketSifterVesc;
  iVescAccess *backhoeShoulderVesc;
  iVescAccess *backhoeWristVesc;

  // these should not be initialized if we are not simulating
  // SimOutriggers *outriggerSimulation;
  SimBucket *bucketSimulation;
  SimBackhoe *backhoeSimulation;
  if (simulating)
  {
    // SimOutrigger
    //  outriggerSimulation = new SimOutriggers(0, 0);  // initial deployment length
    //  outriggerRightVesc = outriggerSimulation->getRVesc();
    //  outriggerLeftVesc = outriggerSimulation->getLVesc();
    // SimBucket
    bucketSimulation = new SimBucket();
    bucketBigConveyorVesc = bucketSimulation->getBigConveyorVesc();
    bucketLittleConveyorVesc = bucketSimulation->getLittleConveyorVesc();
    bucketSifterVesc = bucketSimulation->getSifterVesc();
    // SimBackhoe
    backhoeSimulation = new SimBackhoe(0, 0);  // shoulder and wrist angle
    backhoeShoulderVesc = backhoeSimulation->getShoulderVesc();
    backhoeWristVesc = backhoeSimulation->getWristVesc();
    // populate inital backhoe position
    backhoeInitialShoulderTheta = 0;
    backhoeInitialWristTheta = 0;
  }
  else
  {
    //  outriggerSimulation = NULL;  // Don't use these pointers.
    bucketSimulation = NULL;   // This is a physical run.
    backhoeSimulation = NULL;  // You'll cause exceptions.

    // initialize real vescs here

    // populate inital backhoe position
  }

  LinearSafetyController linearSafety (linear_joint_params, backhoeWristVesc, false);
  BackhoeSafetyController backhoeSafety (central_joint_params, backhoeShoulderVesc, false);
  // pass vescs (sim or physical) to controllers

  BucketController bucketC(bucketBigConveyorVesc, bucketLittleConveyorVesc, bucketSifterVesc);
  BackhoeController backhoeC(&backhoeSafety, &linearSafety);

  ros::Rate rate(DIGGING_CONTROL_RATE_HZ);

  DigDumpAction ddAct(&backhoeC, &bucketC);

  while (ros::ok())
  {
    if (simulating)  // update simulations if neccesary
    {
      // outriggerSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
      bucketSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
      backhoeSimulation->update(1.0 / DIGGING_CONTROL_RATE_HZ);
    }
    // update controlelrs

    // bucketC.update(1.0 / DIGGING_CONTROL_RATE_HZ); //don't need to update the bucket
    backhoeC.update(1.0 / DIGGING_CONTROL_RATE_HZ);

    // display output if simulating
    if (simulating)
    {
      // ROS_INFO("SIM OUTRIGGERS AT: %f", outriggerSimulation->getPosL());
      // ROS_INFO("SIM OUTRIGGERS AT: %f", outriggerSimulation->getPosR());

      // ROS_INFO("LIMIT AT %d", (int) outriggerRightVesc->getLimitSwitchState());
      // ROS_INFO("LIMIT AT %d", (int) outriggerLeftVesc->getLimitSwitchState());
    }
    else
    {
      // display output for physical
    }
    // TODO publish markers to a topic

    ros::spinOnce();
    rate.sleep();
  }
}
