#include <backhoe_controller/backhoe_controller.h>
#include <bucket_controller/bucket_controller.h>
#include <outrigger_controller/outrigger_controller.h>
#include <ros/ros.h>
#include <vesc_access/vesc_access.h>
//TODO backhoe params
#include <visualization_msgs/Marker.h>
#include <sim_robot/sim_outriggers.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "the_backhoe_master");

  ros::NodeHandle node  ("~");
  bool simulating;
  if (node.hasParam("simulating_digging"))
  {
    node.getParam("simulating_digging", simulating);
    if (simulating) {ROS_WARN ("\nRUNNING DIGGING AS SIMULATION\n");}
    else {ROS_WARN ("\nRUNNING DIGGING AS PHYSICAL\n");}
  }
  else
  {
    ROS_ERROR ("\n\nsimulating_digging param not defined! aborting.\n\n");
    return -1;
  }

  iVescAccess *outriggerRightVesc;
  iVescAccess *outriggerLeftVesc;

  //these should not be initialized if we are not simulating
  SimOutriggers * outriggerSimulation;
  if (simulating) 
  {
    outriggerSimulation = new SimOutriggers(0,0);
    outriggerRightVesc = outriggerSimulation->getRVesc();
    outriggerLeftVesc = outriggerSimulation->getLVesc();
        //SimBucket
        //SimBackhoe
  } 
  else 
  {
    outriggerSimulation = NULL; //dont call anything related to this.

    //initialize real vescs here

  }

  //pass vescs (sim or physical) to controllers
  OutriggerController outriggerC(outriggerLeftVesc, outriggerRightVesc);

  ros::Rate rate(50.0);


  while (ros::ok())
  {
    if (simulating) //update simulations if neccesary
    {
      outriggerSimulation->update(.02);
    }
    //update controlelrs
    outriggerC.update(.02);

    //controller logic
    if (outriggerC.isRetracted())
    {
      ROS_INFO("DEPLOYING");
      outriggerC.deploy();
    }
    if (outriggerC.isDeployed())
    {
      ROS_INFO("RETRACTING");
      outriggerC.retract();
    }


    //display output if simulating
    if (simulating) 
    {
      ROS_INFO("SIM OUTRIGGERS AT: %f", outriggerSimulation->getPosL());
      ROS_INFO("SIM OUTRIGGERS AT: %f", outriggerSimulation->getPosR());

      ROS_INFO("LIMIT AT %d", (int) outriggerRightVesc->getLimitSwitchState());
      ROS_INFO("LIMIT AT %d", (int) outriggerLeftVesc->getLimitSwitchState());
    }
    else
    {
      //display output for physical
    }
    //TODO publish markers to a topic


    ros::spinOnce();
    rate.sleep();

  }


}
