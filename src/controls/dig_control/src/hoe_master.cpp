#include <backhoe_controller/backhoe_controller.h>
#include <bucket_controller/bucket_controller.h>
#include <outrigger_controller/outrigger_controller.h>


//#define SIMULATING_DIGGING 1

#include <ros/ros.h>

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>

//TODO backhoe params

#include <visualization_msgs/Marker.h>

#ifndef SIMULATING_DIGGING
#error You must define a value (1/0) for SIMULATING_DIGGING
#endif

#if SIMULATING_DIGGING == 1
#include <sim_robot/sim_outriggers.h>

#else
#error Who do you think you are?
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "the_backhoe_master");

  ros::NodeHandle node;

#if SIMULATING_DIGGING == 1
  SimOutriggers outriggers( 0, 0);
  iVescAccess *outriggerRightVesc = outriggers.getRVesc();
  iVescAccess *outriggerLeftVesc = outriggers.getLVesc();

#else
//INITIALIZE REAL VESC OBJECTS

#endif

  OutriggerController outriggerC(outriggerLeftVesc, outriggerRightVesc);

  ros::Rate rate(50.0);


  while (ros::ok())
  {
#if SIMULATING_DIGGING == 1
    outriggers.update(.02);
    //update marker too

#endif

    outriggerC.update(.02);
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

#if SIMULATING_DIGGING == 1
    ROS_INFO("OUTRIGGERS AT: %f", outriggers.getPosL());
    ROS_INFO("OUTRIGGERS AT: %f", outriggers.getPosR());

    ROS_INFO("LIMIT AT %d", (int)outriggerRightVesc->getLimitSwitchState());
    ROS_INFO("LIMIT AT %d", (int)outriggerLeftVesc->getLimitSwitchState());
#endif

    ros::spinOnce();
    rate.sleep();

  }


}
