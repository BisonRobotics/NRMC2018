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
    if(!node.param("simulating_digging", simulating, true)){
        ROS_WARN ("simulating_digging was not found, assumed simulating");
    }

    iVescAccess *outriggerRightVesc;
    iVescAccess *outriggerLeftVesc;

    SimOutriggers outriggers(0, 0);
    if (simulating) {
        outriggerRightVesc = outriggers.getRVesc();
        outriggerLeftVesc = outriggers.getLVesc();
        //SimBucket
        //SimBackhoe
    } else {
        ROS_WARN ("No good option for failure");
        return -1;
    }

  OutriggerController outriggerC(outriggerLeftVesc, outriggerRightVesc);

  ros::Rate rate(50.0);


  while (ros::ok())
  {
    if (simulating) {
        outriggers.update(.02);
        //update marker too
    }

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

    if (simulating) {
        ROS_INFO("OUTRIGGERS AT: %f", outriggers.getPosL());
        ROS_INFO("OUTRIGGERS AT: %f", outriggers.getPosR());

        ROS_INFO("LIMIT AT %d", (int) outriggerRightVesc->getLimitSwitchState());
        ROS_INFO("LIMIT AT %d", (int) outriggerLeftVesc->getLimitSwitchState());
    }
    //TODO publish markers to a topic


    ros::spinOnce();
    rate.sleep();

  }


}
