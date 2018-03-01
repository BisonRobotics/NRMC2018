#include<dig_control/OutriggerAction.h>
#include<actionlib/server/simple_action_server.h>
#include <outrigger_controller/outrigger_controller.h>
#include <ros/ros.h>
#include <vesc_access/vesc_access.h>
#include <sim_robot/sim_outriggers.h>

SimOutriggers *outriggerSimulation;
iVescAccess *outriggerRightVesc;
iVescAccess *outriggerLeftVesc;
OutriggerController *outriggerC;
bool simulating;

void deployOutriggers(const dig_control::OutriggerGoalConstPtr & goal, 
             actionlib::SimpleActionServer<dig_control::OutriggerAction> * serv_ptr)
{
  //deploy outriggers
  outriggerC->deploy();
  while (outriggerC->isDeployed() == false)
  {
    //make loop speed constant, can that be done with ros rate and sleep?
    outriggerC->update(.02);
    if (simulating)
    {
      outriggerSimulation->update(.02);
    }
  }
  serv_ptr->setSucceeded();
}

void retractOutriggers(const dig_control::OutriggerGoalConstPtr & goal, 
             actionlib::SimpleActionServer<dig_control::OutriggerAction> * serv_ptr)
{
  //deploy outriggers
  outriggerC->retract();
  while (outriggerC->isRetracted() == false)
  {
    //make loop speed constant, can that be done with ros rate and sleep?
    outriggerC->update(.02);
    if (simulating)
    {
      outriggerSimulation->update(.02);
    }
  }
  serv_ptr->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "outriggerAction.h");
  ros::NodeHandle n;

  simulating = true; //TODO, read this from rosparam

  actionlib::SimpleActionServer<dig_control::OutriggerAction> deployServer(n, "deploy_riggers", 
                                       boost::bind(&deployOutriggers, _1, &deployServer), false);
  actionlib::SimpleActionServer<dig_control::OutriggerAction> retractServer(n, "retract_riggers", 
                                       boost::bind(&retractOutriggers, _1, &retractServer), false);

  if (simulating)
  {
    outriggerSimulation = new SimOutriggers(0,0);
    outriggerRightVesc = outriggerSimulation->getRVesc();
    outriggerLeftVesc = outriggerSimulation->getLVesc();
  }
  else
  {
    outriggerSimulation = NULL;
    //populate real VESCS here
  }
  outriggerC = new OutriggerController(outriggerLeftVesc, outriggerRightVesc);

  deployServer.start();
  retractServer.start();
  ros::spin();
  return 0;
}