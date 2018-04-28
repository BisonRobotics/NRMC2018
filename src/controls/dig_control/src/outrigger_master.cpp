#include <dig_control/OutriggerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <outrigger_controller/outrigger_controller.h>
#include <sim_robot/sim_outriggers.h>
#include "wheel_params/wheel_params.h"

SimOutriggers *outriggerSimulation;
iVescAccess *outriggerRightVesc;
iVescAccess *outriggerLeftVesc;
OutriggerController *outriggerC;
bool simulating;

void deployOutriggers(const dig_control::OutriggerGoalConstPtr &goal,
                      actionlib::SimpleActionServer<dig_control::OutriggerAction> *serv_ptr)
{
  // deploy outriggers
  outriggerC->deploy();
  ros::Rate r(50);
  while (!outriggerC->isDeployed() && ros::ok())
  {
    r.sleep();
    outriggerC->update(.02);
    if (simulating)
    {
      outriggerSimulation->update(.02);
    }
  }
  serv_ptr->setSucceeded();
}

void retractOutriggers(const dig_control::OutriggerGoalConstPtr &goal,
                       actionlib::SimpleActionServer<dig_control::OutriggerAction> *serv_ptr)
{
  // deploy outriggers
  ros::Rate r(50);
  outriggerC->retract();
  while (!outriggerC->isRetracted() && ros::ok())  // is this structure OK? is the ros::ok() check necessary?
  {
    r.sleep();
    outriggerC->update(.02);
    if (simulating)
    {
      outriggerSimulation->update(.02);
    }
  }
  serv_ptr->setSucceeded();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outrigger_action_server");
  ros::NodeHandle n("~");
  ros::Rate vesc_init_rate (10);
  if (n.hasParam("simulating_digging"))
  {
    n.getParam("simulating_digging", simulating);
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

  actionlib::SimpleActionServer<dig_control::OutriggerAction> deployServer(
      n, "deploy_riggers", boost::bind(&deployOutriggers, _1, &deployServer), false);
  actionlib::SimpleActionServer<dig_control::OutriggerAction> retractServer(
      n, "retract_riggers", boost::bind(&retractOutriggers, _1, &retractServer), false);
  ros::Time lastTime = ros::Time::now();
  if (simulating)
  {
    outriggerSimulation = new SimOutriggers(0, 0);
    outriggerRightVesc = outriggerSimulation->getRVesc();
    outriggerLeftVesc = outriggerSimulation->getLVesc();
  }
  else
  {
    outriggerSimulation = NULL;
    bool no_except = false;
    while (!no_except  && ros::ok()) {
      try {
        outriggerRightVesc = new VescAccess(right_outrigger_param);
        outriggerLeftVesc = new VescAccess(left_outrigger_param);
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

  }
  outriggerC = new OutriggerController(outriggerLeftVesc, outriggerRightVesc);

  deployServer.start();
  retractServer.start();
  ros::spin();
  return 0;
}
