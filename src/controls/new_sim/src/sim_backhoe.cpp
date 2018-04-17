#include "sim_robot/sim_backhoe.h"
#include <ros/ros.h>

SimBackhoe::SimBackhoe(double shoulderTheta, double wristTheta)
{
  shTh = shoulderTheta;
  wrTh = wristTheta;

  sh = new SimVesc(16, 0, 1.0);
  wr = new SimVesc(16, 0, 1.0);
}

SimBackhoe::SimBackhoe(double shoulderTheta, double wristTheta, double shoulderBottomLimit,
                       double shoulderUpperLimit, double wristBottomLimit, double wristUpperLimit)
{
  shTh = shoulderTheta;
  wrTh = wristTheta;

  sh = new SimVesc(8, 0, 1.0, shoulderTheta, shoulderBottomLimit, shoulderUpperLimit, true, 1.3);
  wr = new SimVesc(8, 0, 1.0, wristTheta, wristBottomLimit, wristUpperLimit, false, 0);
}

void SimBackhoe::update(double dt)
{
  ROS_INFO("shoulder sim update");
  sh->update(dt);
  ROS_INFO("wrist sim update");
  wr->update(dt);

  //shTh += sh->getLinearVelocity() * dt;
  //wrTh += wr->getLinearVelocity() * dt;
}

double SimBackhoe::getShTheta()
{
  return sh->getPotPosition();
}

double SimBackhoe::getWrTheta()
{
  return wr->getPotPosition();
}

iVescAccess *SimBackhoe::getShoulderVesc()
{
  return sh;
}

iVescAccess *SimBackhoe::getWristVesc()
{
  return wr;
}