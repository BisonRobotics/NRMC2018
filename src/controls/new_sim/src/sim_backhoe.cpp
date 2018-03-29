#include "sim_robot/sim_backhoe.h"

SimBackhoe::SimBackhoe(double shoulderTheta, double wristTheta)
{
  shTh = shoulderTheta;
  wrTh = wristTheta;

  sh = new SimVesc(16, 0, 1.0, shoulderTheta, 0, 3.1);
  wr = new SimVesc(16, 0, 1.0, wristTheta, 0, 3.1);
}

void SimBackhoe::update(double dt)
{
  sh->update(dt);
  wr->update(dt);

  shTh += sh->getLinearVelocity() * dt;
  wrTh += wr->getLinearVelocity() * dt;
}

double SimBackhoe::getShTheta()
{
  return shTh;
}

double SimBackhoe::getWrTheta()
{
  return wrTh;
}

iVescAccess *SimBackhoe::getShoulderVesc()
{
  return sh;
}

iVescAccess *SimBackhoe::getWristVesc()
{
  return wr;
}