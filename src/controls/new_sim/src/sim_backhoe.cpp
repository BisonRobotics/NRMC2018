#include "sim_robot/sim_backhoe.h"

SimBackhoe::SimBackhoe(double shouldlerTheta, double wristTheta)
{
  shTh = shouldlerTheta;
  wrTh = wristTheta;

  sh = new SimVesc(16, 0, 1.0);
  wr = new SimVesc(16, 0, 1.0);
}

void SimBackhoe::update(double dt)
{
  sh->update(dt);
  wr->update(dt);

  //TODO update torque given by vesc to simulate adding weight
  // maybe increase it by a step everytime the angle goes from
  // one spot and past another?

  // and reset it after going past dump angle?

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