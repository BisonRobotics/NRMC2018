#include "sim_robot/sim_outriggers.h"

SimOutriggers::SimOutriggers(double initialPosL, double initialPosR)
{
  l = new SimVesc(16, 0, 1.0);
  r = new SimVesc(16, 0, 1.0);

  posL = initialPosL;
  posR = initialPosR;
}

iVescAccess* SimOutriggers::getLVesc()
{
  return l;
}

iVescAccess* SimOutriggers::getRVesc()
{
  return r;
}

void SimOutriggers::update(double dt)
{
  l->update(dt);
  r->update(dt);

  posL += l->getLinearVelocity() * dt;
  posR += r->getLinearVelocity() * dt;

  l->setLimitSwitchState(nsVescAccess::limitSwitchState::inTransit);
  r->setLimitSwitchState(nsVescAccess::limitSwitchState::inTransit);

  if (posL < 0)
  {
    posL = 0;
    l->setLinearVelocity(0);
  }
  else if (posL > .1778)
  {
    posL = .1778;
    l->setLinearVelocity(0);
  }

  if (posR < 0)
  {
    posR = 0;
    r->setLinearVelocity(0);
  }
  else if (posR > .1778)
  {
    posR = .1778;
    r->setLinearVelocity(0);
  }
}

double SimOutriggers::getPosL()
{
  return posL;
}

double SimOutriggers::getPosR()
{
  return posR;
}
