#include <outrigger_controller/outrigger_controller.h>

OutriggerController::OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc)
{
  l = lVesc;
  r = rVesc;

  isDeployed =false;
  isRetracted = true;
}

void OutriggerController::deploy()
{
  isRetracted = false;
  //TODO replace with real values
  l->setLinearVelocity(.5);
  r->setLinearVelocity(.5);
}

void OutriggerController::retract()
{
  isDeployed = false;
  //TODO replace with real values
  l->setLinearVelocity(-.5);
  r->setLinearVelocity(-.5);
}

void OutriggerController::update(double dt);
{
  //TODO poll vescs for limit switch status/feedback here
  //update is deployed/retracted status
}

bool OutriggerController::isDeployed()
{
  return isDeployed;
}

bool OutriggerController::isRetracted()
{
  return isRetracted;
}