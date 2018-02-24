#include <outrigger_controller/outrigger_controller.h>

OutriggerController::OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc)
{
  l = lVesc;
  r = rVesc;

  deployed = false;
  retracted = true;
}

void OutriggerController::deploy()
{
  retracted = false;
  // TODO replace with real values
  l->setLinearVelocity(.5);
  r->setLinearVelocity(.5);
}

void OutriggerController::retract()
{
  deployed = false;
  // TODO replace with real values
  l->setLinearVelocity(-.5);
  r->setLinearVelocity(-.5);
}

void OutriggerController::update(double dt)
{
  // TODO poll vescs for limit switch status/feedback here
  // update is deployed/retracted status
  if (l->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion &&
      r->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    deployed = true;
  }
  else if (l->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion &&
           r->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion)
  {
    retracted = true;
  }
}

bool OutriggerController::isDeployed()
{
  return deployed;
}

bool OutriggerController::isRetracted()
{
  return retracted;
}
