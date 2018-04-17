#include <outrigger_controller/outrigger_controller.h>

OutriggerController::OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc)
{
  l = lVesc;
  r = rVesc;

  deploying = false;
  retracting = false;
  timeSpent = 0;
}

void OutriggerController::deploy()
{
  deploying = true;
  retracting = false;
  timeSpent = 0;
  // TODO replace with real values
  l->setDuty(.5);
  r->setDuty(.5);
}

void OutriggerController::retract()
{
  retracting = true;
  deploying = false;
  timeSpent = 0;
  // TODO replace with real values
  l->setDuty(-.5);
  r->setDuty(-.5);
}

void OutriggerController::update(double dt)
{
  // TODO poll vescs for limit switch status/feedback here
  // update is deployed/retracted status
  if (deploying || retracting)
  {
    timeSpent += dt;
    if (timeSpent >= timeToActuate)
    {
       if (deploying)
       {
         deploying = false;
         deployed = true;
       }
       else if (retracting)
       {
         retracting = false;
         retracted = true;
       }
    }
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
