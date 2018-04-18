#include <outrigger_controller/outrigger_controller.h>

OutriggerController::OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc)
{
  l = lVesc;
  r = rVesc;

  deploying = false;
  retracting = false;
  
  deployed = false;
  retracted = false;
  
  time_spent = 0;
}

void OutriggerController::deploy()
{
  deploying = true;
  retracting = false;
  
  deployed = false;
  retracted = false;
  
  time_spent = 0;
  // TODO replace with real values
  l->setDuty(.5);
  r->setDuty(.5);
}

void OutriggerController::retract()
{
  retracting = true;
  deploying = false;
  
  deployed = false;
  retracted = false;
  
  time_spent = 0;
  // TODO replace with real values
  l->setDuty(-.5);
  r->setDuty(-.5);
}

void OutriggerController::update(double dt)
{
  // update is deployed/retracted status
  if (deploying || retracting)
  {
    time_spent += dt;
    if (time_spent >= TIME_TO_ACTUATE)
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
