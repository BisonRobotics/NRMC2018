#include <outrigger_controller/outrigger_controller.h>
#define OUTRIGGER_ACTUATION_DUTY .5

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
  l->setDuty(OUTRIGGER_ACTUATION_DUTY);
  r->setDuty(OUTRIGGER_ACTUATION_DUTY);
  ROS_INFO("DEPLOYING");
}

void OutriggerController::retract()
{
  retracting = true;
  deploying = false;

  deployed = false;
  retracted = false;

  time_spent = 0;
  l->setDuty(-OUTRIGGER_ACTUATION_DUTY);
  r->setDuty(-OUTRIGGER_ACTUATION_DUTY);
}

void OutriggerController::update(double dt)
{
  // update is deployed/retracted status
  if (deploying || retracting)
  {
    l->getTorque();
    r->getTorque();
    time_spent += dt;
    if (time_spent >= TIME_TO_ACTUATE)
    {
      if (deploying)
      {
        deploying = false;
        deployed = true;
        l->setDuty(0);
        r->setDuty(0);
      }
      else if (retracting)
      {
        retracting = false;
        retracted = true;
        l->setDuty(0);
        r->setDuty(0);
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
