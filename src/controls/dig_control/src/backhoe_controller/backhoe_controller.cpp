#include <backhoe_controller/backhoe_controller.h>
#include <cmath>

BackhoeController::BackhoeController(iSafetyVesc *backhoeSafety, iSafetyVesc *linearSafety)

{
  this->backhoe_safety = backhoeSafety;
  this->linear_safety = linearSafety;
}


void BackhoeController::setShoulderSetpoint(double angle)
{
  if (getIsInit()) {
    backhoe_safety->setPositionSetpoint(angle);
  }
}

void BackhoeController::setWristSetpoint(double angle)
{
  if (getIsInit()) {
    linear_safety->setPositionSetpoint(angle);
  }
}

void BackhoeController::setShoulderVelocity(double velocity)
{
  if (getIsInit()) {
    backhoe_safety->setVelocitySetpoint(velocity);
  }
}

void BackhoeController::setWristVelocity(double velocity)
{
  if (getIsInit()) {
    linear_safety->setVelocitySetpoint(velocity);
  }
}

void BackhoeController::update(double dt)
{
  if (getIsInit()) {
    backhoe_safety->updatePosition(dt);
    linear_safety->updatePosition(dt);
    safetyCheck();
    backhoe_safety->updateVelocity();
    linear_safety->updateVelocity();
  }
}

void BackhoeController::safetyCheck()
{
  if (backhoe_safety->getVelocity() < 0 && backhoe_safety->getPosition() < backhoe_safety->getSafetyPosition() &&
      backhoe_safety->getPosition() > backhoe_safety->getSafetyPosition())
  {
    backhoe_safety->stop();
    if (linear_safety->getVelocity() > 0)
    {
      linear_safety->stop();
    }
  }
}

void BackhoeController::init()
{
  linear_safety->init ();
  backhoe_safety->init ();
}

bool BackhoeController::shoulderAtSetpoint()
{
  return backhoe_safety->isAtSetpoint();
}

bool BackhoeController::wristAtSetpoint()
{
  return linear_safety->isAtSetpoint();
}

double BackhoeController::getWeightInBackhoe()
{
  // calculate weight
  return 0.0;
}

double BackhoeController::getWeightInBucket()
{
  return 0.0;
}

void BackhoeController::tareBackhoe()
{
}

void BackhoeController::tareBucket()
{
}

bool BackhoeController::getIsInit()
{
  return linear_safety->isInit() && backhoe_safety->isInit();
}