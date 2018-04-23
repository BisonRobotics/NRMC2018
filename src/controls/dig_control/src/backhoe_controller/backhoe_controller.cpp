#include <backhoe_controller/backhoe_controller.h>
#include <cmath>
#include <ros/ros.h>

BackhoeController::BackhoeController(iSafetyController *backhoeSafety, iSafetyController *linearSafety)

{
  this->backhoe_safety = backhoeSafety;
  this->linear_safety = linearSafety;
}

void BackhoeController::setShoulderSetpoint(double angle)
{
  if (getIsInit())
  {
    backhoe_safety->setPositionSetpoint(angle);
  }
}

void BackhoeController::setWristSetpoint(double angle)
{
  if (getIsInit())
  {
    linear_safety->setPositionSetpoint(angle);
  }
}

void BackhoeController::setShoulderTorque(double torque)
{
  if (getIsInit())
  {
    backhoe_safety->setTorque(torque);
  }
}

void BackhoeController::setShoulderVelocity(double velocity)
{
  if (getIsInit())
  {
    backhoe_safety->setVelocity(velocity);
  }
}

void BackhoeController::abandonShoulderPositionSetpointAndSetTorqueWithoutStopping(double torque)
{
  if (getIsInit())
  {
    backhoe_safety->abandonPositionSetpointAndSetTorqueWithoutStopping(torque);
  }
}

void BackhoeController::setWristVelocity(double velocity)
{
  if (getIsInit())
  {
    linear_safety->setVelocity(velocity);
  }
}

void BackhoeController::update(double dt)
{
  if (getIsInit())
  {
    safetyCheck();
  //  ROS_INFO("central drive update");
    backhoe_safety->update(dt);
    //ROS_INFO("Linear actuator update");
    linear_safety->update(dt);
  }
}

void BackhoeController::safetyCheck()
{
  if ((backhoe_safety->getLinearVelocity() > 0) &&
      backhoe_safety->getPositionEstimate() > backhoe_safety->getSafetyPosition() &&
      linear_safety->getPositionEstimate() > linear_safety->getSafetyPosition())
  {
    ROS_INFO("BC says safety stop 1");
    backhoe_safety->stop();
    if (linear_safety->getLinearVelocity() > 0)
    {
      linear_safety->stop();
      ROS_INFO("BC says safety stop 1");
    }
  }
}

bool BackhoeController::shoulderAtSetpoint()
{
  return backhoe_safety->isAtSetpoint();
}

bool BackhoeController::wristAtSetpoint()
{
  return linear_safety->isAtSetpoint();
}

double BackhoeController::getShoulderTorque()
{
  return backhoe_safety->getTorque();
}

double BackhoeController::getShoulderVelocity()
{
  return backhoe_safety->getLinearVelocity();
}

bool BackhoeController::hasHitGround()
{
  return fabs(backhoe_safety->getTorque()) > fabs(this->ground_torque);
}

bool BackhoeController::getIsInit()
{
  return backhoe_safety->getInitStatus() && linear_safety->getInitStatus();
}
