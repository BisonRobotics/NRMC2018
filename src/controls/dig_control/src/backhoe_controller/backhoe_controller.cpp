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
    ROS_INFO ("Back hoe set velocity through API %.4f", velocity);
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
    ROS_INFO ("Wrist Set velocity through API %.4f", velocity);
  }
}

void BackhoeController::setWristTorque (double torque)
{
  if (getIsInit())
  {
    linear_safety->setTorque (torque);
  }
}


void BackhoeController::update(double dt)
{
  if (getIsInit())
  {
    safetyCheck();
    backhoe_safety->update(dt);
    linear_safety->update(dt);
  }
  else
  {
      ROS_ERROR("UPDATE CALLED WITHOUT INIT");
  }
}

void BackhoeController::safetyCheck()
{

  // if were are greater than the safety distance and we want to move up on the central, stop it.
  // if we are greater than the safety distance on the linear and the central is above its safety distance, stop the linear

  if (backhoe_safety->getPositionEstimate() > backhoe_safety->getSafetyPosition () &&
      (backhoe_safety->getCommandedTorque() > .001 || backhoe_safety->getCommandedVelocity() > .001)
      && linear_safety->getPositionEstimate() > linear_safety->getSafetyPosition())
  {
    backhoe_safety->stop();
    ROS_WARN ("Backhoe stopped");
  }

  if (backhoe_safety->getPositionEstimate() > backhoe_safety->getSafetyPosition() &&
      (linear_safety->getCommandedTorque() > .001 || linear_safety->getCommandedVelocity () > .001))
  {
    linear_safety->stop();
    ROS_WARN ("Linear stopped!");
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

double BackhoeController::getPositionEstimate()
{
    return backhoe_safety->getPositionEstimate();
}

bool BackhoeController::hasHitGround()
{
  return fabs(backhoe_safety->getTorque()) > fabs(this->ground_torque);
}

bool BackhoeController::getIsInit()
{
  return backhoe_safety->getInitStatus() && linear_safety->getInitStatus();
}

void BackhoeController::stopWrist ()
{
  linear_safety->stop();
}

void BackhoeController::stopShoulder()
{
  backhoe_safety->stop();
}
