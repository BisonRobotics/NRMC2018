#include <backhoe_controller/backhoe_controller.h>
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

BackhoeController::BackhoeController(double initialShoulderTheta, double initialWristTheta, iVescAccess *shVesc,
                                     iVescAccess *wrVesc)
{
  shoulderSetpoint = initialShoulderTheta;
  shoulderAngleEst = initialShoulderTheta;
  isShoulderAtSetpoint = true;
  sh = shVesc;

  wristSetpoint = initialWristTheta;
  wristAngleEst = initialWristTheta;
  isWristAtSetpoint = true;
  wr = wrVesc;
}

void BackhoeController::setShoulderSetpoint(double angle)
{
  shoulderSetpoint = angle;
}

void BackhoeController::setWristSetpoint(double angle)
{
  wristSetpoint = angle;
}

void BackhoeController::update(double dt)
{
  shoulderAngleEst += sh->getLinearVelocity() * dt;
  double error = shoulderSetpoint - shoulderAngleEst;
  // TODO change this constant to something like a gain
  if (error > .2)
    error = .2;
  else if (error < -.2)
    error = -.2;
  // TODO change this constant to a gain
  sh->setLinearVelocity(.5 * error);
  // TODO change cutoff to parameter/gain
  isShoulderAtSetpoint = (error < .05);

  wristAngleEst += wr->getLinearVelocity() * dt;
  error = wristSetpoint - wristAngleEst;
  // TODO change this constant to something like a gain
  if (error > .2)
    error = .2;
  else if (error < -.2)
    error = -.2;
  // TODO change this constant to a gain
  wr->setLinearVelocity(.5 * error);
  // TODO change cutoff to parameter/gain
  isWristAtSetpoint = (error < .04);
}

bool BackhoeController::shoulderAtSetpoint()
{
  return isShoulderAtSetpoint;
}

bool BackhoeController::wristAtSetpoint()
{
  return isWristAtSetpoint;
}