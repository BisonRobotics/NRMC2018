#include <backhoe_controller/backhoe_controller.h>
#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

BackhoeController::BackhoeController(double initialShoulderTheta, double initialWristTheta, iVescAccess *shVesc, iVescAccess *wrVesc)
{
    shoulderSetPoint = initialShoulderTheta;
    shoulderAngleEst = initialShoulderTheta;
    sh = shVesc;

    wristSetPoint = initialWristTheta;
    wristAngleEst = initialWristTheta;
    wr = wrVesc;
}

void BackhoeController::setShoulderSetPoint(double angle)
{
    shoulderSetPoint = angle;
}

void BackhoeController::setWristSetPoint(double angle)
{
    wristSetPoint = angle;
}

void BackhoeController::update(double dt)
{
  shoulderAngleEst += sh->getLinearVelocity() * dt;
  double error = shoulderSetPoint - shoulderAngleEst;
  //TODO change this constant to something like a gain
  if (error > .2) error =.2;
  else if (error < -.2) error = -.2;
  //TODO change this constant to a gain
  sh->setLinearVelocity(.5 * error);

  wristAngleEst += wr->getLinearVelocity() * dt;
  error = wristSetPoint - wristAngleEst;
  //TODO change this constant to something like a gain
  if (error > .2) error =.2;
  else if (error < -.2) error = -.2;
  //TODO change this constant to a gain
  wr->setLinearVelocity(.5 * error);

}
