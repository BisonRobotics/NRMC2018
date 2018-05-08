#include "sim_robot/sim_imu.h"

SimImu::SimImu(double xnoise1, double ynoise1, double omeganoise1)
{
  xnoise = xnoise1;
  ynoise = ynoise1;
  omeganoise = omeganoise1;
}

double SimImu::getX()
{
  return x;
}

double SimImu::getY()
{
  return y;
}

double SimImu::getOmega()
{
  return omega;
}

ReadableSensors::ReadStatus SimImu::receiveData()
{
  return ReadableSensors::ReadStatus::READ_SUCCESS;
}

void SimImu::update(double x1, double y1, double omega1)
{
  // add noise eventually
  x = x1;
  y = y1;
  omega = omega1;
}

tf2::Quaternion SimImu::getOrientation()
{
  return (tf2::Quaternion (0,0,0,1));
}