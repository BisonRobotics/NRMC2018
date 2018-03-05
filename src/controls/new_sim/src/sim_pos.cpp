#include "sim_robot/sim_pos.h"

SimPos::SimPos(double xnoise1, double ynoise1, double thetanoise1)
{
  xnoise = xnoise1;
  ynoise = ynoise1;
  thetanoise = thetanoise1;
  is_floating = false;
}

double SimPos::getX()
{
  return x;
}

double SimPos::getY()
{
  return y;
}

double SimPos::getTheta()
{
  return theta;
}

ReadableSensors::ReadStatus SimPos::receiveData()
{
  return ReadableSensors::ReadStatus::READ_SUCCESS;
}

void SimPos::update(double x1, double y1, double theta1)
{
  // todo, add noise
  x = x1;
  y = y1;
  theta = theta1;
}

bool SimPos::isFloating()
{
  return is_floating;
}

void SimPos::setIsFloating(bool is_floating)
{
  this->is_floating = is_floating;
}