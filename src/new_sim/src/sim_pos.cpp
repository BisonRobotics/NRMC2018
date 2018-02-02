#include "sim_robot/sim_pos.h"

SimPos::SimPos(double xnoise1, double ynoise1, double thetanoise1)
{
  xnoise = xnoise1;
  ynoise = ynoise1;
  thetanoise = thetanoise1;
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
  //todo, add noise
  x = x1;
  y = y1;
  theta = theta1;
}
