#include <vesc_access/ivesc_access.h>

SimVesc::SimVesc(double Pgain, double Igain)
{
  vesc_Pgain = Pgain;
  vesc_Igain = Igain;
  vel =0;
  setVel =0;
  errI =0;
}

void SimVesc::update(double dt)
{
   double err = setVel - vel;
   errI += err; 
   vel = -Pgain * err + -Igain * errI; 
}

void SimVesc::setLinearVelocity(float meters_per_second)
{
  setVel = meters_per_second;
}

void SimVesc::getLinearVelocity(void)
{
  return vel;
}

float SimVesv::getTorque(void)
{
  return 0;
}

float SimVesc::setTorque(float current)
{
  //not implemented
}