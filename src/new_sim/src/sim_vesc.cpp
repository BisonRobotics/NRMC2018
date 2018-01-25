#include <vesc_access/ivesc_access.h>
#include "sim_robot/sim_vesc.h"


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
   vel = -vesc_Pgain * err + -vesc_Igain * errI; 
}

void SimVesc::setLinearVelocity(float meters_per_second)
{
  setVel = meters_per_second;
}

float SimVesc::getLinearVelocity(void)
{
  return vel;
}

float SimVesc::getTorque(void)
{
  return 0;
}

void SimVesc::setTorque(float current)
{
  //not implemented
}
