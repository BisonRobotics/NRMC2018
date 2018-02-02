#include <vesc_access/ivesc_access.h>
#include "sim_robot/sim_vesc.h"
#include "sensor_msgs/JointState.h"

SimVesc::SimVesc(double Pgain, double Igain, double velo_factor)
{
  vesc_Pgain = Pgain;
  vesc_Igain = Igain;
  vel =0;
  setVel =0;
  errI =0;
    velocity_factor = velo_factor;
}

void SimVesc::update(double dt)
{
   /*
   double err = setVel - vel;
   errI += err * dt; 
   vel = -vesc_Pgain * err + -vesc_Igain * errI * dt; 
  */
   //vel = setVel;
   double err = vel - setVel;
   vel += (-vesc_Pgain) * err * dt;
}

void SimVesc::setLinearVelocity(float meters_per_second)
{
  setVel = meters_per_second * velocity_factor;
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
