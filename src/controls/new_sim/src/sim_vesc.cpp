#include <vesc_access/ivesc_access.h>
#include "sim_robot/sim_vesc.h"
#include "sensor_msgs/JointState.h"

SimVesc::SimVesc(double Pgain, double Igain, double velo_factor)
{
  vesc_Pgain = Pgain;
  vesc_Igain = Igain;
  vel = 0;
  setVel = 0;
  errI = 0;
  velocity_factor = velo_factor;
  limitSwitch = nsVescAccess::limitSwitchState::inTransit;
  pot_pos = 0.0;
}

void SimVesc::update(double dt)
{
  // note I gain not implemented
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
  if (vel >0) return 100;
  else return -100;
}

void SimVesc::setTorque(float current)
{
  // not implemented
}

void SimVesc::setLimitSwitchState(nsVescAccess::limitSwitchState state)
{
  limitSwitch = state;
}

nsVescAccess::limitSwitchState SimVesc::getLimitSwitchState(void)
{
  return limitSwitch;
}

float SimVesc::getPotPosition(void)
{
  return pot_pos;
}

void SimVesc::setPotPosition(float pos)
{
  pot_pos = pos;
}