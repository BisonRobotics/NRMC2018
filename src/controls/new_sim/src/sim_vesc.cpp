#include <vesc_access/ivesc_access.h>
#include "sim_robot/sim_vesc.h"
#include "sensor_msgs/JointState.h"

SimVesc::SimVesc(double Pgain, double Igain, double velo_factor)
{
  vesc_Pgain = Pgain;
  vesc_Igain = Igain;
  vel = 0;
  torque = 0;
  setVel = 0;
  errI = 0;
  velocity_factor = velo_factor;
  limitSwitch = nsVescAccess::limitSwitchState::inTransit;
  pot_pos = 0.0;
  hasLimits = false;

  hitsGround = false;
}

SimVesc::SimVesc(double Pgain, double Igain, double velo_factor, 
                 double initialPos, double beginLimit, double endLimit, 
                 bool hitsGround, double groundPos)
{
  vesc_Pgain = Pgain;
  vesc_Igain = Igain;
  vel = 0;
  torque =0;
  setVel = 0;
  errI = 0;
  velocity_factor = velo_factor;
  limitSwitch = nsVescAccess::limitSwitchState::inTransit;
  pot_pos = initialPos;
  hasLimits = true;
  this->beginLimit = beginLimit;
  this->endLimit = endLimit;

  if (hitsGround)
  {
    this->hitsGround = true;
    this->groundPos = groundPos;
  }
  else
  {
    hitsGround = false;
  }
}

void SimVesc::update(double dt)
{
  // note I gain not implemented
  double err = vel - setVel;
  vel += (-vesc_Pgain) * err * dt;
  if (hitsGround)
  {
    if (pot_pos < groundPos && vel < 0) //ground is towards negative position
    {
      vel = 0;
      onGround = true;
    }
    else 
    {
      onGround = false;
    }
  }
  pot_pos += vel;
  if (hasLimits)
  {
    if (pot_pos > endLimit)
    {
      vel =0;
      setVel =0;
      limitSwitch = nsVescAccess::limitSwitchState::topOfMotion;
    }
    else if (pot_pos < beginLimit)
    {
      vel =0;
      setVel =0;
      limitSwitch = nsVescAccess::limitSwitchState::bottomOfMotion;
    }
    else
    {
      limitSwitch = nsVescAccess::limitSwitchState::inTransit;
    }
  }
  else
  {
     limitSwitch = nsVescAccess::limitSwitchState::inTransit;
  }
  if (vel >0) 
  {
     torque = 20;
  }
  else if (vel < 0)
  { 
    torque = -20;
  }
  if (onGround)
  {
    torque = 70;
  }
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
  return torque;
}

void SimVesc::setTorque(float current)
{
  setLinearVelocity(.3*current);
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