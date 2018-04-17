#include <teleop_interface/teleop_interface.h>
#include <wheel_params/wheel_params.h>

TeleopInterface::TeleopInterface(float velocity, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl)
{
  this->fl = fl;
  this->fr = fr;
  this->br = br;
  this->bl = bl;
  initializeVars(velocity);
  this->internally_alloc = false;
}

TeleopInterface::TeleopInterface(float velocity)
{
  this->fl = new VescAccess(front_left_param);
  this->fr = new VescAccess(front_right_param);
  this->br = new VescAccess(back_right_param);
  this->bl = new VescAccess(back_left_param);
  initializeVars(velocity);
  this->internally_alloc = true;
}

void TeleopInterface::initializeVars(float velocity)
{
  setVelocity(velocity);
  stopMotors();
}

float TeleopInterface::getVelocity()
{
  return this->velocity_scale;
}

void TeleopInterface::setVelocity(float velocity)
{
  this->velocity_scale = std::abs(velocity);
}

void TeleopInterface::stopMotors()
{
  fl->setLinearVelocity(0.0f);
  fr->setLinearVelocity(0.0f);
  br->setLinearVelocity(0.0f);
  bl->setLinearVelocity(0.0f);
}

float TeleopInterface::clamp(float number, float max, float min)
{
  return std::max(min, std::min(number, max));
}

void TeleopInterface::update(float left_vel, float right_vel)
{
  if (fabs(left_vel) > .001)
  {
    fl->setLinearVelocity(left_vel * velocity_scale);
    bl->setLinearVelocity(left_vel * velocity_scale);
  }
  else
  {
    fl->setTorque(0);
    bl->setTorque(0);
  }

  if (fabs(right_vel) > .001)
  {
    fr->setLinearVelocity(right_vel * velocity_scale);
    br->setLinearVelocity(right_vel * velocity_scale);
  }
  else
  {
    fr->setTorque(0);
    br->setTorque(0);
  }

  fl->getTorque();
  bl->getTorque();
  br->getTorque();
  fr->getTorque();
}

TeleopInterface::~TeleopInterface()
{
  if (internally_alloc)
  {
    delete fl;
    delete fr;
    delete bl;
    delete br;
  }
}
