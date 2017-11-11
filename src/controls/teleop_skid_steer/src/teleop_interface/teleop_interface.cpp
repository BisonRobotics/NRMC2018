#include <teleop_interface/teleop_interface.h>
#include <wheel_params/wheel_params.h>

#define FRONT_LEFT_WHEEL_ID 0
#define FRONT_RIGHT_WHEEL_ID 1
#define BACK_RIGHT_WHEEL_ID 2
#define BACK_LEFT_WHEEL_ID 3
#define CAN_NETWORK ("can0")

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
  float max_velocity = MAX_WHEEL_VELOCITY;
  float max_torque = MAX_WHEEL_TORQUE;
  float gear_ratio = WHEEL_GEAR_RATIO;
  float torque_constant = WHEEL_TORQUE_CONSTANT;
  unsigned int pole_pairs = WHEEL_POLE_PAIRS;
  float output_ratio = WHEEL_OUTPUT_RATIO;
  char *name = (char *)WHEEL_CAN_NETWORK;
  this->fl = new VescAccess(FRONT_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque, torque_constant,
                            name, pole_pairs);
  this->fr = new VescAccess(FRONT_RIGHT_WHEEL_ID, -1.0f * gear_ratio, output_ratio, max_velocity, max_torque,
                            torque_constant, name, pole_pairs);

  this->br = new VescAccess(BACK_RIGHT_WHEEL_ID, -1.0f * gear_ratio, output_ratio, max_velocity, max_torque,
                            torque_constant, name, pole_pairs);
  this->bl = new VescAccess(BACK_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque, torque_constant,
                            name, pole_pairs);
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
  if (velocity < 0.0f)
  {
    velocity = -1.0 * velocity;
  }
  this->velocity_scale = velocity;
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
  if (number < min)
  {
    return min;
  }
  else if (number > max)
  {
    return max;
  }
  return number;
}

void TeleopInterface::update(float left_vel, float right_vel)
{
  fl->setLinearVelocity(left_vel * velocity_scale);
  bl->setLinearVelocity(left_vel * velocity_scale);
  fr->setLinearVelocity(right_vel * velocity_scale);
  br->setLinearVelocity(right_vel * velocity_scale);
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
