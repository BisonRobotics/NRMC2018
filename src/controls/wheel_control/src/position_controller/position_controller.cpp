#include <position_controller/position_controller.h>
#include <math.h>

#define FRONT_LEFT_WHEEL_ID 0
#define FRONT_RIGHT_WHEEL_ID 1
#define BACK_RIGHT_WHEEL_ID 2
#define BACK_LEFT_WHEEL_ID 3
#define CAN_NETWORK ("can0")

PositionController::PositionController(float velocity)
{
  float max_velocity = 20.0f;
  float max_torque = 20.0f;
  float gear_ratio = 12.0f;
  float torque_constant = 4.0f;
  unsigned int pole_pairs = 8;
  float output_ratio = 10.0f;
  char *name = (char *)CAN_NETWORK;
  iVescAccess *fl = new VescAccess(FRONT_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                   torque_constant, name, pole_pairs);
  iVescAccess *fr = new VescAccess(FRONT_RIGHT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                   torque_constant, name, pole_pairs);

  iVescAccess *br = new VescAccess(BACK_RIGHT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                   torque_constant, name, pole_pairs);
  iVescAccess *bl = new VescAccess(BACK_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                   torque_constant, name, pole_pairs);
  PositionController(velocity, fl, fr, br, bl);
  this->internally_alloc = true;
}

PositionController::PositionController(float velocity, iVescAccess *fl, iVescAccess *fr, iVescAccess *br,
                                       iVescAccess *bl)
{
  setVelocity(velocity);
  this->distance = 0.0f;
  initial_state.x = 0.0f;
  initial_state.y = 0.0f;
  current_state.x = 0.0f;
  current_state.y = 0.0f;
  this->currently_moving = false;
  this->bleft_wheel = bl;
  this->bright_wheel = br;
  this->fright_wheel = fr;
  this->fleft_wheel = fl;
  this->goal_received = false;
  this->position_received = false;
  this->internally_alloc = false;
}

void PositionController::setVelocity(float velocity)
{
  velocity = fabs(velocity);
  this->velocity = velocity;
}

float PositionController::getVelocity(void)
{
  return (this->velocity);
}

float PositionController::getDistance(void)
{
  return (distance);
}

void PositionController::setDistance(float distance)
{
  if (!goal_received)
  {
    this->distance = fabs(distance);
    this->goal_received = true;
    this->distance_square = distance * distance;
  }
}

void PositionController::setCurrentState(float position_x, float position_y)
{
  this->current_state.x = position_x;
  this->current_state.y = position_y;
  this->position_received = true;
}

void PositionController::setInitialState(void)
{
  this->initial_state.x = this->current_state.x;
  this->initial_state.y = this->current_state.y;
}

void PositionController::startVescs(void)
{
  this->currently_moving = true;
  setInitialState();
  fleft_wheel->setLinearVelocity(this->velocity);
  fright_wheel->setLinearVelocity(this->velocity);
  bright_wheel->setLinearVelocity(this->velocity);
  bleft_wheel->setLinearVelocity(this->velocity);
}

void PositionController::update(float position_x, float position_y)
{
  setCurrentState(position_x, position_y);
  if (this->goal_received && this->position_received)
  {
    if (exceededDistance())
    {
      closeGoal();
    }
    else
    {
      if (!currently_moving)
      {
        startVescs();
      }
    }
  }
}

bool PositionController::isMoving(void)
{
  return currently_moving;
}

float PositionController::getDistanceTravelledSqr(void)
{
  float euclid_dist_sqr;
  euclid_dist_sqr = ((initial_state.x - current_state.x) * (initial_state.x - current_state.x));
  euclid_dist_sqr += ((initial_state.y - current_state.y) * (initial_state.y - current_state.y));
  return (euclid_dist_sqr);
}

bool PositionController::exceededDistance(void)
{
  return (getDistanceTravelledSqr() >= (this->distance * this->distance));
}

void PositionController::closeGoal(void)
{
  goal_received = false;
  stopVescs();
}

void PositionController::stopVescs(void)
{
  currently_moving = false;
  fleft_wheel->setLinearVelocity(0.0f);
  fright_wheel->setLinearVelocity(0.0f);
  bright_wheel->setLinearVelocity(0.0f);
  bleft_wheel->setLinearVelocity(0.0f);
}

PositionController::~PositionController()
{
  if (internally_alloc)
  {
    delete fleft_wheel;
    delete fright_wheel;
    delete bright_wheel;
    delete bleft_wheel;
  }
}
