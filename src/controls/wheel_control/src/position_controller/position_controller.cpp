#include <position_controller/position_controller.h>
#include <math.h>

#define FRONT_LEFT_WHEEL_ID 0
#define FRONT_RIGHT_WHEEL_ID 1
#define BACK_RIGHT_WHEEL_ID 2
#define BACK_LEFT_WHEEL_ID 3
#define CAN_NETWORK ("can0")

PositionController::PositionController(float velocity, float tolerance)
{
  float max_velocity = 20.0f;
  float max_torque = 20.0f;
  float gear_ratio = 12.0f;
  float torque_constant = 4.0f;
  unsigned int pole_pairs = 8;
  float output_ratio = 10.0f;
  char *name = (char *)CAN_NETWORK;
  setVelocity(velocity);
  setTolerance(tolerance);
  this->goal_received = false;
  this->position_received = false;
  this->fleft_wheel = new VescAccess(FRONT_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                     torque_constant, name, pole_pairs);

  this->fright_wheel = new VescAccess(FRONT_RIGHT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                      torque_constant, name, pole_pairs);

  this->bright_wheel = new VescAccess(BACK_RIGHT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                      torque_constant, name, pole_pairs);
  this->bleft_wheel = new VescAccess(BACK_LEFT_WHEEL_ID, gear_ratio, output_ratio, max_velocity, max_torque,
                                     torque_constant, name, pole_pairs);
  this->currently_moving = false;
}

PositionController::PositionController(float velocity, float tolerance, iVescAccess *fl, iVescAccess *fr,
                                       iVescAccess *br, iVescAccess *bl)
{
  setVelocity(velocity);
  setTolerance(tolerance);
  this->currently_moving = false;
  this->bleft_wheel = bl;
  this->bright_wheel = br;
  this->fright_wheel = fr;
  this->fleft_wheel = fl;
  this->goal_received = false;
  this->position_received = false;
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
  }
}

void PositionController::setTolerance(float tolerance)
{
  this->tolerance = tolerance;
  this->tol_sqr = tolerance * tolerance;
}

float PositionController::getTolerance(void)
{
  return (tolerance);
}

void PositionController::setCurrentState(float position_x, float position_y)
{
  this->current_state.x = position_x;
  this->current_state.y = position_y;
  this->position_received = true;
}

void PositionController::setInitialState(void)
{
  initial_state = current_state;
}

void PositionController::startVescs(void)
{
  currently_moving = true;
  fleft_wheel->setLinearVelocity(this->velocity);
  fright_wheel->setLinearVelocity(this->velocity);
  bright_wheel->setLinearVelocity(this->velocity);
  bleft_wheel->setLinearVelocity(this->velocity);
}

void PositionController::update(float position_x, float position_y)
{
  setCurrentState(position_x, position_y);
  if (goal_received)
  {
    if (position_received)
    {
      if (!inTolerance() && !exceededDistance())
      {
        if (!currently_moving)
        {
          startVescs();
        }
      }
      else
      {
        closeGoal();
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
  return (getDistanceTravelledSqr() > (distance * distance));
}

bool PositionController::inTolerance(void)
{
  return (this->tol_sqr >= fabs(getDistanceTravelledSqr() - this->distance));
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
