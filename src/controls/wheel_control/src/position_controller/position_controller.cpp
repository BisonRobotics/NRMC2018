#include <position_controller/position_controller.h>
#include <math.h>
#include <wheel_params/wheel_params.h>

void PositionController::initializeMembers(float velocity)
{
  setVelocity(velocity);
  this->distance = 0.0f;
  initial_state.x = 0.0f;
  initial_state.y = 0.0f;
  current_state.x = 0.0f;
  current_state.y = 0.0f;
  this->currently_moving = false;
  this->goal_received = false;
  this->position_received = false;
}

PositionController::PositionController(float velocity)
{
  this->front_left_wheel = new VescAccess(front_left_param);
  this->front_right_wheel = new VescAccess(front_right_param);
  this->back_right_wheel = new VescAccess(back_right_param);
  this->back_left_wheel = new VescAccess(back_left_param);
  initializeMembers(velocity);
  this->internally_alloc = true;
}

PositionController::PositionController(float velocity, iVescAccess *fl, iVescAccess *fr, iVescAccess *br,
                                       iVescAccess *bl)
{
  this->back_left_wheel = bl;
  this->back_right_wheel = br;
  this->front_right_wheel = fr;
  this->front_left_wheel = fl;
  this->internally_alloc = false;
  initializeMembers(velocity);
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
  if (!goal_received && position_received)
  {
    this->distance = fabs(distance);
    this->goal_received = true;
    this->distance_square = distance * distance;
    setInitialState();
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
  //  setInitialState();
  front_left_wheel->setLinearVelocity(this->velocity);
  front_right_wheel->setLinearVelocity(this->velocity);
  back_right_wheel->setLinearVelocity(this->velocity);
  back_left_wheel->setLinearVelocity(this->velocity);
}

void PositionController::update(float position_x, float position_y)
{
  setCurrentState(position_x, position_y);
  if (this->goal_received && this->position_received)
  {
    if (exceededDistance() && currently_moving)
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
  return (getDistanceTravelledSqr() >= (this->distance_square));
}

void PositionController::closeGoal(void)
{
  goal_received = false;
  stopVescs();
}

void PositionController::stopVescs(void)
{
  currently_moving = false;
  front_left_wheel->setLinearVelocity(0.0f);
  front_right_wheel->setLinearVelocity(0.0f);
  back_right_wheel->setLinearVelocity(0.0f);
  back_left_wheel->setLinearVelocity(0.0f);
}

PositionController::~PositionController()
{
  if (internally_alloc)
  {
    delete front_left_wheel;
    delete front_right_wheel;
    delete back_right_wheel;
    delete back_left_wheel;
  }
}

float PositionController::getDistanceRemaining(void)
{
  return fabs(distance - sqrt(getDistanceTravelledSqr()));
}
