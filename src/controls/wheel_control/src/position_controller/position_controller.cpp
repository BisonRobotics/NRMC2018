#include <position_controller/position_controller.h>
#include <math.h>

PositionController::PositionController (float velocity, float tolerance){
  setVelocity (velocity);
  setTolerance (tolerance);
  this->goal_received = false;
  this->position_received = false;
}

void PositionController::setVelocity (float velocity){
  velocity = fabs (velocity);
  this->velocity = velocity;

}

float PositionController::getVelocity (void){
  return (this->velocity);
}

float PositionController::getDistance (void){
  return (distance);
}

void PositionController::setDistance (float distance){
  this->distance = distance;
}

void PositionController::setTolerance (float tolerance){
  this->tolerance = tolerance;
}

float PositionController::getTolerance (void){
  return (tolerance);
}

void PositionController::setPosition (float position){
  this->position = position;
}


void PositionController::update (float position){
  setPosition (position);  
}

bool PositionController::isMoving (void){
  if (goal_received){
    if (position_received){
      float delta = fabs(position - distance);
      
    }
  }
}
