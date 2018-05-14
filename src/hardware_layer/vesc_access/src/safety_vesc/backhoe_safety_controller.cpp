#include "safety_vesc/backhoe_safety_controller.h"
#include <ros/ros.h>

BackhoeSafetyController::BackhoeSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc)
  : SafetyController(vesc, params)
{
  torque_magnitude = 8.0;
}

bool BackhoeSafetyController::init()
{
  this->position_estimate = this->vesc->getPotPosition();
  is_init = true;
  return is_init;
}

void BackhoeSafetyController::updatePositionEstimate(double dt)
{
  this->position_estimate = vesc->getPotPosition();
  // SafetyController::updatePositionEstimate(dt);
}