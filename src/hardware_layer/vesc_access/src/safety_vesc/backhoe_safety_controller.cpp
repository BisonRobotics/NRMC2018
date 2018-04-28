#include "safety_vesc/backhoe_safety_controller.h"

BackhoeSafetyController::BackhoeSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc)
  : SafetyController(vesc, params)
{
}

bool BackhoeSafetyController::init()
{
  this->position_estimate = this->vesc->getPotPosition();
  is_init = true;
  return true;
}

void BackhoeSafetyController::updatePositionEstimate(double dt)
{
  this->position_estimate = vesc->getPotPosition();
 // SafetyController::updatePositionEstimate(dt);
}