#include "safety_vesc/backhoe_safety_controller.h"
#include <ros/ros.h>
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
  //ROS_INFO ("POSITION ESTIMATE CENTRAL: %.4f", position_estimate);
 // SafetyController::updatePositionEstimate(dt);
}