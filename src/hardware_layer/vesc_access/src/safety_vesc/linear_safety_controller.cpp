#include "safety_vesc/linear_safety_controller.h"
#include <ros/ros.h>

LinearSafetyController::LinearSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc)
  : SafetyController::SafetyController(vesc, params)
{
  has_set_init_vel = false;
}

void LinearSafetyController::updatePositionEstimate(double dt)
{
  this->position_estimate +=  -1.0*4.5657*vesc->getLinearVelocity() * dt;
  SafetyController::updatePositionEstimate(dt);
}

bool LinearSafetyController::init()
{
  static constexpr float start_torque = -5.0f;

  if (this->vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    this->vesc->setLinearVelocity(0);
    this->position_estimate = params.lower_limit_position;
    this->is_init = true;
  }
  else
  {
    this->is_init = false;
    this->vesc->setTorque(start_torque);
  }
  return is_init;
}