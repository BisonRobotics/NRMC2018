#include "safety_vesc/linear_safety_controller.h"


LinearSafetyController::LinearSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc, bool in_velocity)
: SafetyController::SafetyController(vesc, params, in_velocity)
{

}

void LinearSafetyController::updatePosition(double dt)
{
    this->position_estimate += vesc->getLinearVelocity()*dt;
    SafetyController::updatePosition(dt);
}

void LinearSafetyController::init ()
{
    static constexpr float start_torque = -1.0f;
    this->vesc->setTorque(start_torque);
    while (vesc->getLimitSwitchState() != nsVescAccess::limitSwitchState::bottomOfMotion);
    this->vesc->setTorque(0.0);
    this->position_estimate = params.lower_limit_position;
    this->is_init = true;
}