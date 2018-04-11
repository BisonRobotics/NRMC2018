#include "safety_vesc/backhoe_safety_controller.h"


BackhoeSafetyController::BackhoeSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc, bool in_velocity)
: SafetyController (vesc, params, in_velocity)
{

}


bool BackhoeSafetyController::init()
{
    this->position_estimate = this->vesc->getPotPosition ();
    is_init = true;
    return true;
}

void BackhoeSafetyController::updatePosition(double dt)
{
    this->position_estimate = this->vesc->getPotPosition();
}

