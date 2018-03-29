#include "safety_vesc/backhoe_safety.h"


BackhoeSafety::BackhoeSafety(safetyvesc::joint_params_t params, iVescAccess *vesc, bool in_velocity)
: SafetyVesc (vesc, params, in_velocity)
{

}


void BackhoeSafety::init()
{
    this->position_estimate = this->vesc->getPotPosition ();
    is_init = true;
}

void BackhoeSafety::updatePosition(double dt)
{
    this->position_estimate = this->vesc->getPotPosition();
}