#include "safety_vesc/safety_vesc.h"
#include <math.h>

SafetyVesc::SafetyVesc(iVescAccess *vesc, safetyvesc::joint_params_t params, bool in_velocity)
{
    this->params = params;
    this->vesc = vesc;
    this->in_velocity = in_velocity;
    this->is_init = false;
    this->set_position = 0;
    this->set_velocity = 0;
}

void SafetyVesc::setPositionSetpoint(double position)
{
    if (is_init && !in_velocity)
    {
        this->set_position = position;
    }
}

void SafetyVesc::setVelocitySetpoint(double velocity)
{
    if (is_init && in_velocity)
    {
        this->set_velocity = velocity;
    }
}

double SafetyVesc::updateVelocity(void)
{
    if (is_init)
    {
        if (!in_velocity)
        {
            set_velocity = params.gain*(position_estimate - set_position);
        }
        if (set_position <= params.minimum_pos && set_velocity < 0)
        {
            set_velocity = 0;
        }
        if (set_position >= params.maximum_pos && set_velocity > 0)
        {
            set_velocity = 0;
        }
        vesc->setLinearVelocity(set_velocity);
    }
}

bool SafetyVesc::isAtSetpoint(void)
{
    return fabs(position_estimate) < fabs(params.setpoint_tolerance);
}

void SafetyVesc::updatePosition(double dt)
{
    if (is_init)
    {
        switch (vesc->getLimitSwitchState()) {
            case nsVescAccess::limitSwitchState::bottomOfMotion:
                this->position_estimate = params.lower_limit_position;
                break;
            case nsVescAccess::limitSwitchState::topOfMotion:
                this->position_estimate = params.upper_limit_position;
                break;
            case nsVescAccess::limitSwitchState::inTransit:
                break;
        }
    }
}
