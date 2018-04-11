#include "safety_vesc/safety_controller.h"
#include <math.h>
#include <sstream>

SafetyController::SafetyController(iVescAccess *vesc, safetycontroller::joint_params_t params, bool in_velocity)
{
    this->params = params;
    this->vesc = vesc;
    this->in_velocity = in_velocity;
    this->is_init = false;
    this->set_position = 0;
    this->set_velocity = 0;
    this->position_estimate =0;
}

void SafetyController::setPositionSetpoint(double position)
{
    performIsInit();
    if (!in_velocity)
    {
        if  (position > params.maximum_pos || position < params.minimum_pos){
            std::stringstream ss;
            ss << "Out of bounds at : " << position;
            this->set_position = this->position_estimate;
            this->stop();
            throw BackhoeException (ss.str ());
        }
        this->set_position = position;
    }
    else
    {
        throw BackhoeException ("tried to set position in velocity mode");
    }
}

void SafetyController::performIsInit ()
{
    if (!is_init){
        this->stop();
        throw BackhoeException ("controller is not initialized!");
    }
}


void SafetyController::setVelocitySetpoint(double velocity)
{
    performIsInit();
    if (in_velocity)
    {
        this->set_velocity = velocity;
    }
    else
    {
        throw BackhoeException ("tried to set velocity in position mode");
    }
}

double SafetyController::updateVelocity(void)
{
    performIsInit();
    if (!in_velocity)
    {
        set_velocity = params.gain*(set_position - position_estimate);
    }

    if (position_estimate <= (params.minimum_pos + params.limit_switch_safety_margin) && set_velocity < 0)
    {
        set_velocity = 0;
    } else if (position_estimate >= (params.maximum_pos - params.limit_switch_safety_margin) && set_velocity > 0)
    {
        set_velocity = 0;
    }

    if (set_velocity > fabs(params.max_abs_velocity))
    {
        set_velocity = params.max_abs_velocity;
    }
    else if (set_velocity < -fabs(params.max_abs_velocity))
    {
        set_velocity = -params.max_abs_velocity;
    }

    vesc->setLinearVelocity(set_velocity);
}

bool SafetyController::isAtSetpoint(void)
{
    return fabs(position_estimate - set_position) < fabs(params.setpoint_tolerance);
}

void SafetyController::updatePosition(double dt)
{
    performIsInit();
    switch (vesc->getLimitSwitchState())
    {
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

double SafetyController::getSafetyPosition()
{
    return this->params.safety_check_pos;
}

void SafetyController::stop()
{
   this->vesc->setLinearVelocity(0);
}

double SafetyController::getPosition()
{
    return this->position_estimate;
}

double SafetyController::getVelocity ()
{
    return set_velocity;
}

bool SafetyController::init()
{
    is_init = true;
    return is_init;
}

double SafetyController::getSetPosition()
{
    return set_position;
}
