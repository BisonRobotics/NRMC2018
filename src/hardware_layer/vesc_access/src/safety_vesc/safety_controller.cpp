#include "safety_vesc/safety_controller.h"
#include <math.h>
#include <sstream>
#include <ros/ros.h>

SafetyController::SafetyController(iVescAccess *vesc, safetycontroller::joint_params_t params)
{
    this->params = params;
    this->vesc = vesc;
    this->is_init = false;
    control_mode = safetycontroller::none;
    this->set_velocity = 0;
    this->set_torque = 0;
    this->stopped = true;
    this->position_estimate = (params.upper_limit_position + params.lower_limit_position)/2.0;
}

void SafetyController::setPositionSetpoint(double position)
{
    checkIsInit();

    if  (position > params.maximum_pos || position < params.minimum_pos)
    {
      std::stringstream ss;
      ss << "Out of bounds at : " << position;
      this->set_position = this->position_estimate;
      this->stop();
      throw BackhoeException (ss.str ());
    }
    this->set_position = position;
    control_mode = safetycontroller::position_control;
}

void SafetyController::checkIsInit ()
{
    if (!is_init){
        this->stop();
        throw BackhoeException ("controller is not initialized!");
    }
}

double SafetyController::symmetricClamp (double number, double bound)
{
    return std::max(-bound,std::min(number,bound));
}

void SafetyController::setVelocity(double velocity)
{
    checkIsInit();
    if (control_mode != safetycontroller::position_control)
    {
      set_velocity = symmetricClamp(velocity,params.max_abs_velocity);
      control_mode = safetycontroller::velocity_control;
    }
    else
    {
        throw BackhoeException ("tried to set velocity in position mode");
    }
}

void SafetyController::setTorque(double torque)
{
    checkIsInit();
    if (control_mode != safetycontroller::position_control)
    {
        set_torque = symmetricClamp(torque, params.max_abs_torque);
        control_mode = safetycontroller::torque_control;
    }
    else
    {
        throw BackhoeException ("tried to set torque in position mode");
    }
}


void SafetyController::update(double dt)
{
    stopped = false;
    checkIsInit();
    ROS_INFO("doing safety controller update");
    if (control_mode == safetycontroller::position_control)
    {
      if (isAtSetpoint())
      {
        ROS_INFO("stopped because at setpoint");
        stop();
        control_mode = safetycontroller::none;
      }
      else
      {
        set_velocity = symmetricClamp(params.gain*(set_position - position_estimate), params.max_abs_velocity);
      }
    }
    //this happens in any mode 
    updatePositionEstimate(dt);
    if (position_estimate <= (params.minimum_pos + params.limit_switch_safety_margin) && 
        (set_velocity < 0 || set_torque < 0))
    {
      ROS_INFO("stopped because guessing too close to min limit switch");
      stop();
    } 
    else if (position_estimate >= (params.maximum_pos - params.limit_switch_safety_margin) && 
              (set_velocity > 0 || set_torque >0))
    {
      ROS_INFO("stopped because guessing too close to max limit switch");
      stop();
    }

    if (!stopped)
    {
        switch (control_mode){
            case safetycontroller::position_control:
            case safetycontroller::velocity_control:
                vesc->setLinearVelocity(set_velocity);
                break;
            case safetycontroller::torque_control:
                vesc->setTorque(set_torque);
                break;
        }
    }
}

bool SafetyController::isAtSetpoint(void)
{
    return fabs(position_estimate - set_position) < fabs(params.setpoint_tolerance);
}

double SafetyController::getSafetyPosition()
{
    return this->params.safety_check_pos;
}

void SafetyController::stop()
{
   this->vesc->setLinearVelocity(0);
   stopped=true;
   control_mode = safetycontroller::none;
}

void SafetyController::abandonPositionSetpointAndSetTorqueWithoutStopping(double torque)
{
   control_mode = safetycontroller::torque_control;
   setTorque(torque);
}

double SafetyController::getPositionEstimate()
{
    return this->position_estimate;
}

bool SafetyController::init()
{
    is_init = true;
    return is_init;
}

bool SafetyController::getInitStatus()
{
    return is_init;
}

double SafetyController::getPositionSetpoint()
{
    return set_position;
}

float SafetyController::getLinearVelocity()
{
    return vesc->getLinearVelocity();
}

float SafetyController::getTorque()
{
    return vesc->getTorque();
}

void SafetyController::updatePositionEstimate(double dt)
{
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

safetycontroller::controlModeState SafetyController::getControlMode()
{
    return control_mode;
}