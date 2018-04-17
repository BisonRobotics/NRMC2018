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
    this->position_estimate = (params.lower_limit_position+params.upper_limit_position)/2.0;
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
    ROS_INFO("%s doing safety controller update", params.name.c_str());
    if (control_mode == safetycontroller::position_control)
    {
      if (isAtSetpoint())
      {
        ROS_INFO("%s stopped because at setpoint %4f with posestimate %.4f", params.name.c_str(), set_position, position_estimate);
        stop();
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
      ROS_INFO("%s stopped because guessing too close to min limit switch at %.4f", params.name.c_str(), position_estimate);
      stop();
    } 
    else if (position_estimate >= (params.maximum_pos - params.limit_switch_safety_margin) && 
              (set_velocity > 0 || set_torque >0))
    {
      ROS_INFO("%s stopped because guessing too close to max limit switch at %.4f", params.name.c_str(), position_estimate);
      stop();
    }

    if (!stopped)
    {
        switch (control_mode){
            case safetycontroller::position_control:
            case safetycontroller::velocity_control:
                vesc->setLinearVelocity(set_velocity);
                ROS_INFO ("%s set velocity: %.4f", params.name.c_str(), set_velocity);
                break;
            case safetycontroller::torque_control:
                vesc->setTorque(set_torque);
                ROS_INFO ("%s set velocity: %.4f", params.name.c_str(), set_velocity);
                break;
        }
    }
}

bool SafetyController::isAtSetpoint(void)
{
    return fabs(position_estimate - set_position) < fabs(params.setpoint_tolerance) || control_mode==safetycontroller::controlModeState::none;
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

void SafetyController::checkPositionEstimateAgainstLimitSwitchesAndResetItIfNeeded()
{
    switch (vesc->getLimitSwitchState())
    {
        case nsVescAccess::limitSwitchState::bottomOfMotion:
            ROS_INFO ("%s at lower limit", params.name.c_str());
            this->position_estimate = params.lower_limit_position;
            break;
        case nsVescAccess::limitSwitchState::topOfMotion:
           this->position_estimate = params.upper_limit_position;
            ROS_INFO ("%s at upper limit", params.name.c_str());
           break;
       case nsVescAccess::limitSwitchState::inTransit:
           break;
    }
}

safetycontroller::controlModeState SafetyController::getControlMode()
{
    return control_mode;
}

void SafetyController::updatePositionEstimate(double dt) //you must call this method in your implementation which overrrides this one
{
    checkPositionEstimateAgainstLimitSwitchesAndResetItIfNeeded();
}