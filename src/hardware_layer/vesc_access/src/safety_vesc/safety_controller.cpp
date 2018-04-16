#include "safety_vesc/safety_controller.h"
#include <math.h>
#include <sstream>
#include <ros/ros.h>

SafetyController::SafetyController(iVescAccess *vesc, safetycontroller::joint_params_t params)
{
    this->params = params;
    this->vesc = vesc;
    this->is_init = false;
    this->in_position_control = false;
    this->in_open_loop_velocity_control = false;
    this->in_open_loop_torque_control = false;
    this->set_velocity = 0;
    this->set_torque = 0;
    this->stopped = true;
    this->position_estimate = 0.0;
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
    in_position_control = true;
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
    if (!in_position_control)
    {
      set_velocity = symmetricClamp(velocity,params.max_abs_velocity);
      in_open_loop_velocity_control = true;
      in_open_loop_torque_control = false;
    }
    else
    {
        throw BackhoeException ("tried to set velocity in position mode");
    }
}

void SafetyController::setTorque(double torque)
{
    checkIsInit();
    if (!in_position_control)
    {
        set_torque = symmetricClamp(torque, params.max_abs_torque);
        in_open_loop_velocity_control = false;
        in_open_loop_torque_control = true;
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
    if (in_position_control)
    {
      if (isAtSetpoint())
      {
        ROS_INFO("stopped because at setpoint");
        stop();
        in_position_control = false;
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
       if (in_position_control || in_open_loop_velocity_control) 
       {
           vesc->setLinearVelocity(set_velocity);
           ROS_INFO("setting velocity to %.4f", set_velocity);
       }
       else if (in_open_loop_torque_control)
       {
           vesc->setTorque(set_torque);
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
   in_position_control = false;
   in_open_loop_velocity_control = false;
   in_open_loop_torque_control = false;
}

void SafetyController::abandonPositionSetpointAndSetTorqueWithoutStopping(double torque)
{
   in_position_control = false;
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