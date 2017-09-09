#include <robot_parameter_wrapper/robot_parameter_wrapper.h>
#include <math.h>
RobotParameterWrapper::RobotParameterWrapper(float transmission_ratio, float output_ratio, float velocity_limit,
                                             float torque_limit, iVesc *vesc)
{
  this->vesc = vesc;
  this->transmission_ratio = transmission_ratio;
  this->output_ratio = output_ratio;
  this->torque_limit = torque_limit;
  this->velocity_limit = velocity_limit;
}

RobotParameterWrapper::RobotParameterWrapper(uint8_t VESC_ID, float transmission_ratio, float output_ratio,
                                             float velocity_limit, float torque_limit, char *can_network)
{
  RobotParameterWrapper(transmission_ratio, output_ratio, velocity_limit, torque_limit, new Vesc(can_network, VESC_ID));
}

void RobotParameterWrapper::setOutputRatio(float output_ratio)
{
  this->output_ratio = output_ratio;
}

void RobotParameterWrapper::setTransmissionRatio(float transmission_ratio)
{
  this->transmission_ratio = transmission_ratio;
}

void RobotParameterWrapper::setLinearVelocity(float meters_per_second)
{
  if (fabs(meters_per_second) > this->velocity_limit)
  {
    if (meters_per_second >= 0)
    {
      meters_per_second = velocity_limit;
    }
    else
    {
      meters_per_second = velocity_limit * -1.0f;
    }
  }
  float rpm = meters_per_second / (this->output_ratio * this->transmission_ratio);
  this->vesc->setRpm(rpm);
}

void RobotParameterWrapper::setTorque(float newton_meters)
{
  if (fabs(newton_meters) > this->torque_limit)
  {
    if (newton_meters >= 0)
    {
      newton_meters = torque_limit;
    }
    else
    {
      newton_meters = -1.0f * torque_limit;
    }
  }
  this->vesc->setCurrent(newton_meters);
}

void RobotParameterWrapper::setTorqueLimit(float newton_meters)
{
  this->torque_limit = newton_meters;
}

void RobotParameterWrapper::setLinearVelocityLimit(float meters_per_second)
{
  this->velocity_limit = meters_per_second;
}

float RobotParameterWrapper::getLinearVelocityLimit(void)
{
  return velocity_limit;
}

float RobotParameterWrapper::getTransmissionRatio(void)
{
  return transmission_ratio;
}

float RobotParameterWrapper::getOutputRatio(void)
{
  return output_ratio;
}

float RobotParameterWrapper::getTorqueLimit(void)
{
  return torque_limit;
}
