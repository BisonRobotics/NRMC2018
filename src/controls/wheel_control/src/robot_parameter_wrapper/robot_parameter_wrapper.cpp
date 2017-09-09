#include <robot_parameter_wrapper/robot_parameter_wrapper.h>
#include <math.h>


RobotParameterWrapper::RobotParameterWrapper(float transmission_ratio, float output_ratio, float velocity_limit,
                                             float torque_limit, float torque_constant, iVesc *vesc)
{
  this->vesc = vesc;
  setTransmissionRatio(transmission_ratio);
  setOutputRatio(output_ratio);	
  setTorqueLimit(torque_limit);
  setLinearVelocityLimit(velocity_limit);
  setTorqueConstant(torque_constant);
}

RobotParameterWrapper::RobotParameterWrapper(uint8_t VESC_ID, float transmission_ratio, float output_ratio,
                                             float velocity_limit, float torque_limit, float torque_constant, char *can_network)
{
  RobotParameterWrapper(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant,  new Vesc(can_network, VESC_ID));
}

void RobotParameterWrapper::setOutputRatio(float output_ratio)
{
  if (output_ratio == 0.0f){
	output_ratio = 1.0f;
	}
  this->output_ratio = output_ratio;
}

void RobotParameterWrapper::setTransmissionRatio(float transmission_ratio)
{
  if (transmission_ratio == 0.0f){
	transmission_ratio = 1.0f;
}
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

void RobotParameterWrapper::setTorque(float newton_meters) //TODO utilize torque constant here
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
  this->vesc->setCurrent(newton_meters/torque_constant);
}

void RobotParameterWrapper::setTorqueConstant (float torque_constant){
 if (torque_constant != 0.0f){
	this->torque_constant = torque_constant;
 }else {
	this->torque_constant = 1.0f; 
}
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
