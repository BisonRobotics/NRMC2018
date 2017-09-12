#include <vesc_access/vesc_access.h>
#include <math.h>

VescAccess::VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
                       float torque_constant, iVesc *vesc, unsigned int pole_pairs)
{
  this->vesc = vesc;
  setTransmissionRatio(transmission_ratio);
  setOutputRatio(output_ratio);
  setTorqueLimit(torque_limit);
  setLinearVelocityLimit(velocity_limit);
  setTorqueConstant(torque_constant);
  setPolePairs(pole_pairs);
  this->read_only = false;
}

VescAccess::VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit,
                       float torque_limit, float torque_constant, char *can_network, unsigned int pole_pairs)
{
  VescAccess(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant,
             new Vesc(can_network, VESC_ID), pole_pairs);
}

VescAccess::VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
                       float torque_constant, iVesc *vesc, unsigned int pole_pairs, bool read_only)
{
  this->vesc = vesc;
  setTransmissionRatio(transmission_ratio);
  setOutputRatio(output_ratio);
  setTorqueLimit(torque_limit);
  setLinearVelocityLimit(velocity_limit);
  setTorqueConstant(torque_constant);
  setPolePairs(pole_pairs);
  this->read_only = read_only;
}


VescAccess::VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit,
                       float torque_limit, float torque_constant, char *can_network, unsigned int pole_pairs, bool read_only)
{
  VescAccess(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant,
             new Vesc(can_network, VESC_ID), pole_pairs, read_only);
}

void VescAccess::setOutputRatio(float output_ratio)
{
  if (output_ratio == 0.0f)
  {
    output_ratio = 1.0f;
  }
  this->output_ratio = output_ratio;
}

void VescAccess::setTransmissionRatio(float transmission_ratio)
{
  if (transmission_ratio == 0.0f)
  {
    transmission_ratio = 1.0f;
  }
  this->transmission_ratio = transmission_ratio;
}

void VescAccess::setLinearVelocity(float meters_per_second)
{
  if (!read_only){
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
  float rpm = convertLinearVelocityToRpm(meters_per_second);
  this->vesc->setRpm(rpm);
}
}

void VescAccess::setTorque(float newton_meters)  // TODO utilize torque constant here
{
  if (!read_only){
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
  this->vesc->setCurrent(convertTorqueToCurrent(newton_meters));
  }
}

void VescAccess::setTorqueConstant(float torque_constant)
{
  if (torque_constant != 0.0f)
  {
    this->torque_constant = torque_constant;
  }
  else
  {
    this->torque_constant = 1.0f;
  }
}

void VescAccess::setTorqueLimit(float newton_meters)
{
  this->torque_limit = newton_meters;
}

void VescAccess::setLinearVelocityLimit(float meters_per_second)
{
  this->velocity_limit = meters_per_second;
}

float VescAccess::getLinearVelocityLimit(void)
{
  return velocity_limit;
}

float VescAccess::getTransmissionRatio(void)
{
  return transmission_ratio;
}

float VescAccess::getOutputRatio(void)
{
  return output_ratio;
}

float VescAccess::getTorqueLimit(void)
{
  return torque_limit;
}

float VescAccess::convertTorqueToCurrent(float torque)
{
  return (torque / torque_constant);
}

float VescAccess::convertLinearVelocityToRpm(float velocity)
{
  return (velocity / (this->output_ratio * this->transmission_ratio));
}

float VescAccess::convertRpmToLinearVelocity(float rpm)
{
  return (rpm * (this->output_ratio * this->transmission_ratio));
}

float VescAccess::convertRpmToLinearVelocity(int rpm)
{
  float f_rpm = (float)rpm;
  return (convertRpmToLinearVelocity(f_rpm));
}

float VescAccess::convertCurrentToTorque(float current)
{
  return (current * torque_constant);
}

float VescAccess::getTorque(void)
{
  return (convertCurrentToTorque(vesc->getCurrent()));
}

float VescAccess::getLinearVelocity(void)
{
  return (convertRpmToLinearVelocity(convertErpmToRpm(vesc->getRpm())));
}

float VescAccess::convertErpmToRpm(float rpm)
{
  return (rpm / (1.0f * this->pole_pairs));
}

void VescAccess::setPolePairs(unsigned int pole_pairs)
{
  if (pole_pairs == 0)
  {
    this->pole_pairs = 1;
  }
  else
  {
    this->pole_pairs = pole_pairs;
  }
}
