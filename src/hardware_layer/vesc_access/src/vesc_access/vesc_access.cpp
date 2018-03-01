#include <vesc_access/vesc_access.h>
#include <math.h>

void VescAccess::initializeMembers(float transmission_ratio, float output_ratio, float velocity_limit,
                                   float torque_limit, float torque_constant, unsigned int pole_pairs, bool read_only)
{
  setTransmissionRatio(transmission_ratio);
  setOutputRatio(output_ratio);
  setTorqueLimit(torque_limit);
  setLinearVelocityLimit(velocity_limit);
  setTorqueConstant(torque_constant);
  setPolePairs(pole_pairs);
  this->read_only = read_only;
  this->minADC = 0;
  this->maxADC = 0x0FFF; // 12 bit ADC
  this->radians_per_turn = M_PI_2;
  this->rad_per_count = radians_per_turn / (1.0f * (maxADC - minADC));
  this->rad_offset = 0.0;
}

VescAccess::VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
                       float torque_constant, iVesc *vesc, unsigned int pole_pairs)
{
  this->vesc = vesc;
  initializeMembers(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant, pole_pairs, false);
}

VescAccess::VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit,
                       float torque_limit, float torque_constant, char *can_network, unsigned int pole_pairs)
{
  this->vesc = new Vesc(can_network, VESC_ID);
  initializeMembers(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant, pole_pairs, false);
}

VescAccess::VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
                       float torque_constant, iVesc *vesc, unsigned int pole_pairs, bool read_only)
{
  this->vesc = vesc;
  initializeMembers(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant, pole_pairs,
                    read_only);
}

VescAccess::VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit,
                       float torque_limit, float torque_constant, char *can_network, unsigned int pole_pairs,
                       bool read_only)
{

  this->vesc = new Vesc(can_network, VESC_ID);
  initializeMembers(transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_constant, pole_pairs, false);
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
  if (!read_only)
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
    float rpm = convertLinearVelocityToRpm(meters_per_second);
    // std::cout << "setting linear" << std::endl;
    if (this->vesc)
    {
      this->vesc->setRpm(convertRpmToErpm(rpm));
    }
    else
    {
      //  std::cout << "vesc not allocated" << std::endl;
    }
  }
}

void VescAccess::setTorque(float newton_meters)  // TODO utilize torque constant here
{
  if (!read_only)
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
  return (torque / (this->torque_constant * this->transmission_ratio));
}

float VescAccess::convertLinearVelocityToRpm(float velocity)
{
  return (velocity * this->transmission_ratio) / this->output_ratio;
}

float VescAccess::convertRpmToLinearVelocity(float rpm)
{
  return (rpm * (this->output_ratio / this->transmission_ratio));
}

float VescAccess::convertRpmToLinearVelocity(int rpm)
{
  float f_rpm = (float)rpm;
  return (convertRpmToLinearVelocity(f_rpm));
}

float VescAccess::convertCurrentToTorque(float current)
{
  return (current * this->torque_constant * this->transmission_ratio);
}

float VescAccess::getTorque(void)
{
  return (convertCurrentToTorque(vesc->getCurrent()));
}

float VescAccess::getLinearVelocity(void)
{
  return (convertRpmToLinearVelocity(convertErpmToRpm(vesc->getRpm())));
}

float VescAccess::convertErpmToRpm(float erpm)
{
  return (erpm / (1.0f * this->pole_pairs));
}

float VescAccess::convertRpmToErpm(float rpm)
{
  return (rpm * this->pole_pairs);
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

nsVescAccess::limitSwitchState VescAccess::getLimitSwitchState(void)
{
  nsVescAccess::limitSwitchState state;
  if (vesc->getForLimit())
  {
    state = nsVescAccess::limitSwitchState::topOfMotion;
  }
  else if (vesc->getRevLimit())
  {
    state = nsVescAccess::limitSwitchState::bottomOfMotion;
  }
  else
  {
    state = nsVescAccess::limitSwitchState::inTransit;
  }
  if (vesc->getRevLimit() && vesc->getForLimit())
  {
    throw std::runtime_error("both limit switches activated. Bad News");
  }
  return state;
}

float VescAccess::getPotPosition(void)
{
  return vesc->getADC() * rad_per_count - rad_offset;
}