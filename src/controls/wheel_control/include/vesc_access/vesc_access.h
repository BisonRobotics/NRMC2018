#ifndef __ROBOT_PARAMETER_WRAPPER__
#define __ROBOT_PARAMETER_WRAPPER__
#include <vesc_control/vesc_socket_can.h>

class VescAccess
{
public:
  //    VescAccess (unsigned int VESC_ID, double transmission_ratio, double output_ratio);
  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network);
  VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, iVesc *vesc);
  void setTorque(float newton_meters);
  void setLinearVelocity(float meters_per_second);
  float getLinearVelocityLimit(void);
  float getTransmissionRatio(void);
  float getOutputRatio(void);
  float getTorqueLimit(void);

private:
  void setTorqueLimit(float newtown_meters);
  void setLinearVelocityLimit(float meters_per_second);
  void setTorqueConstant(float torque_ratio);
  float transmission_ratio;
  float output_ratio;
  float torque_constant;
  float velocity_limit;
  float torque_limit;
  iVesc *vesc;
  void setTransmissionRatio(float transmission_ratio);
  void setOutputRatio(float output_ratio);
};

#endif
