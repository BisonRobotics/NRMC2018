#ifndef __VESC_ACCESS__
#define __VESC_ACCESS__
#include <vesc_control/vesc_socket_can.h>
#include <vesc_access/ivesc_access.h>

class VescAccess : public iVescAccess
{
public:
  //    VescAccess (unsigned int VESC_ID, double transmission_ratio, double output_ratio);
  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network, unsigned int pole_pairs, bool has_limits);
  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network, unsigned int pole_pairs, bool has_limits, std::string name);
  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network, unsigned int pole_pairs, bool has_limits, std::string name,
             float max_duty);

  VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, iVesc *vesc, unsigned int pole_pairs, bool has_limits);

  VescAccess(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, iVesc *vesc, unsigned int pole_pairs);

  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network, unsigned int pole_pairs);

  VescAccess(uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
             float torque_constant, char *can_network, unsigned int pole_pairs, std::string name);

  explicit VescAccess(nsVescAccess::vesc_param_struct_t param);

  VescAccess(nsVescAccess::vesc_param_struct_t param, bool has_limits);

  void setTorque(float newton_meters) override;
  void setLinearVelocity(float meters_per_second) override;
  float getLinearVelocityLimit(void);
  float getTransmissionRatio(void);
  float getOutputRatio(void);
  float getTorqueLimit(void);
  float getTorque(void) override;
  float getLinearVelocity(void) override;
  nsVescAccess::limitSwitchState getLimitSwitchState(void) override;
  float getPotPosition(void) override;
  void setDuty(float duty) override;
  float convertTorqueToCurrent(float torque);
  iVesc *vesc;
private:
  void setTorqueLimit(float newtown_meters);
  void setLinearVelocityLimit(float meters_per_second);
  void setTorqueConstant(float torque_ratio);
  void setPolePairs(unsigned int pole_pairs);
  void setMaxDuty(float max_duty);
  float transmission_ratio;
  float output_ratio;
  float torque_constant;
  float velocity_limit;
  float torque_limit;
  unsigned int pole_pairs;
  float rad_per_count;
  float rad_offset;
  float max_duty;

  void setTransmissionRatio(float transmission_ratio);
  void setOutputRatio(float output_ratio);

  float convertLinearVelocityToRpm(float velocity);
  float convertRpmToLinearVelocity(float rpm);
  float convertRpmToLinearVelocity(int rpm);
  float convertCurrentToTorque(float current);
  float convertErpmToRpm(float erpm);
  bool has_limits;
  std::string name;
  float convertRpmToErpm(float rpm);
  void initializeMembers(float transmission_ratio, float output_ratio, float velocity_limit, float torque_limit,
                         float torque_constant, unsigned int pole_pairs, bool has_limits, float max_duty = 1.0f);
};

#endif
