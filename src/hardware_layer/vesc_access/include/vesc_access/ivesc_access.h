#ifndef __VESC_ACCESS_INTERFACE_H_
#define __VESC_ACCESS_INTERFACE_H_
#include "stdint.h"
#include <string>
namespace nsVescAccess
{
enum limitSwitchState
{
  topOfMotion,
  bottomOfMotion,
  inTransit
};

typedef struct vesc_param_struct
{
  float max_velocity;
  float max_torque;
  float max_duty;
  float gear_ratio;
  float output_ratio;
  unsigned int pole_pairs;
  float torque_constant;
  char can_network[10];
  unsigned int can_id;
  std::string name;
} vesc_param_struct_t;
}

class iVescAccess
{
public:
  virtual void setLinearVelocity(float meters_per_second) = 0;
  virtual void setTorque(float current) = 0;
  virtual float getLinearVelocity(void) = 0;
  virtual float getTorque(void) = 0;
  virtual nsVescAccess::limitSwitchState getLimitSwitchState(void) = 0;
  virtual float getPotPosition(void) = 0;
  virtual void setDuty(float d) = 0;
};

#endif
