#ifndef __VESC_ACCESS_INTERFACE_H_
#define __VESC_ACCESS_INTERFACE_H_

namespace nsVescAccess
{
enum limitSwitchState
{
  topOfMotion,
  bottomOfMotion,
  inTransit
};
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
};


#endif
