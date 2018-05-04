#ifndef SIM_VESC_H
#define SIM_VESC_H

#include <vesc_access/ivesc_access.h>

class SimVesc : public iVescAccess
{
public:
  SimVesc(double Pgain, double Igain, double velo);
  SimVesc(double Pgain, double Igain, double velo_factor, double initialPos, double beginLimit, double endLimit,
          bool hitsGround = false, double groundPos = 0.0, double lff = 1.0);
  void setLinearVelocity(float meters_per_second) override;
  void setTorque(float current) override;  // note: not really implemented
  float getLinearVelocity(void) override;
  float getTorque(void) override;  // note: not really implemented
  void update(double dt);          // use P gain to adjust speed to set speed over time
  nsVescAccess::limitSwitchState getLimitSwitchState(void) override;
  void setLimitSwitchState(nsVescAccess::limitSwitchState state);
  float getPotPosition(void) override;
  void setPotPosition(float pos);
  bool ableToHitGround();
  void setDuty(float d) override;

private:
  float vel;
  float torque;
  float setVel;
  double errI;
  double vesc_Pgain;
  double vesc_Igain;
  double velocity_factor;
  nsVescAccess::limitSwitchState limitSwitch;
  double pot_pos;
  double beginLimit;
  double endLimit;
  double groundPos;
  double linearFudgeFuckter;
  bool hasLimits;
  bool hitsGround;
  bool onGround;
};

#endif
