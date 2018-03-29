#ifndef PROJECT_SAFETY_VESC_H
#define PROJECT_SAFETY_VESC_H

#include "safety_vesc/isafety_controller.h"
#include "vesc_access/ivesc_access.h"

class SafetyController : public iSafetyController {
public:
  SafetyController (iVescAccess *vesc, safetycontroller::joint_params_t params, bool in_velocity);
  void setPositionSetpoint (double position) override;
  void setVelocitySetpoint (double velocity) override;
  bool isInit () override {return is_init;}
  double updateVelocity (void) override;
  bool isAtSetpoint (void) override;
  double getSafetyPosition () override;
  double getVelocity () override;
  double getPosition () override;
  void stop () override;
  virtual void init () override;
  virtual double getSetPosition () override;
  void updatePosition (double dt) override;
protected:
  safetycontroller::joint_params_t params;
  iVescAccess *vesc;
  bool is_init;
  double position_estimate;
private:
  double set_position;
  double set_velocity;
  bool in_velocity;
};

#endif //PROJECT_SAFETY_VESC_H
