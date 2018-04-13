#ifndef PROJECT_SAFETY_VESC_H
#define PROJECT_SAFETY_VESC_H

#include <stdexcept>
#include "safety_vesc/isafety_controller.h"
#include "vesc_access/ivesc_access.h"


class BackhoeException : public std::runtime_error
{
public:
  explicit BackhoeException(std::string msg) : std::runtime_error(msg)
  {
  }
};



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
  bool init() override;
  virtual double getSetPosition () override;
  float getLinearVelocity () {return vesc->getLinearVelocity();}
  float getTorque () override {return vesc->getTorque();}
protected:
  safetycontroller::joint_params_t params;
  iVescAccess *vesc;
  bool is_init;
  double position_estimate;
private:
  double set_position;
  double set_velocity;
  bool in_position_control;
  void performIsInit();
};

#endif //PROJECT_SAFETY_VESC_H
