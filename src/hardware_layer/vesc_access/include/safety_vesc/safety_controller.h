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

class SafetyController : public iSafetyController
{
public:
  SafetyController(iVescAccess *vesc, safetycontroller::joint_params_t params);
  void setPositionSetpoint(double position) override;
  void setVelocity(double velocity) override;
  void setTorque(double torque) override;
  bool getInitStatus() override;
  void update(double dt) override;
  bool isAtSetpoint(void) override;
  double getSafetyPosition() override;
  double getPositionEstimate() override;
  void stop() override;
  bool init() override;
  double getPositionSetpoint() override;
  float getLinearVelocity() override;
  float getTorque() override;
  void abandonPositionSetpointAndSetTorqueWithoutStopping(double torque) override;
  safetycontroller::controlModeState getControlMode() override;
  void updatePositionEstimate(double dt) override;  // you must call this method in your implementation which overrides
  float getCommandedTorque () {return set_torque;};
  float getCommandedVelocity() {return set_velocity;};
protected:
  void checkIsInit() override;
  safetycontroller::joint_params_t params;
  iVescAccess *vesc;
  bool is_init;
  double position_estimate;
  double torque_magnitude;
private:
  safetycontroller::controlModeState control_mode;
  double set_position;
  double set_torque;
  double set_velocity;
  double symmetricClamp(double number, double bound);
};

#endif  // PROJECT_SAFETY_VESC_H
