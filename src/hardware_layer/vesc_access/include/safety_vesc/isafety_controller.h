#ifndef PROJECT_iSAFETY_VESC_H
#define PROJECT_iSAFETY_VESC_H

#include <string>

namespace safetycontroller
{
typedef struct joint_params
{
  double minimum_pos;
  double maximum_pos;
  double safety_check_pos;
  double gain;
  double setpoint_tolerance;
  double lower_limit_position;
  double upper_limit_position;
  double max_abs_velocity;
  double limit_switch_safety_margin;
  double max_abs_torque;
  std::string name;
} joint_params_t;

enum controlModeState
{
  none,
  position_control,
  velocity_control,
  torque_control
};
}

class iSafetyController
{
public:
  virtual void setPositionSetpoint(double pos) = 0;

  virtual void setVelocity(double velocity) = 0;

  virtual void setTorque(double torque) = 0;

  virtual bool getInitStatus() = 0;  // gets status to see if initialized (without taking action)

  virtual void checkIsInit() = 0;  // throws exception if not initialized

  virtual void update(double dt) = 0;

  virtual bool isAtSetpoint() = 0;

  virtual double getSafetyPosition() = 0;

  virtual double getPositionEstimate() = 0;

  virtual void stop() = 0;

  virtual bool init() = 0;

  virtual double getPositionSetpoint() = 0;

  virtual float getLinearVelocity() = 0;

  virtual float getTorque() = 0;

  virtual void abandonPositionSetpointAndSetTorqueWithoutStopping(double torque) = 0;

  virtual void updatePositionEstimate(double dt) = 0;

  virtual safetycontroller::controlModeState getControlMode() = 0;
  virtual float getCommandedTorque () =0;
  virtual float getCommandedVelocity() =0;
};

#endif  // PROJECT_SAFETY_VESC_H
