#ifndef BACKHOE_CONTROLLER_H
#define BACKHOE_CONTROLLER_H

#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include "safety_vesc/backhoe_safety_controller.h"
#include "safety_vesc/linear_safety_controller.h"

class BackhoeController
{
public:
  BackhoeController(iSafetyController *backhoeSafety, iSafetyController *linearSafety);

  // TODO, return status on update based on operation (see waypoint controller)
  void setShoulderSetpoint(double angle);  // in rad from horizontal
  void setWristSetpoint(double distance);  // in m
  void setShoulderTorque(double torque);   // in n*m
  void setWristVelocity(double velocity);  // in m/s
  void update(double dt);
  void abandonShoulderPositionSetpointAndSetTorqueWithoutStopping(double torque);
  void setShoulderVelocity(double velocity);
  void setWristTorque (double torque);
  bool hasHitGround();
  bool shoulderAtSetpoint();
  bool wristAtSetpoint();
  double getShoulderTorque();
  double getShoulderVelocity();
  double getPositionEstimate();
  bool getIsInit(void);
  void stopWrist (void);
  void stopShoulder (void);
private:
  void safetyCheck();
  iSafetyController *backhoe_safety;
  iSafetyController *linear_safety;
  static constexpr float ground_torque = 50.0f;
};

#endif
