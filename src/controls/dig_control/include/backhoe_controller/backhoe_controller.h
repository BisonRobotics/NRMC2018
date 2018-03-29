#ifndef BACKHOE_CONTROLLER_H
#define BACKHOE_CONTROLLER_H

#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include "safety_vesc/backhoe_safety.h"
#include "safety_vesc/linear_safety.h"


class BackhoeController
{
public:
  BackhoeController (iSafetyVesc *backhoeSafety, iSafetyVesc *linearSafety);

  // TODO, return status on update based on operation (see waypoint controller)
  void setShoulderSetpoint(double angle);     // in rad from horizontal
  void setWristSetpoint(double distance);     // in m
  void setShoulderVelocity(double velocity);  // in rad/s
  void setWristVelocity(double velocity);     // in m/s
  void init();
  void update(double dt);
  void tareBucket(void);
  void tareBackhoe(void);
  double getWeightInBucket(void);
  double getWeightInBackhoe(void);
  bool shoulderAtSetpoint();
  bool wristAtSetpoint();
  bool getIsInit (void);
private:
  void safetyCheck();
  iSafetyVesc *backhoe_safety;
  iSafetyVesc *linear_safety;
};

#endif
