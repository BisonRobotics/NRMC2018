#ifndef PROJECT_BACKHOE_SAFETY_H
#define PROJECT_BACKHOE_SAFETY_H

#include "safety_vesc/safety_controller.h"

class BackhoeSafetyController : public SafetyController
{
public:
  BackhoeSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc);
  bool init() override;
  void updatePositionEstimate(double dt) override;
private:
  bool has_setpoint_been_set;
};

#endif  // PROJECT_BACKHOE_SAFETY_H
