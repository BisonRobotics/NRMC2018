#ifndef PROJECT_LINEAR_SAFETY_H
#define PROJECT_LINEAR_SAFETY_H
#include "safety_controller.h"
#include "vesc_access/ivesc_access.h"

class LinearSafetyController : public SafetyController
{
public:
  LinearSafetyController(safetycontroller::joint_params_t params, iVescAccess *vesc);
  bool init() override;
  void updatePositionEstimate(double dt) override;

private:
  bool has_set_init_vel;
};

#endif  // PROJECT_LINEAR_SAFETY_H
