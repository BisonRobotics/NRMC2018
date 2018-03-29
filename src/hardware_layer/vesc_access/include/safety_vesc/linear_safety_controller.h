#ifndef PROJECT_LINEAR_SAFETY_H
#define PROJECT_LINEAR_SAFETY_H
#include "safety_controller.h"
#include "vesc_access/ivesc_access.h"

class LinearSafetyController : public SafetyController
{
public:
  LinearSafetyController (safetycontroller::joint_params_t params, iVescAccess *vesc, bool in_velocity);
  void init ();
  void updatePosition (double dt) override;
};


#endif //PROJECT_LINEAR_SAFETY_H
