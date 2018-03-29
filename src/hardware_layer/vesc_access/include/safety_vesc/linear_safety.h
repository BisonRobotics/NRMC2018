#ifndef PROJECT_LINEAR_SAFETY_H
#define PROJECT_LINEAR_SAFETY_H
#include "safety_vesc.h"
#include "vesc_access/ivesc_access.h"

class LinearSafety : public SafetyVesc
{
public:
  LinearSafety (safetyvesc::joint_params_t params, iVescAccess *vesc, bool in_velocity);
  void init ();
protected:
  void updatePosition (double dt) override;
private:
};


#endif //PROJECT_LINEAR_SAFETY_H
