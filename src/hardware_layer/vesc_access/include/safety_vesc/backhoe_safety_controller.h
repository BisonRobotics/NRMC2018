#ifndef PROJECT_BACKHOE_SAFETY_H
#define PROJECT_BACKHOE_SAFETY_H

#include "safety_vesc/safety_controller.h"

class BackhoeSafetyController : public SafetyController
{
public:
    BackhoeSafetyController (safetycontroller::joint_params_t params, iVescAccess *vesc, bool in_velocity);
    void init () override;
    void updatePosition (double dt) override;
};

#endif //PROJECT_BACKHOE_SAFETY_H
