#ifndef PROJECT_BACKHOE_SAFETY_H
#define PROJECT_BACKHOE_SAFETY_H

#include "safety_vesc/safety_vesc.h"

class BackhoeSafety : public SafetyVesc
{
public:
    BackhoeSafety (safetyvesc::joint_params_t params, iVescAccess *vesc, bool in_velocity);
    void init () override;
    void updatePosition (double dt) override;
};

#endif //PROJECT_BACKHOE_SAFETY_H
