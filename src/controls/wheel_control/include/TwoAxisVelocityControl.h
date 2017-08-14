#ifndef PROJECT_TWOAXISVELOCITYCONTROL_H
#define PROJECT_TWOAXISVELOCITYCONTROL_H

#include "VelocityInterface.h"

namespace wheel_control {

    class TwoAxisVelocityController : public VelocityInterface
    {
        public:
            void update();
            void set_velocity(double lin_vel, double ang_vel);
    };



}

#endif //PROJECT_TWOAXISVELOCITYCONTROL_H
