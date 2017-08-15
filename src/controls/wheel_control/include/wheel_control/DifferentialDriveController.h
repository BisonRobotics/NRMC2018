#ifndef PROJECT_DIFFERENTIALDRIVECONTROLLER_H
#define PROJECT_DIFFERENTIALDRIVECONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include "VelocityInterface.h"

namespace wheel_control {

    class DifferentialDriveController : public VelocityInterface
    {
        public:
            void set_velocity(double lin_vel, double ang_vel) override;
    };
}

PLUGINLIB_EXPORT_CLASS(wheel_control::DifferentialDriveController, wheel_control::VelocityInterface)

#endif //PROJECT_DIFFERENTIALDRIVECONTROLLER_H
