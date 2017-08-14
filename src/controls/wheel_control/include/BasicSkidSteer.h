#ifndef PROJECT_BASIC_SKID_STEER_H
#define PROJECT_BASIC_SKID_STEER_H

#include "Interface.h"
#include <pluginlib/class_list_macros.h>


namespace wheel_control {

    class BasicSkidSteer : public SkidSteerInterface
    {
        public:
            BasicSkidSteer();
            void load() override;
            void register_joints() override;
            void update() override;

    };

}

PLUGINLIB_EXPORT_CLASS(wheel_control::BasicSkidSteer, wheel_control::SkidSteerInterface)

#endif //PROJECT_BASIC_SKID_STEER_H
