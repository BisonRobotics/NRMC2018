#ifndef PROJECT_BASIC_SKID_STEER_H
#define PROJECT_BASIC_SKID_STEER_H

#include "SkidSteerInterface.h"

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


#endif //PROJECT_BASIC_SKID_STEER_H
