#ifndef PROJECT_SKIDSTEERINTERFACE_H
#define PROJECT_SKIDSTEERINTERFACE_H

#include <map>

namespace wheel_control
{
    struct Wheel
    {
        int    id        = -1;
        double vel       = 0.0;
        double x_pos     = 0.0;
        double y_pos     = 0.0;
        double radius    = 0.0;
    };

    typedef std::map<std::string, Wheel> Wheels;

    class SkidSteerInterface
    {
        public:
            virtual void load() = 0;
            virtual void register_joints() = 0;
            virtual void update() = 0;

            Wheels wheels = {{"right_front", Wheel()},
                             {"right_back",  Wheel()},
                             {"left_front",  Wheel()},
                             {"left_back",   Wheel()}};
    };
}

#endif //PROJECT_SKIDSTEERINTERFACE_H
