#ifndef PROJECT_SKIDSTEERINTERFACE_H
#define PROJECT_SKIDSTEERINTERFACE_H

#include <map>
#include <string>
#include "Wheels.h"

namespace wheel_control
{
    typedef std::map<std::string, Wheel> Wheels;

    class Interface
    {
        public:
            void load(Wheels *wheels) {
                this->wheels = wheels;
            }

            void unload()
            {
                delete this->wheels;
            }

            virtual void update() = 0;

            void register_joints(int right_front, int right_back, int left_front, int left_back) {

            }

            Wheels wheels;
    };
}

#endif //PROJECT_SKIDSTEERINTERFACE_H
