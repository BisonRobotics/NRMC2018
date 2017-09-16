#ifndef PROJECT_SKIDSTEERINTERFACE_H
#define PROJECT_SKIDSTEERINTERFACE_H

#include <map>
#include <string>
#include "wheel_control/Wheels.h"
#include <stdexcept>

namespace wheel_control
{
typedef std::map<std::string, JointState> JointStates;
class Interface
{
public:
  void load(Wheels *wheels)
  {
    this->wheels = wheels;
  }

  void unload()
  {
    delete this->wheels;
  }

  void update(JointStates current_wheel_states)
  {
    bool flag;
    for (auto wheel : current_wheel_states)
    {
      flag = true;
      for (auto local : this->wheels->get())
      {
        if (wheel.first == local->name)
        {
          *(local->current_state) = wheel.second;
          flag = false;
        }
      }
      if (flag)
      {
        std::string error_msg = wheel.first + " wheel not found";
        throw std::invalid_argument(error_msg);
      }
    }
  }

  Wheels *wheels;
};
}

#endif  // PROJECT_SKIDSTEERINTERFACE_H
