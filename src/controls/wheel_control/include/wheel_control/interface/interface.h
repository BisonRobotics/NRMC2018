#ifndef PROJECT_SKIDSTEERINTERFACE_H
#define PROJECT_SKIDSTEERINTERFACE_H

#include <string>
#include <stdexcept>
#include <sensor_msgs/JointState.h>
#include <wheel_control/wheels/wheels.h>

namespace wheel_control
{
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

  void update(sensor_msgs::JointState *current_wheel_states)
  {
    this->wheels->current_state = *current_wheel_states;
  }

  Wheels *wheels;
};
}

#endif  // PROJECT_SKIDSTEERINTERFACE_H
