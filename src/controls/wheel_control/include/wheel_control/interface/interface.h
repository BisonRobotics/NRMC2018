#ifndef PROJECT_WHEELCONTROLINTERFACE_H
#define PROJECT_WHEELCONTROLINTERFACE_H

#include <string>
#include <stdexcept>
#include <sensor_msgs/JointState.h>
#include <wheel_control/wheels/wheels.h>

namespace wheel_control
{
const int pos_t = 0;
const int vel_t = 1;
const int eff_t = 2;

class Interface
{
public:
  int type = -1;
  Wheels *wheels;

  virtual void updateDesiredState() = 0;

  void load(Wheels *wheels)
  {
    this->wheels = wheels;
  }

  void unload()
  {
    delete this->wheels;
  }

  void sendJointCommands()
  {
    updateDesiredState();
    if (type == pos_t)
    {
      for (int i = 0; i < 4; i++)
      {
        wheels->setPosition(i, wheels->desired_state.position[i]);
      }
    }
    if (type == vel_t)
    {
      for (int i = 0; i < 4; i++)
      {
        wheels->setVelocity(i, wheels->desired_state.velocity[i]);
      }
    }
    if (type == eff_t)
    {
      for (int i = 0; i < 4; i++)
      {
        wheels->setEffort(i, wheels->desired_state.effort[i]);
      }
    }
    else
    {
      // TODO throw error
    }
  }
};
}

#endif  // PROJECT_WHEELCONTROLINTERFACE_H
