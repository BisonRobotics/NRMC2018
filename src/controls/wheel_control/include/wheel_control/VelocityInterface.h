//
// Created by root on 8/13/17.
//

#ifndef PROJECT_SKIDSTEERVELOCITYCONTROLLER_H
#define PROJECT_SKIDSTEERVELOCITYCONTROLLER_H

#include "Interface.h"

namespace wheel_control
{
class VelocityInterface : public Interface
{
public:
  virtual void set_velocity(double x, double y) = 0;
};
}

#endif  // PROJECT_SKIDSTEERVELOCITYCONTROLLER_H
