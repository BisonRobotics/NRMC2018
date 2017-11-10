#ifndef PROJECT_WHEELCONTROLVELOCITYCONTROLLER_H
#define PROJECT_WHEELCONTROLVELOCITYCONTROLLER_H

#include <wheel_control/interface/interface.h>

namespace wheel_control
{
class VelocityInterface : public Interface
{
public:
  virtual void setVelocity(double x, double y) = 0;
};
}

#endif  // PROJECT_WHEELCONTROLVELOCITYCONTROLLER_H
