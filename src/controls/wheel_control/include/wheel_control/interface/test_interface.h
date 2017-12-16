#ifndef PROJECT_WHEELCONTROLTESTINTERFACE_H
#define PROJECT_WHEELCONTROLTESTINTERFACE_H

#include <wheel_control/interface/interface.h>

namespace wheel_control
{
class TestInterface : public Interface
{
public:
  void updateDesiredState() override{};
};
}

#endif  // WHEELCONTROLTESTINTERFACE_H
