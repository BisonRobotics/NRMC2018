//
// Created by nmach on 11/5/17.
//

#ifndef PROJECT_TEST_WHEELS_H
#define PROJECT_TEST_WHEELS_H

#include <wheel_control/wheels/wheels.h>

namespace wheel_control
{

class TestWheels : public Wheels
{
public:
  // Use same constructors as Wheels class
  TestWheels() : Wheels() {};
  TestWheels(double x, double y) : Wheels(x, y) {};

  // These methods were created for interacting with the hardware or simulation, don't need them for unit tests.
  void set_position(int index, double position) override {};
  void set_velocity(int index, double velocity) override {};
  void set_effort(int index, double effort) override {};
  double get_position(int index) override {};
  double get_velocity(int index) override {};
  double get_effort(int index) override {};
};
}

#endif //PROJECT_TEST_WHEELS_H
