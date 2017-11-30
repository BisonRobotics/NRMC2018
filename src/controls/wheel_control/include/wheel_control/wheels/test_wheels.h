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
  TestWheels() : Wheels(){};
  TestWheels(double x, double y) : Wheels(x, y){};

  // These methods were created for interacting with the hardware or simulation, don't need them for unit tests.
  void setPosition(int index, double position) override{};
  void setVelocity(int index, double velocity) override{};
  void setEffort(int index, double effort) override{};
  double getPosition(int index) override{};
  double getVelocity(int index) override{};
  double getEffort(int index) override{};
};
}

#endif  // PROJECT_TEST_WHEELS_H
