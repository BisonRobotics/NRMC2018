#ifndef PROJECT_ROBOT_H
#define PROJECT_ROBOT_H

#include <string>
#include <vector>
#include <cmath>

#include <sensor_msgs/JointState.h>
#include <tf/tf.h>

#include <wheel_control/wheels/wheels.h>
#include <wheel_control/velocity_interface/velocity_interface.h>

namespace robot_control
{
class Robot
{
public:
  Robot();
  void spinOnce();
  void setVelocity(double linear, double angular);
  virtual void initialize(wheel_control::Wheels *wheels, wheel_control::VelocityInterface *controller) = 0;
  virtual void getPosition(tf::Transform *position) = 0;
private:
  double desired_velocity[2];
  wheel_control::Wheels *wheels;
  wheel_control::VelocityInterface *wheel_controller;
};
}

#endif  // PROJECT_ROBOT_H
