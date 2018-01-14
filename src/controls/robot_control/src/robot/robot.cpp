#include <robot_control/robot/robot.h>

using namespace robot_control;

// TODO status bar messages should probably be thrown errors
Robot::Robot()
{
  desired_velocity[0] = 0.0;
  desired_velocity[1] = 0.0;
}

void Robot::spinOnce()
{
  wheel_controller->sendJointCommands();
}

void Robot::setVelocity(double linear, double angular)
{
  wheel_controller->setVelocity(linear, angular);
}
