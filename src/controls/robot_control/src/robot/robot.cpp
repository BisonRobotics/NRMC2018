#include <robot_control/robot/robot.h>

using namespace robot_control;

//TODO status bar messages should probably be thrown errors
Robot::Robot()
{
  desired_velocity[0] = 0.0;
  desired_velocity[1] = 0.0;
}

void Robot::spin_once()
{
  wheel_controller->send_joint_commands();
}


void Robot::set_velocity(double linear, double angular)
{
  wheel_controller->set_velocity(linear, angular);
}

