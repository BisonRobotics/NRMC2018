#include <wheel_control/differential_drive_controller/differential_drive_controller.h>

using namespace wheel_control;

DifferentialDriveController::DifferentialDriveController()
{
  type = vel_t;
}

void DifferentialDriveController::setVelocity(double lin_vel, double ang_vel)
{
  // Assumes all wheels are the same size and equidistant, which is valid for our robot
  double wheel_separation = 2.0 * this->wheels->y_pos[FLI];

  double rot_vel = ang_vel * wheel_separation;

  this->wheels->desired_state.velocity[FLI] = lin_vel - rot_vel;
  this->wheels->desired_state.velocity[FRI] = lin_vel + rot_vel;
  this->wheels->desired_state.velocity[BLI] = lin_vel - rot_vel;
  this->wheels->desired_state.velocity[BRI] = lin_vel + rot_vel;
}
