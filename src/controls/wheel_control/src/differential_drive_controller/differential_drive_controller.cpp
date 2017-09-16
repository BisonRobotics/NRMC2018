#include "wheel_control/differential_drive_controller/differential_drive_controller.h"

using namespace wheel_control;

void DifferentialDriveController::set_velocity(double lin_vel, double ang_vel)
{
  // Assumes all wheels are the same size and equidistant, which is valid for our robot
  double wheel_separation = 2.0 * this->wheels->right_front->y_pos;

  double rot_vel = ang_vel * wheel_separation;

  this->wheels->right_front->desired_state->velocity = lin_vel + rot_vel;
  this->wheels->right_back->desired_state->velocity = lin_vel + rot_vel;
  this->wheels->left_front->desired_state->velocity = lin_vel - rot_vel;
  this->wheels->left_back->desired_state->velocity = lin_vel - rot_vel;
}
