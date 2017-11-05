#include "wheel_control/differential_drive_controller/differential_drive_controller.h"

using namespace wheel_control;

void DifferentialDriveController::set_velocity(double lin_vel, double ang_vel)
{
  // Assumes all wheels are the same size and equidistant, which is valid for our robot
  double wheel_separation = 2.0 * this->wheels->front_right->y_pos;

  double rot_vel = ang_vel * wheel_separation;

  this->wheels->front_right->desired_state->velocity = lin_vel + rot_vel;
  this->wheels->back_right->desired_state->velocity = lin_vel + rot_vel;
  this->wheels->front_left->desired_state->velocity = lin_vel - rot_vel;
  this->wheels->back_left->desired_state->velocity = lin_vel - rot_vel;
}
