#include "DifferentialDriveController.h"

using namespace wheel_control;

void DifferentialDriveController::set_velocity(double lin_vel, double ang_vel) {
    // Assumes all wheels are the same size and equidistant, which is valid for our robot
    double radius = this->wheels->right_front->radius;
    double wheel_separation = 2.0 * this->wheels->right_front->y_pos;

    double lin_vel_rad = lin_vel / radius;
    double rot_vel_rad = ang_vel * wheel_separation / radius;

    this->wheels->right_front->desired_state->velocity = lin_vel_rad + rot_vel_rad;
    this->wheels->right_back ->desired_state->velocity = lin_vel_rad + rot_vel_rad;
    this->wheels->left_front ->desired_state->velocity = lin_vel_rad - rot_vel_rad;
    this->wheels->left_back  ->desired_state->velocity = lin_vel_rad - rot_vel_rad;
}
