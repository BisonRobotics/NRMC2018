#include <localizer/localizer.h>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#define CONSTANT_TO_AVERAGE_TWO_NUMBERS 2.0f

Localizer::Localizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                     iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc)
{
  state_vector.x_pos = xi;
  state_vector.y_pos = yi;
  state_vector.theta = thi;

  state_vector.x_vel = 0;
  state_vector.y_vel = 0;
  state_vector.omega = 0;

  state_vector.x_accel = 0;
  state_vector.y_accel = 0;
  state_vector.alpha = 0;

  front_left_vesc = frontLeftVesc;
  front_right_vesc = frontRightVesc;
  back_right_vesc = backRightVesc;
  back_left_vesc = backLeftVesc;

  // gettimeofday(&current_time, NULL);  // initialize _prevmsgtime with something
  // current_time.tv_sec -= 1;           // make it in the past to avoid false positives

  axle_len = axleLen;
}

Localizer::UpdateStatus Localizer::updateStateVector(double dt, double theta_est)
{
    state_vector.theta = theta_est;
    updateStateVector(dt);
}

Localizer::UpdateStatus Localizer::updateStateVector(double dt)
{
  // get linear velocities of wheels
  double front_left_velocity = this->front_left_vesc->getLinearVelocity();
  double front_right_velocity = this->front_right_vesc->getLinearVelocity();
  double back_right_velocity = this->back_right_vesc->getLinearVelocity();
  double back_left_velocity = this->back_left_vesc->getLinearVelocity();

  double average_left_velocity = (front_left_velocity + back_left_velocity) / CONSTANT_TO_AVERAGE_TWO_NUMBERS;
  double average_right_velocity = (front_right_velocity + back_right_velocity) / CONSTANT_TO_AVERAGE_TWO_NUMBERS;

  double w = (average_right_velocity - average_left_velocity) / axle_len;
  double turn_radius;
  Eigen::Matrix2f rot;
  Eigen::Vector2f d_pos;
  Eigen::Vector2f rotation_on_y;
  double d_theta;

  if (std::abs(average_right_velocity - average_left_velocity) > 0.01f)  // no dividing by zero
  {
    turn_radius = (axle_len / 2.0f) * (average_right_velocity + average_left_velocity) /
                  (average_right_velocity - average_left_velocity);  // turn radius
    rot << cos(w * dt), -sin(w * dt), sin(w * dt), cos(w * dt);      // rotation matrix
    rotation_on_y << 0, -turn_radius;
    d_pos = rot * (rotation_on_y)-rotation_on_y;
    d_theta = w * dt;
  }
  else
  {
    d_pos << average_right_velocity *dt, 0;
    d_theta = 0;
  }

  // d_pos is in robot coordinates, must transform to world
  // rotate by worldrobot theta

  Eigen::Matrix2f wrot;
  wrot << cos(state_vector.theta), -sin(state_vector.theta), sin(state_vector.theta), cos(state_vector.theta);
  Eigen::Vector2f d_pos_world;
  d_pos_world = wrot * d_pos;

  // add it all up

  state_vector.x_pos += d_pos_world(0);
  state_vector.y_pos += d_pos_world(1);
  state_vector.theta += d_theta;
  if (state_vector.theta > M_PI)
    state_vector.theta -= 2.0 * M_PI;
  else if (state_vector.theta < -M_PI)
    state_vector.theta += 2.0 * M_PI;

  state_vector.x_vel = d_pos_world(0) / dt;
  state_vector.y_vel = d_pos_world(1) / dt;
  state_vector.omega = d_theta / dt;

  state_vector.y_accel = 0;  // no information about this, unless we did a derivative
  state_vector.x_accel = 0;
  state_vector.alpha = 0;
  return Localizer::UpdateStatus::UPDATE_SUCCESS;
}

LocalizerInterface::stateVector Localizer::getStateVector()
{
  return state_vector;
}
