#include <localizer/localizer.h>
#include <Eigen/Dense>
#include <cmath>
#define CONSTANT_TO_AVERAGE_TWO_NUMBERS 2.0f

Localizer::Localizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                     iVescAccess *backLeftVesc)
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

  //gettimeofday(&current_time, NULL);  // initialize _prevmsgtime with something
  //current_time.tv_sec -= 1;           // make it in the past to avoid false positives

  axle_len = axleLen;
}
/*
Localizer::UpdateStatus Localizer::updateStateVector()
{
  if (false)
    return Localizer::UpdateStatus::UPDATE_FAILED_SENSOR_ERROR;
  else
  {
    // take current time and subtract it from previous time to get dt
    previous_time = current_time;
    gettimeofday(&current_time, NULL);
    dtms = timediffms(current_time, previous_time);
    float dt = dtms / 1000.0f;
    updateStateVector(dt);
    return Localizer::UpdateStatus::UPDATE_SUCCESS;
  }
}
*/
Localizer::UpdateStatus Localizer::updateStateVector(float dt)
{
  // get linear velocities of wheels
  float front_left_velocity = this->front_left_vesc->getLinearVelocity();
  float front_right_velocity = this->front_right_vesc->getLinearVelocity();
  float back_right_velocity = this->back_right_vesc->getLinearVelocity();
  float back_left_velocity = this->back_left_vesc->getLinearVelocity();

  float average_left_velocity = (front_left_velocity + back_left_velocity) / CONSTANT_TO_AVERAGE_TWO_NUMBERS;
  float average_right_velocity = (front_right_velocity + back_right_velocity) / CONSTANT_TO_AVERAGE_TWO_NUMBERS;

  float w = (average_right_velocity - average_left_velocity) / axle_len;
  float turn_radius;
  Eigen::Matrix2f rot;
  Eigen::Vector2f d_pos;
  Eigen::Vector2f rotation_on_y;
  float d_theta;

  if (std::abs(average_right_velocity - average_left_velocity) > 0.01f)  // no dividing by zero
  {
    turn_radius = axle_len / 2 * (average_right_velocity + average_left_velocity) /
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

  state_vector.x_vel = d_pos_world(0) / dt;
  state_vector.y_vel = d_pos_world(1) / dt;
  state_vector.omega = d_theta / dt;
  return Localizer::UpdateStatus::UPDATE_SUCCESS;
}
/*
int Localizer::timediffms(struct timeval curr, struct timeval prev)
{
  // stolen from candump.c, pretty gross
  struct timeval diff;
  diff.tv_sec = curr.tv_sec - prev.tv_sec;
  diff.tv_usec = curr.tv_usec - prev.tv_usec;
  if (diff.tv_usec < 0)
    diff.tv_sec--, diff.tv_usec += 1000000;
  if (diff.tv_sec < 0)
    diff.tv_sec = diff.tv_usec = 0;
  return diff.tv_sec * 1000 + diff.tv_usec / 1000;
}
*/
LocalizerInterface::stateVector Localizer::getStateVector()
{
  return state_vector;
}

