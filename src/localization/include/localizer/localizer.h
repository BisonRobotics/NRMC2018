#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <vesc_access/vesc_access.h>
#include <can_sensors/imu_can_sensor.h>
#include <readable_sensors/readable_sensors.h>
#include <sys/time.h>

class Localizer
{
public:
  struct stateVector_s
  {
    float x_pos;   // in world coordinates
    float y_pos;   // in world coordinates
    float theta;  // robot rotation about its own center with reference to the
    // world's x axis and positive angles CCW

    float x_vel;   // derivitive of xPos
    float y_vel;   // derivitive of yPos
    float omega;  // derivitive of theta

    float x_accel;
    float y_accel;
    float alpha;
  } state_vector;

  enum class UpdateStatus
  {
    UPDATE_FAILED_SENSOR_ERROR,
    UPDATE_SUCCESS
  };

  Localizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
            iVescAccess *backLeftVesc);  // pass wheel linear vel sensors in as FL, FR, BR, BL
  UpdateStatus updateStateVector();
  UpdateStatus updateStateVector(float dt);

private:
  struct timeval previous_time;
  struct timeval current_time;
  int dtms;  // dt in ms
  iVescAccess *front_left_vesc;
  iVescAccess *front_right_vesc;
  iVescAccess *back_right_vesc;
  iVescAccess *back_left_vesc;
  int timediffms(struct timeval curr, struct timeval prev);
};

#endif
