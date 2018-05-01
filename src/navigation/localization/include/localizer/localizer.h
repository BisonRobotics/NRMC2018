#ifndef __LOCALIZER_H__
#define __LOCALIZER_H__

#include <vesc_access/vesc_access.h>
#include <sensor_access/imu_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>
#include <sys/time.h>
#include <localizer/localizer_interface.h>

class Localizer : public LocalizerInterface::LocalizerInterface_c
{
public:
  LocalizerInterface::stateVector getStateVector();
  UpdateStatus updateStateVector(double dt);
  UpdateStatus updateStateVector(double dt, double theta_est);

  Localizer(double axelLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc,
            iVescAccess *backRightVesc,
            iVescAccess *backLeftVesc);  // pass wheel linear vel sensors in as FL, FR, BR, BL

protected:
  // struct timeval previous_time;
  // struct timeval current_time;
  // int dtms;  // dt in ms
  iVescAccess *front_left_vesc;
  iVescAccess *front_right_vesc;
  iVescAccess *back_right_vesc;
  iVescAccess *back_left_vesc;
  // int timediffms(struct timeval curr, struct timeval prev);
  LocalizerInterface::stateVector state_vector;
  double axle_len;
};

#endif
