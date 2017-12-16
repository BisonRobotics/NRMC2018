#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <vesc_access/vesc_access.h>
#include <sys/time.h>
#include <localizer/localizer_interface.h>

class Localizer : public LocalizerInterface::LocalizerInterface_c
{
public:
  LocalizerInterface::stateVector getStateVector();
  UpdateStatus updateStateVector(float dt);

  Localizer(float axelLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
            iVescAccess *backLeftVesc);  // pass wheel linear vel sensors in as FL, FR, BR, BL

protected:
  //struct timeval previous_time;
  //struct timeval current_time;
  //int dtms;  // dt in ms
  iVescAccess *front_left_vesc;
  iVescAccess *front_right_vesc;
  iVescAccess *back_right_vesc;
  iVescAccess *back_left_vesc;
  //int timediffms(struct timeval curr, struct timeval prev);
  LocalizerInterface::stateVector state_vector;
  float axle_len;
};

#endif
