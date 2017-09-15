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
    float xPos;   // in world coordinates
    float yPos;   // in world coordinates
    float theta;  // robot rotation about its own center with reference to the
    // world's x axis and positive angles CCW

    float xVel;   // derivitive of xPos
    float yVel;   // derivitive of yPos
    float omega;  // derivitive of theta

    float xAccel;
    float yAccel;
    float alpha;
  } stateVector;

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
  struct timeval prevtime;
  struct timeval currtime;
  int dtms;  // dt in ms
  iVescAccess *fleftVesc;
  iVescAccess *frightVesc;
  iVescAccess *brightVesc;
  iVescAccess *bleftVesc;
  int timediffms(struct timeval curr, struct timeval prev);
};

#endif
