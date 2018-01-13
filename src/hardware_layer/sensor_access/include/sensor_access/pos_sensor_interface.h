#ifndef __POS_SENSOR_INTERFACE__
#define __POS_SENSOR_INTERFACE__

#include <readable_sensors/readable_sensors.h>

class PosSensorInterface : public ReadableSensors
{
public:
  virtual float getX() = 0;
  virtual float getY() = 0;
  virtual float getTheta() = 0;
  virtual float getServoTheta() = 0;

  virtual ReadableSensors::ReadStatus receiveData() = 0;
};

#endif
