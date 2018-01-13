#ifndef __IMU_SENSOR_INTERFACE__
#define __IMU_SENSOR_INTERFACE__

#include <readable_sensors/readable_sensors.h>

class ImuSensorInterface : public ReadableSensors
{
public:
  virtual float getX() = 0;
  virtual float getY() = 0;
  virtual float getOmega() = 0;

  virtual ReadableSensors::ReadStatus receiveData() = 0;
};

#endif
