#ifndef __POS_SENSOR_INTERFACE__
#define __POS_SENSOR_INTERFACE__

#include <readable_sensors/readable_sensors.h>

class PosSensorInterface : public ReadableSensors
{
public:
  virtual double getX() = 0;
  virtual double getY() = 0;
  virtual double getZ() = 0;
  virtual double getTheta() = 0;
  virtual bool isFloating() = 0;
  virtual ReadableSensors::ReadStatus receiveData() = 0;
};

#endif
