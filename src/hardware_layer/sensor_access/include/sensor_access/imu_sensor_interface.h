#ifndef __IMU_SENSOR_INTERFACE__
#define __IMU_SENSOR_INTERFACE__

#include <readable_sensors/readable_sensors.h>
#include <tf2/LinearMath/Quaternion.h>
class ImuSensorInterface : public ReadableSensors
{
public:
  virtual double getX() = 0;
  virtual double getY() = 0;
  virtual double getOmega() = 0;
  virtual tf2::Quaternion getOrientation ()= 0;
  virtual ReadableSensors::ReadStatus receiveData() = 0;
};

#endif
