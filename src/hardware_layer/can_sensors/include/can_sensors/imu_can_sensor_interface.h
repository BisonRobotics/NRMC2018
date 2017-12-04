#ifndef IMU_CAN_SENSOR_INTERFACE
#define IMU_CAN_SENSOR_INTERFACE

#include <readable_sensors/readable_sensors.h>

class ImuCanSensorInterface : public ReadableSensors
{
public:
  virtual float getX()=0;
  virtual float getY()=0;
  virtual float getTheta()=0;

  virtual ReadableSensors::ReadStatus receiveData() =0;
};

#endif