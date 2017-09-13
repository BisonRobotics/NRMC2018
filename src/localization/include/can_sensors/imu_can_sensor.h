#ifndef IMU_CAN_SENSOR_H
#define IMU_CAN_SENSOR_H

#include <can_sensors.h>
#include <readable_sensors.h>

class IMUCanSensor : public CanSensor, public ReadableSensors
{
public:
  struct data_s
  {
    float x;  // x acceleration in m/s^2
    float y;  // y
    float z;  // z
  } imuData;

  IMUCanSensor(int cID, char* interface);

  ReadableSensors::ReadStatus receiveData();

private:
  uint8_t receiveBuffer[8];
};

#endif
