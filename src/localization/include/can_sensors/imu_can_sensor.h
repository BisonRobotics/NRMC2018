#ifndef IMU_CAN_SENSOR_H
#define IMU_CAN_SENSOR_H

#include <can_sensors.h>

class IMUCanSensor : public CanSensor
{
public:
  typedef struct
  {
    float x;  // x acceleration in m/s^2
    float y;  // y
    float z;  // z
  } imuData;

  IMUCanSensor(int cID, char* interface);

  int recieveData(void* ret);

private:
  uint8_t recieveBuffer[8];
};

#endif