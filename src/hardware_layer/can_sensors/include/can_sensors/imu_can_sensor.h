#ifndef IMU_CAN_SENSOR_H
#define IMU_CAN_SENSOR_H

#include <can_sensors/can_sensors.h>
#include <can_sensors/imu_can_sensor_interface.h>

class IMUCanSensor : public CanSensor, public ImuCanSensorInterface
{
public:
  IMUCanSensor(int cID, char* interface);

  ReadableSensors::ReadStatus receiveData();

  float getX();
  float getY();
  float getTheta();

private:
  uint8_t receiveBuffer[8];
  struct data_s
  {
    float x;  // x acceleration in m/s^2
    float y;  // y
    float z;  // z
  } imuData;
};

#endif
