#ifndef __MOCK_IMU_SENSOR__
#define __MOCK_IMU_SENSOR__

#include <sensor_access/imu_sensor_interface.h>
#include <gmock/gmock.h>

class MockImuSensor : public ImuSensorInterface
{
public:
  MOCK_METHOD0(getX, double(void));
  MOCK_METHOD0(getY, double(void));
  MOCK_METHOD0(getOmega, double(void));
  MOCK_METHOD0(receiveData, ReadableSensors::ReadStatus());  // this is an "uninteresting function"
};

#endif
