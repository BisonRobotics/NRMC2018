#ifndef MOCK_IMU_SENSOR
#define MOCK_IMU_SENSOR

#include <can_sensors/imu_sensor_interface.h>
#include <gmock/gmock.h>

class MockImuSensor : public ImuSensorInterface
{
public:
  MOCK_METHOD0(getX, float(void));
  MOCK_METHOD0(getY, float(void));
  MOCK_METHOD0(getAlpha, float(void));
  MOCK_METHOD0(receiveData, ReadableSensors::ReadStatus());  // this is an "uninteresting function"
};

#endif
