#ifndef MOCK_POS_SENSOR
#define MOCK_POS_SENSOR

#include <can_sensors/pos_can_sensor_interface.h>
#include <gmock/gmock.h>

class MockPosCanSensor : public PosCanSensorInterface
{
public:
  MOCK_METHOD0(getX, float());
  MOCK_METHOD0(getY, float());
  MOCK_METHOD0(getTheta, float());
  MOCK_METHOD0(getServoTheta, float());
  MOCK_METHOD0(receiveData, ReadableSensors::ReadStatus());  // this is an "uninteresting function"
};

#endif