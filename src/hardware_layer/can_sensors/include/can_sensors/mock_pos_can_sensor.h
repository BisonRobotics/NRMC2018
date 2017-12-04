#ifndef MOCK_POS_SENSOR
#define MOCK_POS_SENSOR

#include <can_sensors/pos_can_sensor_interface.h>
#include <gmock/gmock.h>

class MockPosCanSensor : public PosCanSensorInterface
{
public:
  MOCK_METHOD0(getX, float(void));
  MOCK_METHOD0(getY, float(void));
  MOCK_METHOD0(getTheta, float(void));
  MOCK_METHOD0(getServoTheta, float(void));
};

#endif