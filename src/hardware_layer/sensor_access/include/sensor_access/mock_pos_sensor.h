#ifndef __MOCK_POS_SENSOR__
#define __MOCK_POS_SENSOR__

#include <sensor_access/pos_sensor_interface.h>
#include <gmock/gmock.h>

class MockPosSensor : public PosSensorInterface
{
public:
  MOCK_METHOD0(getX, float());
  MOCK_METHOD0(getY, float());
  MOCK_METHOD0(getTheta, float());
  MOCK_METHOD0(getServoTheta, float());
  MOCK_METHOD0(receiveData, ReadableSensors::ReadStatus());  // this is an "uninteresting function"
};

#endif
