#ifndef __MOCK_POS_SENSOR__
#define __MOCK_POS_SENSOR__

#include <sensor_access/pos_sensor_interface.h>
#include <gmock/gmock.h>

class MockPosSensor : public PosSensorInterface
{
public:
  MOCK_METHOD0(getX, double());
  MOCK_METHOD0(getY, double());
  MOCK_METHOD0(getTheta, double());
  MOCK_METHOD0(isFloating, bool());
  MOCK_METHOD0(receiveData, ReadableSensors::ReadStatus());  // this is an "uninteresting function"
  MOCK_METHOD0 (getZ, double());
};

#endif
