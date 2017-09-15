#ifndef __MOCK_VESC_H_
#define __MOCK_VESC_H_

#include <vesc_control/ivesc.h>
#include <gmock/gmock.h>

class MockVesc : public iVesc
{
public:
  //    MOCK_METHOD0 (~mockVesc);
  MOCK_METHOD1(setRpm, void(float rpm));
  MOCK_METHOD1(setCurrent, void(float current));
  MOCK_METHOD0(getCurrent, float(void));
  MOCK_METHOD0(getRpm, int(void));
};

#endif
