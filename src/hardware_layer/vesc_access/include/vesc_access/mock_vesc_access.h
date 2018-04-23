#ifndef __MOCK_VESC_H_
#define __MOCK_VESC_H_

#include <vesc_access/ivesc_access.h>
#include <gmock/gmock.h>

class MockVescAccess : public iVescAccess
{
public:
  MOCK_METHOD1(setLinearVelocity, void(float meters_per_second));
  MOCK_METHOD1(setTorque, void(float newton_meters));
  MOCK_METHOD0(getLinearVelocity, float(void));
  MOCK_METHOD0(getTorque, float(void));
  MOCK_METHOD0(getLimitSwitchState, nsVescAccess::limitSwitchState(void));
  MOCK_METHOD0(getPotPosition, float(void));
  MOCK_METHOD1(setDuty, void(float));
};

#endif
