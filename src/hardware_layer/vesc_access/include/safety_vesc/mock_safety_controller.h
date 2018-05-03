#ifndef PROJECT_MOCK_SAFETY_VESC_H
#define PROJECT_MOCK_SAFETY_VESC_H

#include "safety_vesc/isafety_controller.h"
#include "gmock/gmock.h"

class MockSafetyController : public iSafetyController
{
public:
  MOCK_METHOD1(setPositionSetpoint, void(double));
  MOCK_METHOD1(setVelocity, void(double));
  MOCK_METHOD0(updateVelocity, double(void));
  MOCK_METHOD0(isAtSetpoint, bool(void));
  MOCK_METHOD0(init, bool(void));
  MOCK_METHOD1(updatePositionEstimate, void(double));
  MOCK_METHOD0(stop, void(void));
  MOCK_METHOD0(getSafetyPosition, double(void));
  MOCK_METHOD0(getSetPosition, double(void));
  MOCK_METHOD0(getTorque, float(void));
  MOCK_METHOD0(getLinearVelocity, float(void));
  MOCK_METHOD1(setTorque, void(double));
  MOCK_METHOD0(getInitStatus, bool(void));
  MOCK_METHOD0(checkIsInit, void(void));
  MOCK_METHOD0(getPositionSetpoint, double(void));
  MOCK_METHOD1(abandonPositionSetpointAndSetTorqueWithoutStopping, void(double));
  MOCK_METHOD1(update, void(double));
  MOCK_METHOD0(getPositionEstimate, double(void));
  MOCK_METHOD0(getControlMode, safetycontroller::controlModeState(void));
  MOCK_METHOD0(getCommandedTorque,float(void));
  MOCK_METHOD0(getCommandedVelocity, float (void));
};

#endif  // PROJECT_MOCK_SAFETY_VESC_H
