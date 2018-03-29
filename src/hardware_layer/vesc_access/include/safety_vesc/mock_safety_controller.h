#ifndef PROJECT_MOCK_SAFETY_VESC_H
#define PROJECT_MOCK_SAFETY_VESC_H

#include "safety_vesc/isafety_controller.h"
#include "gmock/gmock.h"

class MockSafetyController : public iSafetyController{
    MOCK_METHOD1 (setPositionSetpoint,void(double));
    MOCK_METHOD1 (setVelocitySetpoint, void(double));
    MOCK_METHOD0 (getPosition, double (void));
    MOCK_METHOD0 (updateVelocity, double(void));
    MOCK_METHOD0 (isInit, bool (void));
    MOCK_METHOD0 (isAtSetpoint, bool(void));
    MOCK_METHOD0 (init, void(void));
    MOCK_METHOD1 (updatePosition, void(double));
    MOCK_METHOD0 (stop, void(void));
    MOCK_METHOD0 (getSafetyPosition, double (void));
    MOCK_METHOD0 (getVelocity, double(void));
    MOCK_METHOD0 (getSetPosition, double (void));
};


#endif //PROJECT_MOCK_SAFETY_VESC_H
