#ifndef PROJECT_MOCK_SAFETY_VESC_H
#define PROJECT_MOCK_SAFETY_VESC_H

#include "safety_vesc/isafety_vesc.h"
#include "gmock/gmock.h"

class MockSafetyVesc : public iSafetyVesc{
    MOCK_METHOD1 (setPositionSetpoint,void(double));
    MOCK_METHOD1 (setVelocitySetpoint, void(double));
    MOCK_METHOD0 (getPosition, double (void));
    MOCK_METHOD0 (updateVelocity, double(void));
    MOCK_METHOD0 (isInit, bool (void));
    MOCK_METHOD0 (isAtSetpoint, bool(void));
    MOCK_METHOD0 (init, void(void));
    MOCK_METHOD1 (updatePosition, void(double));
};


#endif //PROJECT_MOCK_SAFETY_VESC_H
