#include <gtest/gtest.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include "safety_vesc/linear_safety_controller.h"
#include "wheel_params/wheel_params.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
using ::testing::FloatNear;

TEST (safety_vesc_test, init_stops_on_lower_limit_and_sets_estimate)
{
    NiceMock<MockVescAccess> vesc;
    LinearSafetyController linearSafetyController(linear_joint_params, &vesc,false);
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
    EXPECT_CALL (vesc, setTorque(_)).Times(1);
    linearSafetyController.init ();
    EXPECT_TRUE (linearSafetyController.isInit());
    EXPECT_NEAR (linearSafetyController.getPosition(), linear_joint_params.lower_limit_position, .001);
}

TEST (safety_vesc_test, update_position_integrates_velocity)
{
    NiceMock<MockVescAccess> vesc;
    LinearSafetyController linearSafetyController(linear_joint_params, &vesc, false);
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
    linearSafetyController.init ();
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::inTransit));
    EXPECT_CALL (vesc, getLinearVelocity());
    linearSafetyController.updatePosition(.01);
}


TEST (safety_vesc_test, resets_to_upper_limit_switch)
{
    NiceMock<MockVescAccess> vesc;
    LinearSafetyController linearSafetyController(linear_joint_params, &vesc, false);
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
    linearSafetyController.init ();
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
    linearSafetyController.updatePosition(.01);
    EXPECT_NEAR (linear_joint_params.upper_limit_position, linearSafetyController.getPosition(), .001);
}

TEST (safety_vesc_test, resets_to_lower_limit_switch)
{
    NiceMock<MockVescAccess> vesc;
    LinearSafetyController linearSafetyController(linear_joint_params, &vesc, false);
    ON_CALL (vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
    linearSafetyController.init ();
    linearSafetyController.updatePosition(.01);
    EXPECT_NEAR (linear_joint_params.lower_limit_position, linearSafetyController.getPosition(), .001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
