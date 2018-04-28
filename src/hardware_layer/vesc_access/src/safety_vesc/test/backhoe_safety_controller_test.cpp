#include <gtest/gtest.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include "safety_vesc/backhoe_safety_controller.h"
#include "wheel_params/wheel_params.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
using ::testing::FloatNear;

TEST(safety_vesc_test, init_calls_potentiometer)
{
  NiceMock<MockVescAccess> vesc;
  BackhoeSafetyController backhoeSafetyController(linear_joint_params, &vesc);
  EXPECT_CALL(vesc, getPotPosition());
  backhoeSafetyController.init();
  EXPECT_TRUE(backhoeSafetyController.getInitStatus());
}

TEST(safety_vesc_test, update_position_calls_potentiometer)
{
  NiceMock<MockVescAccess> vesc;
  BackhoeSafetyController backhoeSafetyController(linear_joint_params, &vesc);
  EXPECT_CALL(vesc, getPotPosition());
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  backhoeSafetyController.updatePositionEstimate(0.01);
}

TEST(safety_vesc_test, get_torque_consults_the_vesc)
{
  NiceMock<MockVescAccess> vesc;
  BackhoeSafetyController backhoeSafetyController(linear_joint_params, &vesc);
  EXPECT_CALL(vesc, getTorque());
  backhoeSafetyController.getTorque();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
