#include <gtest/gtest.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include "safety_vesc/safety_controller.h"
#include "wheel_params/wheel_params.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;


TEST (safety_vesc_test, inits_position_and_velocity_to_zero)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params,false);
  EXPECT_NEAR (linearSafety.getPosition(),0, .0001);
  EXPECT_NEAR (linearSafety.getVelocity(),0, .0001);
}


TEST (safety_vesc_test, in_velocity_mode_doesnt_set_position)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, true);
  linearSafety.init();
  linearSafety.setPositionSetpoint(.2);
  EXPECT_NEAR (linearSafety.getSetPosition(), 0,.001);
}


TEST (safety_vesc_test, in_velocity_doesnt_set_until_init)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, true);
  linearSafety.setVelocitySetpoint(.2);
  EXPECT_NEAR(linearSafety.getVelocity(),0,.001);
  linearSafety.init ();
  linearSafety.setVelocitySetpoint(.2);
  EXPECT_NEAR (linearSafety.getVelocity(), .2,.001);
}

TEST (safety_vesc_test, in_position_mode_velocity_doesnt_set)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, false);
  linearSafety.init ();
  linearSafety.setVelocitySetpoint(.3);
  EXPECT_NEAR (linearSafety.getVelocity(), 0,.001);
}

TEST (safety_vesc_test, limit_switches_reset_position)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, false);
  linearSafety.init ();
  ON_CALL(vesc, getPosition[])
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
