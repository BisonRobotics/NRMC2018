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
using ::testing::FloatNear;

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
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.updatePosition(.01);
  EXPECT_NEAR (linearSafety.getPosition(), linear_joint_params.lower_limit_position, .001);
  ON_CALL (vesc,getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.updatePosition(.01);
  EXPECT_NEAR (linearSafety.getPosition(), linear_joint_params.upper_limit_position, .001);
}

TEST (safety_vesc_test, velocities_get_capped_at_safety)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, true);
  linearSafety.init ();
  EXPECT_CALL (vesc, setLinearVelocity(FloatNear(linear_joint_params.max_abs_velocity,.001)));
  linearSafety.setVelocitySetpoint(10);
  linearSafety.updateVelocity();
}

TEST (safety_vesc_test, limit_switches_stop_motion_downward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, true);
  linearSafety.init ();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.updatePosition(.01);
  linearSafety.setVelocitySetpoint(-1.0);
  EXPECT_CALL (vesc, setLinearVelocity (FloatNear(0,.001)));
  linearSafety.updateVelocity();
}

TEST (safety_vesc_test, limit_switches_stop_motion_upward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, true);
  linearSafety.init ();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.updatePosition(.01);
  linearSafety.setVelocitySetpoint(1);
  EXPECT_CALL (vesc, setLinearVelocity (FloatNear(0,.001)));
  linearSafety.updateVelocity();
}

TEST (safety_vesc_test, throws_exception_on_out_of_bounds_setpoint)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params, false);
  linearSafety.init ();
  bool exception_thrown = false;
  try {
   linearSafety.setPositionSetpoint(linear_joint_params.maximum_pos+1.0);
  } catch (BackhoeSetPointException backhoeSetPointException){
   exception_thrown = true;
  }
  EXPECT_TRUE (exception_thrown);
}


TEST (safety_vesc_test, stops_on_stop)
{
  NiceMock<MockVescAccess>vesc;
  SafetyController safetyController (&vesc, linear_joint_params, false);
  EXPECT_CALL (vesc, setLinearVelocity(FloatNear(0,.001)));
  safetyController.stop();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
