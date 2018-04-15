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

TEST (safety_vesc_test, limit_switches_reset_position)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params);
  linearSafety.init ();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.update(.01);
  EXPECT_NEAR (linearSafety.getPositionEstimate(), linear_joint_params.lower_limit_position, .001);
  ON_CALL (vesc,getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.update(.01);
  EXPECT_NEAR (linearSafety.getPositionEstimate(), linear_joint_params.upper_limit_position, .001);
}

TEST (safety_vesc_test, velocities_get_capped_at_safety)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params);
  linearSafety.init ();
  EXPECT_CALL (vesc, setLinearVelocity(FloatNear(linear_joint_params.max_abs_velocity,.001)));
  linearSafety.setVelocity(10);
  linearSafety.update(.01);
}

TEST (safety_vesc_test, limit_switches_stop_motion_downward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params);
  linearSafety.init ();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.updatePositionEstimate(.01);
  linearSafety.setVelocity(-1.0);
  EXPECT_CALL (vesc, setLinearVelocity (FloatNear(0,.001)));
  linearSafety.update(.01);
}

TEST (safety_vesc_test, limit_switches_stop_motion_upward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params);
  linearSafety.init ();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.updatePositionEstimate(.01);
  linearSafety.setVelocity(1);
  EXPECT_CALL (vesc, setLinearVelocity (FloatNear(0,.001)));
  linearSafety.update(.01);
}

TEST (safety_vesc_test, throws_exception_on_out_of_bounds_setpoint)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety (&vesc, linear_joint_params);
  linearSafety.init ();
  EXPECT_CALL (vesc, setLinearVelocity(FloatNear(0,.001)));
  ASSERT_THROW(linearSafety.setPositionSetpoint(linear_joint_params.maximum_pos+1.0), BackhoeException);
  ASSERT_NEAR (linearSafety.getPositionSetpoint(), linearSafety.getPositionEstimate(),.001);
}


TEST (safety_vesc_test, stops_on_stop)
{
  NiceMock<MockVescAccess>vesc;
  SafetyController safetyController (&vesc, linear_joint_params);
  EXPECT_CALL (vesc, setLinearVelocity(FloatNear(0,.001)));
  safetyController.stop();
}

TEST (safety_vesc_test, calls_get_torque)
{
  NiceMock <MockVescAccess> vesc;
  SafetyController safetyController (&vesc, linear_joint_params);
  EXPECT_CALL (vesc, getTorque());
  safetyController.getTorque();
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
