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

TEST(safety_vesc_test, limit_switches_reset_position)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.update(.01);
  EXPECT_NEAR(linearSafety.getPositionEstimate(), linear_joint_params.lower_limit_position, .001);
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.update(.01);
  EXPECT_NEAR(linearSafety.getPositionEstimate(), linear_joint_params.upper_limit_position, .001);
}

TEST(safety_vesc_test, torques_get_capped)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::inTransit));
  EXPECT_CALL(vesc, setTorque(FloatNear(linear_joint_params.max_abs_torque, .001)));
  linearSafety.setTorque(200);
  linearSafety.update(.01);
  EXPECT_CALL(vesc, setTorque(FloatNear(-linear_joint_params.max_abs_torque, .001)));
  linearSafety.setTorque(-100);
  linearSafety.update(.01);
}

TEST(safety_vesc_test, velocities_get_capped_at_safety)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::inTransit));
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(linear_joint_params.max_abs_velocity, .001)));
  linearSafety.setVelocity(10);
  linearSafety.update(.01);
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(-linear_joint_params.max_abs_velocity, .001)));
  linearSafety.setVelocity(-100);
  linearSafety.update(.01);
}

TEST(safety_vesc_test, limit_switches_stop_motion_downward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  linearSafety.setVelocity(-1.0);
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(0, .001)));
  linearSafety.update(.01);
}

TEST(safety_vesc_test, limit_switches_stop_motion_upward)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::topOfMotion));
  linearSafety.setVelocity(1);
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(0, .001)));
  linearSafety.update(.01);
}

TEST(safety_vesc_test, throws_exception_on_out_of_bounds_setpoint)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController linearSafety(&vesc, linear_joint_params);
  linearSafety.init();
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(0, .001)));
  ASSERT_THROW(linearSafety.setPositionSetpoint(linear_joint_params.maximum_pos + 1.0), BackhoeException);
  ASSERT_NEAR(linearSafety.getPositionSetpoint(), linearSafety.getPositionEstimate(), .001);
}

TEST(safety_vesc_test, stops_on_stop)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController safetyController(&vesc, linear_joint_params);
  EXPECT_CALL(vesc, setLinearVelocity(FloatNear(0, .001)));
  safetyController.stop();
}

TEST(safety_vesc_test, calls_get_torque)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController safetyController(&vesc, linear_joint_params);
  EXPECT_CALL(vesc, getTorque());
  safetyController.getTorque();
}

TEST(safety_vesc_test, throws_exception_on_velocity_setpoint_after_pos)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController safetyController(&vesc, linear_joint_params);
  safetyController.init();
  safetyController.setPositionSetpoint(.01);
  EXPECT_THROW(safetyController.setVelocity(10), BackhoeException);
}

TEST(safety_vesc_test, throws_exception_on_torque_setpoint_after_pos)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController safetyController(&vesc, linear_joint_params);
  safetyController.init();
  safetyController.setPositionSetpoint(0);
  EXPECT_THROW(safetyController.setTorque(10), BackhoeException);
}

TEST(safety_vesc_test, can_transition_between_position_and_torque)
{
  NiceMock<MockVescAccess> vesc;
  SafetyController safetyController(&vesc, linear_joint_params);
  safetyController.init();
  safetyController.setPositionSetpoint(0);
  EXPECT_CALL(vesc, setTorque(_));
  ON_CALL(vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::inTransit));
  safetyController.abandonPositionSetpointAndSetTorqueWithoutStopping(0);
  safetyController.update(.01);
  EXPECT_EQ(safetyController.getControlMode(), safetycontroller::controlModeState::torque_control);
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
