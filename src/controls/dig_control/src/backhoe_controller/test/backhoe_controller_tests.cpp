#include "safety_vesc/mock_safety_controller.h"
#include "gtest/gtest.h"
#include "backhoe_controller/backhoe_controller.h"
#include "gmock/gmock.h"
#include "wheel_params/wheel_params.h"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::_;
using ::testing::Lt;

TEST(backhoe_controller_test, only_returns_init_if_both_are_init)
{
  NiceMock<MockSafetyController> linear;
  NiceMock<MockSafetyController> backhoe;
  BackhoeController backhoeController(&backhoe, &linear);
  ON_CALL(backhoe, getInitStatus()).WillByDefault(Return(false));
  ON_CALL(linear, getInitStatus()).WillByDefault(Return(true));
  EXPECT_FALSE(backhoeController.getIsInit());
  ON_CALL(backhoe, getInitStatus()).WillByDefault(Return(true));
  EXPECT_TRUE(backhoeController.getIsInit());
}

TEST(backhoe_controller_test, only_allows_setpoints_if_both_init)
{
  NiceMock<MockSafetyController> linear;
  NiceMock<MockSafetyController> backhoe;
  BackhoeController backhoeController(&backhoe, &linear);
  ON_CALL(backhoe, getInitStatus()).WillByDefault(Return(false));
  ON_CALL(linear, getInitStatus()).WillByDefault(Return(false));
  EXPECT_CALL(linear, setPositionSetpoint(_)).Times(0);
  EXPECT_CALL(backhoe, setPositionSetpoint(_)).Times(0);
  EXPECT_CALL(linear, setVelocity(_)).Times(0);
  EXPECT_CALL(backhoe, setVelocity(_)).Times(0);
  backhoeController.setWristSetpoint(0);
  backhoeController.setShoulderSetpoint(0);
  backhoeController.setWristVelocity(0);
  backhoeController.setShoulderTorque(0);
  backhoeController.update(.01);
}

TEST(backhoe_controller_test, safety_check_works)
{
  NiceMock<MockSafetyController> linear;
  NiceMock<MockSafetyController> backhoe;
  BackhoeController backhoeController(&backhoe, &linear);
  ON_CALL(linear, getInitStatus()).WillByDefault(Return(true));
  ON_CALL(backhoe, getInitStatus()).WillByDefault(Return(true));
  ON_CALL(linear, getSafetyPosition()).WillByDefault(Return(.1));
  ON_CALL(backhoe, getLinearVelocity()).WillByDefault(Return(2));
  ON_CALL(backhoe, getSafetyPosition()).WillByDefault(Return(0));
  ON_CALL(backhoe, getPositionEstimate()).WillByDefault(Return(.2));
  ON_CALL(linear, getPositionEstimate()).WillByDefault(Return(.2));
  ON_CALL(linear, getLinearVelocity()).WillByDefault(Return(2));
  EXPECT_CALL(backhoe, stop());
  EXPECT_CALL(linear, stop());
  backhoeController.update(.1);
}

TEST(backhoe_controller_test, safety_check_works_for_negative_velocities)
{
  NiceMock<MockSafetyController> linear;
  NiceMock<MockSafetyController> backhoe;
  BackhoeController backhoeController(&backhoe, &linear);
  ON_CALL(linear, getInitStatus()).WillByDefault(Return(true));
  ON_CALL(backhoe, getInitStatus()).WillByDefault(Return(true));
  ON_CALL(linear, getSafetyPosition()).WillByDefault(Return(.1));
  ON_CALL(backhoe, getLinearVelocity()).WillByDefault(Return(2));
  ON_CALL(backhoe, getSafetyPosition()).WillByDefault(Return(0));
  ON_CALL(backhoe, getPositionEstimate()).WillByDefault(Return(.2));
  ON_CALL(linear, getPositionEstimate()).WillByDefault(Return(.2));
  ON_CALL(linear, getLinearVelocity()).WillByDefault(Return(-2));
  EXPECT_CALL(backhoe, stop());
  EXPECT_CALL(linear, stop()).Times(0);
  backhoeController.update(.1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
