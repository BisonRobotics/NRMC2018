#include <gtest/gtest.h>
#include <localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
// Declare a test
TEST(MotionTracking, GoesForward)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.5));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= .0049 && loki.stateVector.xPos <= .0051);
  EXPECT_TRUE(loki.stateVector.yPos >= -.0001 && loki.stateVector.yPos <= .0001);
  EXPECT_TRUE(loki.stateVector.theta >= -.0001 && loki.stateVector.theta <= .0001);
}

TEST(MotionTracking, ForwardLeft)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= .0039 && loki.stateVector.xPos <= .0041);
  EXPECT_TRUE(loki.stateVector.yPos >= .0000079 && loki.stateVector.yPos <= .0000081);
  EXPECT_TRUE(loki.stateVector.theta >= .0039 && loki.stateVector.theta <= .0041);
}

TEST(MotionTracking, ForwardRight)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.5));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= .0039 && loki.stateVector.xPos <= .0041);
  EXPECT_TRUE(loki.stateVector.yPos >= -.0000081 && loki.stateVector.yPos <= -.0000079);
  EXPECT_TRUE(loki.stateVector.theta >= -.0041 && loki.stateVector.theta <= -.0039);
}

TEST(MotionTracking, GoesBackward)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= -.0051 && loki.stateVector.xPos <= -.0049);
  EXPECT_TRUE(loki.stateVector.yPos >= -.0001 && loki.stateVector.yPos <= .0001);
  EXPECT_TRUE(loki.stateVector.theta >= -.0001 && loki.stateVector.theta <= .0001);
}

TEST(MotionTracking, BackwardRight)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= -.0041 && loki.stateVector.xPos <= -.0039);
  EXPECT_TRUE(loki.stateVector.yPos >= -.0000081 && loki.stateVector.yPos <= -.0000079);
  EXPECT_TRUE(loki.stateVector.theta >= .0039 && loki.stateVector.theta <= .0041);
}

TEST(MotionTracking, BackwardLeft)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= -.0041 && loki.stateVector.xPos <= -.0039);
  EXPECT_TRUE(loki.stateVector.yPos >= .0000079 && loki.stateVector.yPos <= .0000081);
  EXPECT_TRUE(loki.stateVector.theta >= -.0041 && loki.stateVector.theta <= -.0039);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
