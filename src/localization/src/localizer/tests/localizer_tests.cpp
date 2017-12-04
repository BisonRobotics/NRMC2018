#include <gtest/gtest.h>
#include <localizer/localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;

// Declare a test
TEST(LocalizerTests, GoesForward)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= .0049 && loki.getStateVector().x_pos <= .0051);
  EXPECT_TRUE(loki.getStateVector().y_pos >= -.0001 && loki.getStateVector().y_pos <= .0001);
  EXPECT_TRUE(loki.getStateVector().theta >= -.0001 && loki.getStateVector().theta <= .0001);
}

TEST(LocalizerTests, ForwardLeft)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= .0039 && loki.getStateVector().x_pos <= .0041);
  EXPECT_TRUE(loki.getStateVector().y_pos >= .0000079 && loki.getStateVector().y_pos <= .0000081);
  EXPECT_TRUE(loki.getStateVector().theta >= .0039 && loki.getStateVector().theta <= .0041);
}

TEST(LocalizerTests, ForwardRight)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= .0039 && loki.getStateVector().x_pos <= .0041);
  EXPECT_TRUE(loki.getStateVector().y_pos >= -.0000081 && loki.getStateVector().y_pos <= -.0000079);
  EXPECT_TRUE(loki.getStateVector().theta >= -.0041 && loki.getStateVector().theta <= -.0039);
}

TEST(LocalizerTests, GoesBackward)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= -.0051 && loki.getStateVector().x_pos <= -.0049);
  EXPECT_TRUE(loki.getStateVector().y_pos >= -.0001 && loki.getStateVector().y_pos <= .0001);
  EXPECT_TRUE(loki.getStateVector().theta >= -.0001 && loki.getStateVector().theta <= .0001);
}

TEST(LocalizerTests, BackwardRight)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= -.0041 && loki.getStateVector().x_pos <= -.0039);
  EXPECT_TRUE(loki.getStateVector().y_pos >= -.0000081 && loki.getStateVector().y_pos <= -.0000079);
  EXPECT_TRUE(loki.getStateVector().theta >= .0039 && loki.getStateVector().theta <= .0041);
}

TEST(LocalizerTests, BackwardLeft)
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
  EXPECT_TRUE(loki.getStateVector().x_pos >= -.0041 && loki.getStateVector().x_pos <= -.0039);
  EXPECT_TRUE(loki.getStateVector().y_pos >= .0000079 && loki.getStateVector().y_pos <= .0000081);
  EXPECT_TRUE(loki.getStateVector().theta >= -.0041 && loki.getStateVector().theta <= -.0039);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
