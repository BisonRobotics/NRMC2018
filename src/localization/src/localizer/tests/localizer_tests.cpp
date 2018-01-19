#include <gtest/gtest.h>
#include <localizer/localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

#define POSTOL .03f
#define RADTOL .05f

// Declare a test
TEST(LocalizerTests, GoesForward)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.5));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, .005, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0, .0001);
  EXPECT_NEAR(loki.getStateVector().theta, 0, .0001);

  ASSERT_NEAR(loki.getStateVector().x_vel, .5f, POSTOL);
  ASSERT_NEAR(loki.getStateVector().y_vel, .0f, POSTOL);
  ASSERT_NEAR(loki.getStateVector().omega, .0f, RADTOL);
}

TEST(LocalizerTests, ForwardLeft)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, .004, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, .000008, .0000001);
  EXPECT_NEAR(loki.getStateVector().theta, .004, .0001);
}

TEST(LocalizerTests, ForwardRight)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.5));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, .004, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, -.000008, .0000001);
  EXPECT_NEAR(loki.getStateVector().theta, -.004, .0001);
}

TEST(LocalizerTests, GoesBackward)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.5));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, -.005, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0, .0001);
  EXPECT_NEAR(loki.getStateVector().theta, 0, .0001);
}

TEST(LocalizerTests, BackwardRight)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.5));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, -.004, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, -.000008, .0000001);
  EXPECT_NEAR(loki.getStateVector().theta, .004, .0001);
}

TEST(LocalizerTests, BackwardLeft)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  Localizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_NEAR(loki.getStateVector().x_pos, -.004, .0001);
  EXPECT_NEAR(loki.getStateVector().y_pos, .000008, .0000001);
  EXPECT_NEAR(loki.getStateVector().theta, -.004, .0001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
