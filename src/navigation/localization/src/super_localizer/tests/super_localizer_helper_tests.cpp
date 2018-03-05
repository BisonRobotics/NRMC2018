#include <gtest/gtest.h>
#include <super_localizer/super_localizer_helper.h>
//#include <vesc_access/mock_vesc_access.h>
//#include <can_sensors/mock_pos_can_sensor.h>
//#include <can_sensors/mock_imu_can_sensor.h>
#include <gmock/gmock.h>

#include <random>
#include <cmath>

#define POSTOL .03f
#define RADTOL .05f

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

TEST(LocalizerHelperTests, CanSub)
{
  LocalizerInterface::stateVector v1 = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
  LocalizerInterface::stateVector v2 = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 };

  LocalizerInterface::stateVector v3 = LocalizerInterface::diff(v2, v1);

  EXPECT_NEAR(v3.x_pos, 1.0, .01);
  EXPECT_NEAR(v3.y_pos, 1.0, .01);
  EXPECT_NEAR(v3.theta, 1.0, .01);
  EXPECT_NEAR(v3.x_vel, 1.0, .01);
  EXPECT_NEAR(v3.y_vel, 1.0, .01);
  EXPECT_NEAR(v3.omega, 1.0, .01);
  EXPECT_NEAR(v3.x_accel, 1.0, .01);
  EXPECT_NEAR(v3.y_accel, 1.0, .01);
  EXPECT_NEAR(v3.alpha, 1.0, .01);
}
/*
TEST(LocalizerHelperTests, CanAddFromModelWithOutIMU)
{
  LocalizerInterface::stateVector v1 = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
  LocalizerInterface::stateVector v2 = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 };
  LocalizerInterface::stateVector v3 = LocalizerInterface::addFromModel(v2, v1, 1.0f, false);

  EXPECT_NEAR(v3.x_pos, v2.x_pos + v1.x_vel, .01);
  EXPECT_NEAR(v3.y_pos, v2.y_pos + v1.y_vel, .01);
  EXPECT_NEAR(v3.theta, v2.theta + v1.omega, .01);
  EXPECT_NEAR(v3.x_vel, v1.x_vel, .01);
  EXPECT_NEAR(v3.y_vel, v1.y_vel, .01);
  EXPECT_NEAR(v3.omega, v1.omega, .01);
  EXPECT_NEAR(v3.x_accel, 0.0f, .01);
  EXPECT_NEAR(v3.y_accel, 0.0f, .01);
  EXPECT_NEAR(v3.alpha, 0.0f, .01);
}

TEST(LocalizerHelperTests, CanAddFromModelWithIMU)
{
  LocalizerInterface::stateVector v1 = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
  LocalizerInterface::stateVector v2 = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 };
  LocalizerInterface::stateVector v3 = LocalizerInterface::addFromModel(v2, v1, 1.0f, true);

  EXPECT_NEAR(v3.x_pos, v2.x_pos + v1.x_vel, .01);
  EXPECT_NEAR(v3.y_pos, v2.y_pos + v1.y_vel, .01);
  EXPECT_NEAR(v3.theta, v2.theta + v1.omega, .01);
  EXPECT_NEAR(v3.x_vel, v2.x_vel + v1.x_accel, .01);
  EXPECT_NEAR(v3.y_vel, v2.y_vel + v1.y_accel, .01);
  EXPECT_NEAR(v3.omega, v2.omega + v1.alpha, .01);
  EXPECT_NEAR(v3.x_accel, v1.x_accel, .01);
  EXPECT_NEAR(v3.y_accel, v1.y_accel, .01);
  EXPECT_NEAR(v3.alpha, v1.alpha, .01);
}
*/
TEST(LocalizerHelperTests, CanMultiply)
{
  LocalizerInterface::stateVector v1 = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
  LocalizerInterface::stateVector v2 = { 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 };
  LocalizerInterface::stateVector v3 = LocalizerInterface::multiply(v2, v1);

  EXPECT_NEAR(v3.x_pos, v2.x_pos * v1.x_pos, .01);
  EXPECT_NEAR(v3.y_pos, v2.y_pos * v1.y_pos, .01);
  EXPECT_NEAR(v3.theta, v2.theta * v1.theta, .01);
  EXPECT_NEAR(v3.x_vel, v2.x_vel * v1.x_vel, .01);
  EXPECT_NEAR(v3.y_vel, v2.y_vel * v1.y_vel, .01);
  EXPECT_NEAR(v3.omega, v2.omega * v1.omega, .01);
  EXPECT_NEAR(v3.x_accel, v1.x_accel * v2.x_accel, .01);
  EXPECT_NEAR(v3.y_accel, v1.y_accel * v2.y_accel, .01);
  EXPECT_NEAR(v3.alpha, v1.alpha * v2.alpha, .01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
