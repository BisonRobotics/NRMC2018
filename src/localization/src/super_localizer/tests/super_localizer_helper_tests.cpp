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
  LocalizerInterface::stateVector v1 = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};
  LocalizerInterface::stateVector v2 = {2.0, 3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
  LocalizerInterface::stateVector v3 = diff(v2,v1);

  EXPECT_NEAR (v3.x_pos, 1.0, .01);
  EXPECT_NEAR (v3.y_pos, 1.0, .01);
  EXPECT_NEAR (v3.theta, 1.0, .01);
  EXPECT_NEAR (v3.x_vel, 1.0, .01);
  EXPECT_NEAR (v3.y_vel, 1.0, .01);
  EXPECT_NEAR (v3.omega, 1.0, .01);
  EXPECT_NEAR (v3.x_accel, 1.0, .01);
  EXPECT_NEAR (v3.y_accel, 1.0, .01);
  EXPECT_NEAR (v3.alpha, 1.0, .01);
}


/*TEST(LocalizerHelperTests, CanAddFromModel)
{
  LocalizerInterface::stateVector v1 = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};
  LocalizerInterface::stateVector v2 = {2.0, 3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
  LocalizerInterface::stateVector v3 = addfrommodel(v2,v1,1.0f);

  EXPECT_NEAR (v3.x_pos, 3.0, .01);
  EXPECT_NEAR (v3.y_pos, 5.0, .01);
  EXPECT_NEAR (v3.theta, 7.0, .01);
  EXPECT_NEAR (v3.x_vel, 9.0, .01);
  EXPECT_NEAR (v3.y_vel, 11.0, .01);
  EXPECT_NEAR (v3.omega, 13.0, .01);
  EXPECT_NEAR (v3.x_accel, 15.0, .01);
  EXPECT_NEAR (v3.y_accel, 17.0, .01);
  EXPECT_NEAR (v3.alpha, 19.0, .01);
}
*/



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
