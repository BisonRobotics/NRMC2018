#include <gtest/gtest.h>
#include <wheel_control/wheels/test_wheels.h>

using namespace wheel_control;

TEST(WheelsTests, instantiateWheels)
{
  TestWheels test;

  ASSERT_STREQ("wheel_front_left", test.name[FLI].c_str());
  ASSERT_STREQ("wheel_front_right", test.name[FRI].c_str());
  ASSERT_STREQ("wheel_back_left", test.name[BLI].c_str());
  ASSERT_STREQ("wheel_back_right", test.name[BRI].c_str());
}

TEST(WheelsTests, instantiateWheelsWithParams)
{
  TestWheels test(1.0, 1.0);

  ASSERT_STREQ("wheel_front_left", test.name[FLI].c_str());
  ASSERT_STREQ("wheel_front_right", test.name[FRI].c_str());
  ASSERT_STREQ("wheel_back_left", test.name[BLI].c_str());
  ASSERT_STREQ("wheel_back_right", test.name[BRI].c_str());

  ASSERT_NEAR(0.5, test.x_pos[FLI], 1e-10);
  ASSERT_NEAR(0.5, test.x_pos[FRI], 1e-10);
  ASSERT_NEAR(-0.5, test.x_pos[BLI], 1e-10);
  ASSERT_NEAR(-0.5, test.x_pos[BRI], 1e-10);

  ASSERT_NEAR(0.5, test.y_pos[FLI], 1e-10);
  ASSERT_NEAR(-0.5, test.y_pos[FRI], 1e-10);
  ASSERT_NEAR(0.5, test.y_pos[BLI], 1e-10);
  ASSERT_NEAR(-0.5, test.y_pos[BRI], 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
