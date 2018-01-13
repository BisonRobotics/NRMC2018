#include <gtest/gtest.h>
#include <wheel_control/interface/test_interface.h>
#include <wheel_control/wheels/test_wheels.h>

using namespace wheel_control;

TEST(InterfaceTests, loadInterface)
{
  TestInterface test;
  TestWheels wheels;
  test.load(&wheels);

  ASSERT_STREQ("wheel_front_left", test.wheels->name[FLI].c_str());
  ASSERT_STREQ("wheel_front_right", test.wheels->name[FRI].c_str());
  ASSERT_STREQ("wheel_back_left", test.wheels->name[BLI].c_str());
  ASSERT_STREQ("wheel_back_right", test.wheels->name[BRI].c_str());
}

TEST(InterfaceTests, updateCurrentState)
{
  TestInterface test;
  TestWheels wheels;
  test.load(&wheels);

  sensor_msgs::JointState current_state;
  wheels.current_state.name[0] = "wheel_front_left";
  wheels.current_state.name[1] = "wheel_front_right";
  wheels.current_state.name[2] = "wheel_back_left";
  wheels.current_state.name[3] = "wheel_back_right";
  wheels.current_state.position[0] = 1.0;
  wheels.current_state.position[1] = 2.0;
  wheels.current_state.position[2] = 3.0;
  wheels.current_state.position[3] = 4.0;
  wheels.current_state.velocity[0] = 1.1;
  wheels.current_state.velocity[1] = 2.1;
  wheels.current_state.velocity[2] = 3.1;
  wheels.current_state.velocity[3] = 4.1;
  wheels.current_state.effort[0] = 1.2;
  wheels.current_state.effort[1] = 2.2;
  wheels.current_state.effort[2] = 3.2;
  wheels.current_state.effort[3] = 4.2;

  ASSERT_NEAR(1.0, test.wheels->current_state.position[FLI], 1e-10);
  ASSERT_NEAR(2.0, test.wheels->current_state.position[FRI], 1e-10);
  ASSERT_NEAR(3.0, test.wheels->current_state.position[BLI], 1e-10);
  ASSERT_NEAR(4.0, test.wheels->current_state.position[BRI], 1e-10);

  ASSERT_NEAR(1.1, test.wheels->current_state.velocity[FLI], 1e-10);
  ASSERT_NEAR(2.1, test.wheels->current_state.velocity[FRI], 1e-10);
  ASSERT_NEAR(3.1, test.wheels->current_state.velocity[BLI], 1e-10);
  ASSERT_NEAR(4.1, test.wheels->current_state.velocity[BRI], 1e-10);

  ASSERT_NEAR(1.2, test.wheels->current_state.effort[FLI], 1e-10);
  ASSERT_NEAR(2.2, test.wheels->current_state.effort[FRI], 1e-10);
  ASSERT_NEAR(3.2, test.wheels->current_state.effort[BLI], 1e-10);
  ASSERT_NEAR(4.2, test.wheels->current_state.effort[BRI], 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
