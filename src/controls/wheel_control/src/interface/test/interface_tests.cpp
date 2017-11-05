#include <gtest/gtest.h>
#include "wheel_control/interface/interface.h"

using namespace wheel_control;

TEST(InterfaceTests, loadInterface)
{
  Interface test;
  Wheels *wheels = new Wheels();
  test.load(wheels);

  ASSERT_STREQ("wheel_front_left",  test.wheels->name[FLI].c_str());
  ASSERT_STREQ("wheel_front_right", test.wheels->name[FRI].c_str());
  ASSERT_STREQ("wheel_back_left",   test.wheels->name[BLI].c_str());
  ASSERT_STREQ("wheel_back_right",  test.wheels->name[BRI].c_str());
}

TEST(InterfaceTests, updateMethod)
{
  Interface test;
  Wheels *wheels = new Wheels();
  test.load(wheels);

  sensor_msgs::JointState current_state;
  current_state.name.push_back("wheel_front_left");
  current_state.name.push_back("wheel_front_right");
  current_state.name.push_back("wheel_back_left");
  current_state.name.push_back("wheel_back_right");
  current_state.position.push_back(1.0);
  current_state.position.push_back(2.0);
  current_state.position.push_back(3.0);
  current_state.position.push_back(4.0);
  current_state.velocity.push_back(1.1);
  current_state.velocity.push_back(2.1);
  current_state.velocity.push_back(3.1);
  current_state.velocity.push_back(4.1);
  current_state.effort.push_back(1.2);
  current_state.effort.push_back(2.2);
  current_state.effort.push_back(3.2);
  current_state.effort.push_back(4.2);

  test.update(&current_state);

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

TEST(InterfaceTests, updateMethodInvalidArgument)
{
  Interface test;
  Wheels *wheels = new Wheels();
  test.load(wheels);

  sensor_msgs::JointState current_state;
  current_state.name.push_back("wheel_front_left");
  current_state.name.push_back("wheel_front_right");
  current_state.name.push_back("wheel_back_left");
  current_state.name.push_back("invalid");
  current_state.position.push_back(1.0);
  current_state.position.push_back(2.0);
  current_state.position.push_back(3.0);
  current_state.position.push_back(4.0);
  current_state.velocity.push_back(1.1);
  current_state.velocity.push_back(2.1);
  current_state.velocity.push_back(3.1);
  current_state.velocity.push_back(4.1);
  current_state.effort.push_back(1.2);
  current_state.effort.push_back(2.2);
  current_state.effort.push_back(3.2);
  current_state.effort.push_back(4.2);


  ASSERT_THROW(test.update(&current_state), std::invalid_argument);
  try
  {
    test.update(&current_state);
  }
  catch (std::invalid_argument e)
  {
    ASSERT_STREQ("invalid wheel not found", e.what());
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
