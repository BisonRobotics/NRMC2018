#include <gtest/gtest.h>
#include "wheel_control/interface/interface.h"

using namespace wheel_control;

TEST(InterfaceTests, loadInterface)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  ASSERT_STREQ("front_right", test.wheels->front_right->name.c_str());
  ASSERT_STREQ("back_right", test.wheels->back_right->name.c_str());
  ASSERT_STREQ("front_left", test.wheels->front_left->name.c_str());
  ASSERT_STREQ("back_left", test.wheels->back_left->name.c_str());
}

TEST(InterfaceTests, updateMethod)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  JointStates current_state = { { "front_right", JointState(1.0, 1.1, 1.2) },
                                { "back_right", JointState(2.0, 2.1, 2.2) },
                                { "front_left", JointState(3.0, 3.1, 3.2) },
                                { "back_left", JointState(4.0, 4.1, 4.2) } };
  test.update(current_state);

  ASSERT_NEAR(1.0, test.wheels->front_right->current_state->position, 1e-10);
  ASSERT_NEAR(2.0, test.wheels->back_right->current_state->position, 1e-10);
  ASSERT_NEAR(3.0, test.wheels->front_left->current_state->position, 1e-10);
  ASSERT_NEAR(4.0, test.wheels->back_left->current_state->position, 1e-10);

  ASSERT_NEAR(1.1, test.wheels->front_right->current_state->velocity, 1e-10);
  ASSERT_NEAR(2.1, test.wheels->back_right->current_state->velocity, 1e-10);
  ASSERT_NEAR(3.1, test.wheels->front_left->current_state->velocity, 1e-10);
  ASSERT_NEAR(4.1, test.wheels->back_left->current_state->velocity, 1e-10);

  ASSERT_NEAR(1.2, test.wheels->front_right->current_state->effort, 1e-10);
  ASSERT_NEAR(2.2, test.wheels->back_right->current_state->effort, 1e-10);
  ASSERT_NEAR(3.2, test.wheels->front_left->current_state->effort, 1e-10);
  ASSERT_NEAR(4.2, test.wheels->back_left->current_state->effort, 1e-10);
}

TEST(InterfaceTests, updateMethodInvalidArgument)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  JointStates current_state = { { "front_right", JointState(1.0, 1.1, 1.2) },
                                { "back_right", JointState(2.0, 2.1, 2.2) },
                                { "front_left", JointState(3.0, 3.1, 3.2) },
                                { "invalid", JointState(4.0, 4.1, 4.2) } };

  ASSERT_THROW(test.update(current_state), std::invalid_argument);
  try
  {
    test.update(current_state);
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
