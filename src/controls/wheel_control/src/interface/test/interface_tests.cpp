#include <gtest/gtest.h>
#include "wheel_control/interface/interface.h"

using namespace wheel_control;

TEST(TEST_Interface, Load_Interface)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  ASSERT_STREQ("right_front", test.wheels->right_front->name.c_str());
  ASSERT_STREQ("right_back", test.wheels->right_back->name.c_str());
  ASSERT_STREQ("left_front", test.wheels->left_front->name.c_str());
  ASSERT_STREQ("left_back", test.wheels->left_back->name.c_str());
}

TEST(TEST_Interface, Method_update)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  JointStates current_state = { { "right_front", JointState(1.0, 1.1, 1.2) },
                                { "right_back", JointState(2.0, 2.1, 2.2) },
                                { "left_front", JointState(3.0, 3.1, 3.2) },
                                { "left_back", JointState(4.0, 4.1, 4.2) } };
  test.update(current_state);

  ASSERT_NEAR(1.0, test.wheels->right_front->current_state->position, 1e-10);
  ASSERT_NEAR(2.0, test.wheels->right_back->current_state->position, 1e-10);
  ASSERT_NEAR(3.0, test.wheels->left_front->current_state->position, 1e-10);
  ASSERT_NEAR(4.0, test.wheels->left_back->current_state->position, 1e-10);

  ASSERT_NEAR(1.1, test.wheels->right_front->current_state->velocity, 1e-10);
  ASSERT_NEAR(2.1, test.wheels->right_back->current_state->velocity, 1e-10);
  ASSERT_NEAR(3.1, test.wheels->left_front->current_state->velocity, 1e-10);
  ASSERT_NEAR(4.1, test.wheels->left_back->current_state->velocity, 1e-10);

  ASSERT_NEAR(1.2, test.wheels->right_front->current_state->effort, 1e-10);
  ASSERT_NEAR(2.2, test.wheels->right_back->current_state->effort, 1e-10);
  ASSERT_NEAR(3.2, test.wheels->left_front->current_state->effort, 1e-10);
  ASSERT_NEAR(4.2, test.wheels->left_back->current_state->effort, 1e-10);
}

TEST(TEST_Interface, Method_update_invalid)
{
  Interface test;
  auto *wheels = new Wheels();
  test.load(wheels);

  JointStates current_state = { { "right_front", JointState(1.0, 1.1, 1.2) },
                                { "right_back", JointState(2.0, 2.1, 2.2) },
                                { "left_front", JointState(3.0, 3.1, 3.2) },
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
