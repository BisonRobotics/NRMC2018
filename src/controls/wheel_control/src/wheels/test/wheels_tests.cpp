#include <gtest/gtest.h>
#include "wheel_control/wheels/wheels.h"

using namespace wheel_control;

TEST(TEST_Wheels, Instantiate_JointState)
{
  JointState test(1.1, 1.2, 1.3);

  ASSERT_NEAR(1.1, test.position, 1e-10);
  ASSERT_NEAR(1.2, test.velocity, 1e-10);
  ASSERT_NEAR(1.3, test.effort, 1e-10);
}

TEST(TEST_Wheels, Instantiate_Wheel_With_Name)
{
  Wheel test("Test");

  ASSERT_STREQ("Test", test.name.c_str());

  ASSERT_EQ(-1, test.id);

  ASSERT_NEAR(0.0, test.x_pos, 1e-10);
  ASSERT_NEAR(0.0, test.y_pos, 1e-10);

  ASSERT_NEAR(0.0, test.current_state->position, 1e-10);
  ASSERT_NEAR(0.0, test.current_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, test.current_state->effort, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->position, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->effort, 1e-10);
}

TEST(TEST_Wheels, Instantiate_Wheel_With_Name_And_Wheel_Info)
{
  Wheel test("Test", 1.0, -1.0);

  ASSERT_STREQ("Test", test.name.c_str());

  ASSERT_EQ(-1, test.id);

  ASSERT_NEAR(1.0, test.x_pos, 1e-10);
  ASSERT_NEAR(-1.0, test.y_pos, 1e-10);

  ASSERT_NEAR(0.0, test.current_state->position, 1e-10);
  ASSERT_NEAR(0.0, test.current_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, test.current_state->effort, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->position, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, test.desired_state->effort, 1e-10);
}

TEST(TEST_Wheels, Instantiate_Wheels)
{
  Wheels test;

  ASSERT_STREQ("right_front", test.right_front->name.c_str());
  ASSERT_STREQ("right_back", test.right_back->name.c_str());
  ASSERT_STREQ("left_front", test.left_front->name.c_str());
  ASSERT_STREQ("left_back", test.left_back->name.c_str());
}

TEST(TEST_Wheels, Instantiate_Wheels_With_Params)
{
  Wheels test(1.0, 1.0);

  ASSERT_STREQ("right_front", test.right_front->name.c_str());
  ASSERT_STREQ("right_back", test.right_back->name.c_str());
  ASSERT_STREQ("left_front", test.left_front->name.c_str());
  ASSERT_STREQ("left_back", test.left_back->name.c_str());

  ASSERT_NEAR(0.5, test.right_front->x_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.right_back->x_pos, 1e-10);
  ASSERT_NEAR(0.5, test.left_front->x_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.left_back->x_pos, 1e-10);

  ASSERT_NEAR(0.5, test.right_front->y_pos, 1e-10);
  ASSERT_NEAR(0.5, test.right_back->y_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.left_front->y_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.left_back->y_pos, 1e-10);
}

TEST(TEST_Wheels, Method_get_wheel)
{
  Wheels test;

  ASSERT_STREQ("right_front", test.get_wheel("right_front")->name.c_str());
  ASSERT_STREQ("right_back", test.get_wheel("right_back")->name.c_str());
  ASSERT_STREQ("left_front", test.get_wheel("left_front")->name.c_str());
  ASSERT_STREQ("left_back", test.get_wheel("left_back")->name.c_str());

  ASSERT_THROW(test.get_wheel("invalid"), std::invalid_argument);

  try
  {
    test.get_wheel("invalid");
  }
  catch (std::invalid_argument e)
  {
    ASSERT_STREQ("invalid wheel not found", e.what());
  }
}

TEST(TEST_Wheels, Method_get)
{
  Wheels test;
  std::vector<Wheel *> wheel_vector = test.get();

  ASSERT_STREQ("right_front", wheel_vector[0]->name.c_str());
  ASSERT_STREQ("right_back", wheel_vector[1]->name.c_str());
  ASSERT_STREQ("left_front", wheel_vector[2]->name.c_str());
  ASSERT_STREQ("left_back", wheel_vector[3]->name.c_str());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
