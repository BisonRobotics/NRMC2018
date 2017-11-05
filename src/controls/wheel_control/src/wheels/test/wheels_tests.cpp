#include <gtest/gtest.h>
#include "wheel_control/wheels/wheels.h"

using namespace wheel_control;

TEST(WheelsTests, instantiateJointState)
{
  JointState test(1.1, 1.2, 1.3);

  ASSERT_NEAR(1.1, test.position, 1e-10);
  ASSERT_NEAR(1.2, test.velocity, 1e-10);
  ASSERT_NEAR(1.3, test.effort, 1e-10);
}

TEST(WheelsTests, instantiateWheelWithName)
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

TEST(WheelsTests, instantiateWheelWithNameAndWheelInfo)
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

TEST(WheelsTests, instantiateWheels)
{
  Wheels test;

  ASSERT_STREQ("wheel_front_right", test.front_right->name.c_str());
  ASSERT_STREQ("wheel_back_right",  test.back_right->name.c_str());
  ASSERT_STREQ("wheel_front_left",  test.front_left->name.c_str());
  ASSERT_STREQ("wheel_back_left",   test.back_left->name.c_str());
}

TEST(WheelsTests, instantiateWheelsWithParams)
{
  Wheels test(1.0, 1.0);

  ASSERT_STREQ("wheel_front_right", test.front_right->name.c_str());
  ASSERT_STREQ("wheel_back_right",  test.back_right->name.c_str());
  ASSERT_STREQ("wheel_front_left",  test.front_left->name.c_str());
  ASSERT_STREQ("wheel_back_left",   test.back_left->name.c_str());

  ASSERT_NEAR(0.5,  test.front_right->x_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.back_right->x_pos, 1e-10);
  ASSERT_NEAR(0.5,  test.front_left->x_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.back_left->x_pos, 1e-10);

  ASSERT_NEAR(0.5,  test.front_right->y_pos, 1e-10);
  ASSERT_NEAR(0.5,  test.back_right->y_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.front_left->y_pos, 1e-10);
  ASSERT_NEAR(-0.5, test.back_left->y_pos, 1e-10);
}

TEST(WheelsTests, getWheel)
{
  Wheels test;

  ASSERT_STREQ("wheel_front_right", test.get_wheel("wheel_front_right")->name.c_str());
  ASSERT_STREQ("wheel_back_right",  test.get_wheel("wheel_back_right")->name.c_str());
  ASSERT_STREQ("wheel_front_left",  test.get_wheel("wheel_front_left")->name.c_str());
  ASSERT_STREQ("wheel_back_left",   test.get_wheel("wheel_back_left")->name.c_str());

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

TEST(WheelsTests, getWheelVector)
{
  Wheels test;
  std::vector<Wheel *> wheel_vector = test.get();

  ASSERT_STREQ("wheel_front_left",  wheel_vector[0]->name.c_str());
  ASSERT_STREQ("wheel_front_right", wheel_vector[1]->name.c_str());
  ASSERT_STREQ("wheel_back_left",   wheel_vector[2]->name.c_str());
  ASSERT_STREQ("wheel_back_right",  wheel_vector[3]->name.c_str());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
