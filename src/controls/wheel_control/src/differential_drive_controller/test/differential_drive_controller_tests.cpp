#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include "wheel_control/velocity_interface/velocity_interface.h"

using namespace wheel_control;

TEST(TEST_DifferentialDriveControllerPlugin, Load_Plugin)
{
  pluginlib::ClassLoader<VelocityInterface> drive_loader("wheel_control", "wheel_control::VelocityInterface");
  boost::shared_ptr<VelocityInterface> diff_drive =
      drive_loader.createInstance("wheel_control::DifferentialDriveController");

  Wheels *wheels = new Wheels(1.0, 1.0);
  diff_drive->load(wheels);
  diff_drive->set_velocity(1.0, 0.0);

  ASSERT_NEAR(1.0, diff_drive->wheels->right_front->desired_state->velocity, 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->right_back->desired_state->velocity, 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->left_front->desired_state->velocity, 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->left_back->desired_state->velocity, 1e-10);

  diff_drive->set_velocity(1.0, 1.0);

  ASSERT_NEAR(2.0, diff_drive->wheels->right_front->desired_state->velocity, 1e-10);
  ASSERT_NEAR(2.0, diff_drive->wheels->right_back->desired_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, diff_drive->wheels->left_front->desired_state->velocity, 1e-10);
  ASSERT_NEAR(0.0, diff_drive->wheels->left_back->desired_state->velocity, 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
