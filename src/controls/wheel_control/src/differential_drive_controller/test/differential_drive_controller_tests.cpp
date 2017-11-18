#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <wheel_control/velocity_interface/velocity_interface.h>
#include <wheel_control/wheels/test_wheels.h>

using namespace wheel_control;

TEST(DifferentialDriveControllerTests, loadPlugin)
{
  pluginlib::ClassLoader<VelocityInterface> drive_loader("wheel_control", "wheel_control::VelocityInterface");
  boost::shared_ptr<VelocityInterface> diff_drive =
      drive_loader.createInstance("wheel_control::DifferentialDriveController");

  TestWheels wheels(1.0, 1.0);
  diff_drive->load(&wheels);
  diff_drive->setVelocity(1.0, 0.0);

  ASSERT_NEAR(1.0, diff_drive->wheels->desired_state.velocity[FLI], 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->desired_state.velocity[FRI], 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->desired_state.velocity[BLI], 1e-10);
  ASSERT_NEAR(1.0, diff_drive->wheels->desired_state.velocity[BRI], 1e-10);

  diff_drive->setVelocity(1.0, 1.0);

  ASSERT_NEAR(0.0, diff_drive->wheels->desired_state.velocity[FLI], 1e-10);
  ASSERT_NEAR(2.0, diff_drive->wheels->desired_state.velocity[FRI], 1e-10);
  ASSERT_NEAR(0.0, diff_drive->wheels->desired_state.velocity[BLI], 1e-10);
  ASSERT_NEAR(2.0, diff_drive->wheels->desired_state.velocity[BRI], 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
