#ifndef PROJECT_DIFFERENTIALDRIVECONTROLLER_H
#define PROJECT_DIFFERENTIALDRIVECONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include "wheel_control/velocity_interface/velocity_interface.h"

namespace wheel_control
{
class DifferentialDriveController : public VelocityInterface
{
public:
  DifferentialDriveController();

  void set_velocity(double lin_vel, double ang_vel) override;

  // This controller doesn't rely upon the current state of the wheels, so it doesn't need to update every cycle
  void update_desired_state() {};
};
}

PLUGINLIB_EXPORT_CLASS(wheel_control::DifferentialDriveController, wheel_control::VelocityInterface)

#endif  // PROJECT_DIFFERENTIALDRIVECONTROLLER_H
