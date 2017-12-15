#include <super_localizer/super_localizer_helper.h>

LocalizerInterface::stateVector diff(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs)
{
  LocalizerInterface::stateVector ret;
  ret.alpha = lhs.alpha - rhs.alpha;
  ret.omega = lhs.omega - rhs.omega;
  ret.theta = lhs.theta - rhs.theta;

  ret.x_accel = lhs.x_accel - rhs.x_accel;
  ret.y_accel = lhs.y_accel - rhs.y_accel;
  ret.x_vel = lhs.x_vel - rhs.x_vel;
  ret.y_vel = lhs.y_vel - rhs.y_vel;
  ret.x_pos = lhs.x_pos - rhs.x_pos;
  ret.y_pos = lhs.y_pos - rhs.y_pos;

  return ret;
}

LocalizerInterface::stateVector multiply(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs)
{
  LocalizerInterface::stateVector ret;
  ret.alpha = lhs.alpha * rhs.alpha;
  ret.omega = lhs.omega * rhs.omega;
  ret.theta = lhs.theta * rhs.theta;
  ret.x_accel = lhs.x_accel * rhs.x_accel;
  ret.y_accel = lhs.y_accel * rhs.y_accel;
  ret.x_vel = lhs.x_vel * rhs.x_vel;
  ret.y_vel = lhs.y_vel * rhs.y_vel;
  ret.x_pos = lhs.x_pos * rhs.x_pos;
  ret.y_pos = lhs.y_pos * rhs.y_pos;

  return ret;
}

LocalizerInterface::stateVector addfrommodel(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs, float dt)
{
  LocalizerInterface::stateVector ret;
  ret.x_pos = lhs.x_pos + rhs.x_vel *dt;
  ret.y_pos = lhs.y_pos + rhs.y_vel *dt;
  ret.theta = lhs.theta + rhs.omega *dt;
  ret.x_vel = rhs.x_vel;
  ret.y_vel = rhs.y_vel;
  ret.omega = rhs.omega;

  return ret;
}