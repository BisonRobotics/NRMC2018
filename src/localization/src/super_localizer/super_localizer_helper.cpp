#include <super_localizer/super_localizer_helper.h>
namespace LocalizerInterface
{
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

LocalizerInterface::stateVector addFromModel(LocalizerInterface::stateVector lhs, LocalizerInterface::stateVector rhs,
                                             float dt, bool imu)
{
  LocalizerInterface::stateVector ret;
  ret.x_pos = lhs.x_pos + rhs.x_vel * dt;
  ret.y_pos = lhs.y_pos + rhs.y_vel * dt;
  ret.theta = lhs.theta + rhs.omega * dt;
  if (imu)
  {
    ret.x_vel = lhs.x_vel + rhs.x_accel * dt;
    ret.y_vel = lhs.y_vel + rhs.y_accel * dt;
    ret.omega = lhs.omega + rhs.alpha * dt;
    ret.alpha = rhs.alpha;
    ret.x_accel = rhs.x_accel;
    ret.y_accel = rhs.y_accel;
  }
  else
  {
    ret.x_vel = rhs.x_vel;
    ret.y_vel = rhs.y_vel;
    ret.omega = rhs.omega;
    ret.alpha = 0.0;
    ret.x_accel = 0.0;
    ret.y_accel = 0.0;
  }
  return ret;
}

LocalizerInterface::stateVector initState(float xi, float yi, float theta)
{
  LocalizerInterface::stateVector ret;
  ret.x_pos = xi;
  ret.y_pos = yi;
  ret.theta = theta;
  ret.omega = 0.0f;
  ret.alpha = 0.0f;
  ret.x_vel = 0.0f;
  ret.y_vel = 0.0f;
  ret.x_accel = 0.0f;
  ret.y_accel = 0.0f;
  return ret;
}
}
