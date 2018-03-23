#include <backhoe_controller/backhoe_controller.h>
#include <cmath>

BackhoeController::BackhoeController(iVescAccess *sh_vesc, iVescAccess *wr_vesc, double wrist_setpoint_tolerance,
                                     double shoulder_setpoint_tol, double top_of_wrist_motion, double min_backhoe_angle,
                                     double max_backhoe_angle, double shoulder_safety_angle,
                                     double wrist_safety_distance, bool in_velocity, double shoulder_gain,
                                     double wrist_gain, double min_wrist_distance)
{
  this->shoulder_setpoint = 0.0;
  this->shoulder_angle_estimate = 0.0;
  this->is_shoulder_at_setpoint = true;
  this->shoulder_vesc = sh_vesc;
  this->wrist_setpoint = 0.0;
  this->wrist_angle_estimate = 0.0;
  this->is_wrist_at_setpoint = true;
  this->wrist_vesc = wr_vesc;
  this->bucket_tare_weight = 0;
  this->backhoe_tare_weight = 0;
  this->wrist_setpoint_tolerance = wrist_setpoint_tolerance;
  this->shoulder_setpoint_tolerance = shoulder_setpoint_tol;
  this->top_of_wrist_motion = top_of_wrist_motion;
  this->min_shoulder_angle = min_backhoe_angle;
  this->max_shoulder_angle = max_backhoe_angle;
  this->shoulder_safety_angle = shoulder_safety_angle;
  this->wrist_safety_distance = wrist_safety_distance;
  this->in_velocity_control_mode = in_velocity;
  this->shoulder_gain = shoulder_gain;
  this->wrist_gain = wrist_gain;
  this->shoulder_set_velocity = 0.0;
  this->wrist_set_velocity = 0.0;
  this->min_wrist_distance = min_wrist_distance;
  this->is_init = false;
}

BackhoeController::BackhoeController(backhoecontroller::joint_params_t shoulder_params,
                                     backhoecontroller::joint_params_t wrist_params, bool in_velocity,
                                     iVescAccess *shoulder_vesc, iVescAccess *wrist_vesc)
: BackhoeController (shoulder_vesc, wrist_vesc, wrist_params.setpoint_tolerance, shoulder_params.setpoint_tolerance,
                     wrist_params.maximum_pos, shoulder_params.minimum_pos, shoulder_params.maximum_pos,
                     shoulder_params.safety_check_pos, wrist_params.safety_check_pos, in_velocity, shoulder_params.gain,
                     wrist_params.gain, wrist_params.minimum_pos)
{

}



void BackhoeController::setShoulderSetpoint(double angle)
{
  if (is_init) {
    shoulder_setpoint = angle;
  }
}

void BackhoeController::setWristSetpoint(double angle)
{
  if (is_init) {
    wrist_setpoint = angle;
  }
}

void BackhoeController::setShoulderVelocity(double velocity)
{
  if (is_init) {
    shoulder_set_velocity = velocity;
  }
}

void BackhoeController::setWristVelocity(double velocity)
{
  if (is_init) {
    wrist_set_velocity = velocity;
  }
}

void BackhoeController::update(double dt)
{
  if (is_init) {
    updateShoulderPosition(dt);
    updateWristPosition(dt);

    if (!in_velocity_control_mode) {
      wrist_set_velocity = wrist_setpoint - wrist_angle_estimate;
      shoulder_set_velocity = shoulder_setpoint - shoulder_angle_estimate;
      is_shoulder_at_setpoint = (fabs(shoulder_set_velocity) < fabs(shoulder_setpoint_tolerance));
      is_wrist_at_setpoint = (fabs(wrist_set_velocity) < fabs(wrist_setpoint_tolerance));
    }

    safetyCheck();
    wrist_vesc->setLinearVelocity(wrist_gain * wrist_set_velocity);
    shoulder_vesc->setLinearVelocity(shoulder_gain * shoulder_set_velocity);
  }
}

void BackhoeController::safetyCheck()
{
  if (shoulder_set_velocity < 0 && shoulder_angle_estimate < shoulder_safety_angle &&
      wrist_angle_estimate > wrist_safety_distance)
  {
    shoulder_set_velocity = 0.0;
    if (wrist_set_velocity > 0)
    {
      wrist_set_velocity = 0.0;
    }
  }

  if (shoulder_set_velocity < 0)
  {
    if (shoulder_angle_estimate < min_shoulder_angle)
    {
        shoulder_set_velocity = 0;
    }
  }
  else if (shoulder_set_velocity > 0)
  {
      if (shoulder_angle_estimate > max_shoulder_angle)
      {
          shoulder_set_velocity = 0;
      }
  }

  if (wrist_set_velocity > 0)
  {
    if (wrist_angle_estimate > top_of_wrist_motion)
    {
      wrist_set_velocity = 0;
    }
  }
  else if (wrist_set_velocity < 0)
  {
    if (wrist_angle_estimate < min_wrist_distance)
    {
      wrist_set_velocity = 0;
    }
  }

}

void BackhoeController::init()
{
  bool at_home = false;
  static constexpr float drive_torque = -1.0f;
  wrist_vesc->setTorque (drive_torque);
  while (wrist_vesc->getLimitSwitchState() != nsVescAccess::limitSwitchState::bottomOfMotion);
  wrist_vesc->setLinearVelocity(0.0);
  wrist_angle_estimate = min_wrist_distance;
  wrist_setpoint = wrist_angle_estimate;
  shoulder_angle_estimate = shoulder_vesc->getPotPosition();
  shoulder_setpoint = shoulder_angle_estimate;
  is_init = true;
}

void BackhoeController::updateWristPosition(double dt)
{
  if (wrist_vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    wrist_angle_estimate = 0;
  }
  else if (wrist_vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion)
  {
    wrist_angle_estimate = top_of_wrist_motion;
  }
  else
  {
    wrist_angle_estimate += wrist_vesc->getLinearVelocity() * dt;
  }
}

void BackhoeController::updateShoulderPosition(double dt)
{
  if (this->shoulder_vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    shoulder_angle_estimate = min_shoulder_angle;
  }
  else if (shoulder_vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion)
  {
    shoulder_angle_estimate = max_shoulder_angle;
  }
  else
  {
    shoulder_angle_estimate = shoulder_vesc->getPotPosition();  // transform this if need be
  }
}

bool BackhoeController::shoulderAtSetpoint()
{
  return is_shoulder_at_setpoint;
}

bool BackhoeController::wristAtSetpoint()
{
  return is_wrist_at_setpoint;
}

double BackhoeController::getWeightInBackhoe()
{
  // calculate weight
  return backhoe_tare_weight;
}

double BackhoeController::getWeightInBucket()
{
  return bucket_tare_weight;
}

void BackhoeController::tareBackhoe()
{
  backhoe_tare_weight = 0;
}

void BackhoeController::tareBucket()
{
  bucket_tare_weight = 0;
}