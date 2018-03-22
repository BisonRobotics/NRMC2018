#include <backhoe_controller/backhoe_controller.h>
#include <cmath>

BackhoeController::BackhoeController(double initial_shoulder_theta, double initial_wrist_theta, iVescAccess *sh_vesc,
                                     iVescAccess *wr_vesc, double wrist_setpoint_tolerance, double shoulder_setpoint_tol,
                                      double top_of_wrist_motion, double min_backhoe_angle, double max_backhoe_angle,
                                     double shoulder_safety_angle, double wrist_safety_distance, bool in_velocity,
                                      double shoulder_gain, double wrist_gain)
{
  this->shoulder_setpoint = initial_shoulder_theta;
  this->shoulder_angle_estimate = initial_shoulder_theta;
  this->is_shoulder_at_setpoint = true;
  this->shoulder_vesc = sh_vesc;
  this->wrist_setpoint = initial_wrist_theta;
  this->wrist_angle_estimate = initial_wrist_theta;
  this->is_wrist_at_setpoint = true;
  this->wrist_vesc = wr_vesc;
  this->bucket_tare_weight = 0;
  this->backhoe_tare_weight = 0;
  this->wrist_setpoint_tolerance = wrist_setpoint_tolerance;
  this->shoulder_setpoint_tolerance = shoulder_setpoint_tol;
  this->top_of_wrist_motion = top_of_wrist_motion;
  this->min_backhoe_angle = min_backhoe_angle;
  this->max_backhoe_angle = max_backhoe_angle;
  this->shoulder_safety_angle = shoulder_safety_angle;
  this->wrist_safety_angle = wrist_safety_distance;
  this->in_velocity = in_velocity;
  this->shoulder_gain = shoulder_gain;
  this->wrist_gain = wrist_gain;
}

void BackhoeController::setShoulderSetpoint(double angle)
{
  shoulder_setpoint = angle;
}

void BackhoeController::setWristSetpoint(double angle)
{
  wrist_setpoint = angle;
}

void BackhoeController::setShoulderVelocity(double velocity)
{
  shoulder_set_velocity = velocity;
}

void BackhoeController::setWristVelocity(double velocity)
{
  wrist_set_velocity = velocity;
}

void BackhoeController::update(double dt)
{
  UpdateShoulderPosition(dt);
  UpdateWristPosition(dt);

  if (!in_velocity)
  {
    wrist_set_velocity = wrist_setpoint - wrist_angle_estimate;
    shoulder_set_velocity = shoulder_setpoint - shoulder_angle_estimate;
    is_shoulder_at_setpoint = (fabs(shoulder_set_velocity) < fabs(shoulder_setpoint_tolerance));
    is_wrist_at_setpoint = (fabs(wrist_set_velocity) < fabs(wrist_setpoint_tolerance));
  }

  SafetyCheck ();
  wrist_vesc->setLinearVelocity(wrist_gain * wrist_set_velocity);
  shoulder_vesc->setLinearVelocity(shoulder_gain * shoulder_set_velocity);
}

void BackhoeController::SafetyCheck ()
{
  if (shoulder_set_velocity > 0 && shoulder_angle_estimate > shoulder_safety_angle && wrist_angle_estimate < wrist_safety_angle)
  {
      shoulder_set_velocity = 0.0;
  }
}

void BackhoeController::UpdateWristPosition(double dt)
{
  if (wrist_vesc->getLimitSwitchState()==nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    wrist_angle_estimate = 0;
  }
  else if (wrist_vesc->getLimitSwitchState()==nsVescAccess::limitSwitchState::topOfMotion)
  {
    wrist_angle_estimate = top_of_wrist_motion;
  }
  else
  {
    wrist_angle_estimate += wrist_vesc->getLinearVelocity()*dt;
  }
}

void BackhoeController::UpdateShoulderPosition(double dt)
{
  if (this->shoulder_vesc->getLimitSwitchState()==nsVescAccess::limitSwitchState::bottomOfMotion)
  {
    shoulder_angle_estimate = min_backhoe_angle;
  }
  else if (shoulder_vesc->getLimitSwitchState()==nsVescAccess::limitSwitchState::topOfMotion)
  {
    shoulder_angle_estimate = max_backhoe_angle;
  }
  else
  {
    shoulder_angle_estimate = shoulder_vesc->getPotPosition(); // transform this if need be
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

void BackhoeController::tareBucket ()
{
  bucket_tare_weight = 0;
}