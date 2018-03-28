#ifndef BACKHOE_CONTROLLER_H
#define BACKHOE_CONTROLLER_H

#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

namespace backhoecontroller
{
  typedef struct joint_params {
    double minimum_pos;
    double maximum_pos;
    double safety_check_pos;
    double gain;
    double setpoint_tolerance;
  }joint_params_t;
}

class BackhoeController
{
public:
  BackhoeController(iVescAccess *shoulder_vesc, iVescAccess *wrist_vesc, double wrist_setpoint_tolerance, double shoulder_setpoint_tol,
                    double top_of_wrist_motion, double min_shoulder_angle, double max_shoulder_angle,
                    double shoulder_safety_angle, double wrist_safety_distance, bool in_velocity, double shoulder_gain,
                    double wrist_gain, double min_wrist_distance);

  BackhoeController (backhoecontroller::joint_params_t shoulder_params, backhoecontroller::joint_params_t wrist_params,
                     bool in_velocity, iVescAccess *shoulder_vesc, iVescAccess *wrist_vesc);

  // TODO, return status on update based on operation (see waypoint controller)
  void setShoulderSetpoint(double angle);     // in rad from horizontal
  void setWristSetpoint(double distance);     // in m
  void setShoulderVelocity(double velocity);  // in rad/s
  void setWristVelocity(double velocity);     // in m/s
  void init();
  void update(double dt);
  void tareBucket(void);
  void tareBackhoe(void);
  double getWeightInBucket(void);
  double getWeightInBackhoe(void);
  bool shoulderAtSetpoint();
  bool wristAtSetpoint();
  bool getIsInit (void){return is_init;}
private:
  double shoulder_setpoint;
  double wrist_setpoint;
  double shoulder_angle_estimate;
  double wrist_angle_estimate;
  double bucket_tare_weight;
  double backhoe_tare_weight;
  double wrist_setpoint_tolerance;
  double shoulder_setpoint_tolerance;
  double top_of_wrist_motion;
  double min_shoulder_angle;
  double max_shoulder_angle;
  double shoulder_safety_angle;
  double wrist_safety_distance;
  double shoulder_set_velocity;
  double wrist_set_velocity;
  double shoulder_gain;
  double wrist_gain;
  double min_wrist_distance;
  iVescAccess *shoulder_vesc;
  iVescAccess *wrist_vesc;
  bool is_shoulder_at_setpoint;
  bool is_wrist_at_setpoint;
  bool in_velocity_control_mode;
  bool is_init;
  void safetyCheck();
  void updateWristPosition(double dt);
  void updateShoulderPosition(double dt);
};

#endif
