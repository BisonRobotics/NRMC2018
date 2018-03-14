
#ifndef PROJECT_TELEOP_SAFETY_H
#define PROJECT_TELEOP_SAFETY_H

#include "vesc_access/vesc_access.h"

class TeleopSafety
{
public:
  TeleopSafety ();
  ~TeleopSafety();
  void moveLinear (float effort);
  void moveShoulder (float effort);
  void toggleSifter (void);
  void toggleLargeConveyor (void);
  void moveOutriggers (float effort);
private:
  static constexpr float min_shoulder_angle =-.1f;
  static constexpr float max_shoulder_angle = 1.159f;
  static constexpr float last_angle_that_backhoe_can_be_retracted= 1.0f;
  static constexpr float large_conveyor_velocity=10.0f;
  static constexpr float small_conveyor_velocity=10.0f;
  static constexpr float sifter_velocity = 10.0f;
  static constexpr float deadzone=.1;
  static constexpr float linear_scaling = 1.0f;
  static constexpr float shoulder_scaling = .01f;
  static constexpr float outrigger_scaling = 1.0f;
  iVescAccess *shoulder_vesc;
  iVescAccess *linear_vesc;
  iVescAccess *large_conveyor_vesc;
  iVescAccess *small_conveyor_vesc;
  iVescAccess *sifter_vesc;
  iVescAccess *left_outrigger;
  iVescAccess *right_outrigger;
  bool sifter_state;
  bool large_conveyor_state;
};

#endif //PROJECT_TELEOP_SAFETY_H
