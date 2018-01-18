#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
class TeleopInterface
{
public:
  TeleopInterface(float velocity, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);
  TeleopInterface(float velocity);
  float getVelocity();
  void update(float left_vel, float right_vel);
  ~TeleopInterface();
  iVescAccess *fl, *fr, *br, *bl;

private:
  float velocity_scale;

  void setVelocity(float velocity);
  void stopMotors();
  void initializeVars(float velocity);
  bool internally_alloc;
  float clamp(float number, float max, float min);
};
