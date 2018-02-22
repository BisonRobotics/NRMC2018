#include <vesc_access/ivesc_access.h>

class SimVesc : public iVescAccess
{
public:
  SimVesc(double Pgain, double Igain, double velo);
  void setLinearVelocity(float meters_per_second) override;
  void setTorque(float current) override;
  float getLinearVelocity(void) override;
  float getTorque(void) override;

  void update(double dt);  // use gain in PI loop to bring velocity to set vel
  nsVescAccess::limitSwitchState getLimitSwitchState (void);
private:
  float vel;
  float setVel;
  double errI;
  double vesc_Pgain;
  double vesc_Igain;
  double velocity_factor;
};
