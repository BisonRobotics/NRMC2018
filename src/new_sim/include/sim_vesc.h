#include <vesc_access/ivesc_access.h>

class SimVesc : public iVescAccess
{
  public:
   SimVesc(double Pgain, double Igain);
    void setLinearVelocity(float meters_per_second) override;
    void setTorque(float current) override;
   float getLinearVelocity(void) override;
   float getTorque(void) override;

   void update(double dt); //use gain in PI loop to bring velocity to set vel

  private:
    float vel;
    float setVel;
    double errI;
    double vesc_Pgain;
    double vesc_Igain;
};