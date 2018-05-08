#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class OutriggerController
{
public:
  OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc);

  void deploy();
  void retract();
  void update(double dt);

  bool isDeployed();
  bool isRetracted();

  static constexpr double TIME_TO_ACTUATE = .01;

private:
  bool deploying, deployed, retracting, retracted;
  double time_spent;
  iVescAccess *l, *r;
};
