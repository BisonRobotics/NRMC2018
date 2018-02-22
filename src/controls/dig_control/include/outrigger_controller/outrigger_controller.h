#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class OutriggerController
{
  OutriggerController(iVescAccess *lVesc, iVescAccess *rVesc);

  void deploy();
  void retract();
  void update(double dt);

  bool isDeployed();
  bool isRetracted();
private:
  bool isDeployed, isRetracted;
  iVescAccess *l, *r;
}