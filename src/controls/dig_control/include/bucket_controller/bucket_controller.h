#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class BucketController
{
public:
  BucketController(iVescAccess *bigConveyorVesc, iVescAccess *littleConveyorVesc, iVescAccess *sifterVesc);

  void turnBigConveyorOn();
  void turnBigConveyorOff();
  void turnLittleConveyorOn();
  void turnLittleConveyorOff();
  void turnSifterOn();
  void turnSifterOff();

private:
  iVescAccess *bc, *lc, *sf;
};