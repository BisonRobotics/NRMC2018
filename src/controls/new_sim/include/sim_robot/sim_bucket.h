#include "sim_robot/sim_vesc.h"

class SimBucket
{
public:
  SimBucket();
  iVescAccess *getBigConveyorVesc();
  iVescAccess *getLittleConveyorVesc();
  iVescAccess *getSifterVesc();

  void update(double dt);

private:
  SimVesc *bc;
  SimVesc *lc;
  SimVesc *sf;
};