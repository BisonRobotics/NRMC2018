#include "sim_robot/sim_vesc.h"

class SimOutriggers
{
public:
  SimOutriggers(double initialPosL, double initialPosR);
  iVescAccess *getLVesc();
  iVescAccess *getRVesc();

  void update(double dt);
  double getPosL();
  double getPosR();

private:
  SimVesc *l;
  SimVesc *r;

  double posL, posR;
};