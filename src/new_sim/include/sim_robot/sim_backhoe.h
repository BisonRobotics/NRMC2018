#include "sim_robot/sim_vesc.h"

class SimBackhoe
{
public:
  SimBackhoe(double shoulderTheta, double wristTheta);
  void update(double dt);
  double getShTheta();
  double getWrTheta();

  iVescAccess *getShoulderVesc();
  iVescAccess *getWristVesc();

private:
  SimVesc *sh;
  SimVesc *wr;

  double shTh;
  double wrTh;
};
