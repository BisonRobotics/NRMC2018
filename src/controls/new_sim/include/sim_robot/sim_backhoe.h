#include "sim_robot/sim_vesc.h"

class SimBackhoe
{
public:
  SimBackhoe(double shoulderTheta, double wristTheta);
  SimBackhoe(double shoulderTheta, double wristTheta, double shoulderBottomLimit, double shoulderUpperLimit,
             double wristBottomLimit, double wristUpperLimit);
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
