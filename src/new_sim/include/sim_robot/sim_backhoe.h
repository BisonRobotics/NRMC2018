#include "sim_robot/sim_vesc.h"


class SimBackhoe
{
public:
    SimBackhoe(double shoulderTheta, double wristTheta);
    iVescAccess *getShVesc();
    iVescAccess *getWrVesc();

    void update(double dt);
    double getShTheta();
    double getWrTheta();

private:
    SimVesc *sh;
    SimVesc *wr;

    double shTh;
    double wrTh;
};
