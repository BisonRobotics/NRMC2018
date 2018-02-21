#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class BackhoeController
{
    BackhoeController(double initialShoulderTheta, double initialWristTheta, iVescAccess *shVesc, iVescAccess *wrVesv);

    //TODO, return status on update based on operation (see waypoint controller)
    //Add gains similar to waypoint controller/ localizer

    void setShoulderSetPoint(double angle);
    void setWristSetPoint(double angle);
    void update(double dt);

private:
    double shoulderSetPoint;
    double wristSetPoint;
    double shoulderAngleEst;
    double wristAngleEst;
    iVescAccess *sh, *wr;
};
