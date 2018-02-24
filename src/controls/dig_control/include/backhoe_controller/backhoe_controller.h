#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class BackhoeController
{
public:
  BackhoeController(double initialShoulderTheta, double initialWristTheta, iVescAccess *shVesc, iVescAccess *wrVesv);

  // TODO, return status on update based on operation (see waypoint controller)
  // Add gains similar to waypoint controller/ localizer

  void setShoulderSetpoint(double angle);
  void setWristSetpoint(double angle);
  void update(double dt);
  bool shoulderAtSetpoint();
  bool wristAtSetpoint();

private:
  double shoulderSetpoint;
  double wristSetpoint;
  double shoulderAngleEst;
  double wristAngleEst;
  iVescAccess *sh, *wr;
  bool isShoulderAtSetpoint;
  bool isWristAtSetpoint;
};
