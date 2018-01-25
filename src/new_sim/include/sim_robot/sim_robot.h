#include "sim_robot/sim_pos.h"
#include "sim_robot/sim_imu.h"
#include "sim_robot/sim_vesc.h"
#include "localizer/localizer.h"

class SimRobot
{
  public:
  SimRobot(double axelLen, double xi, double yi ,double thi);

  iVescAccess * getFLVesc();
  iVescAccess * getFRVesc();
  iVescAccess * getBRVesc();
  iVescAccess * getBLVesc();

  ImuSensorInterface * getImu();
  PosSensorInterface * getPos();

  void update(double dt);
  double getX();
  double getY();
  double getTheta();

  private:
  SimVesc * fl;
  SimVesc * fr;
  SimVesc * br;
  SimVesc * bl;

  SimImu * imu;
  SimPos * pos;

  Localizer * deadReck;

  double prevXVel, prevYVel;
};