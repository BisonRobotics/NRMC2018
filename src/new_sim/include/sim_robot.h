

#include <sim_pos.h>
#include <sim_imu.h>
#include <sim_vesc.h>
#include <localizer/localizer.h>

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

  private:
  iVescAccess * fl;
  iVescAccess * fr;
  iVescAccess * br;
  iVescAccess * bl;

  ImuSensorInterface * imu;
  PosSensorInterface * pos;

  Localizer * deadReck;
}
