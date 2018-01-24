#include <sensor_access/pos_sensor_interface.h>

class SimPos : public PosSensorInterface
{
public:
  SimPos(double xnoise1,double ynoise1,double thetanoise1);
  double getX() override;
  double getY() override;
  double getTheta() override;
  ReadableSensors::ReadStatus receiveData() override;

  void update(double x1, double y1, double theta1);
private:
  double x, y, theta;
  double xnoise, ynoise, thetanoise;
};