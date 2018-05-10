#include <sensor_access/pos_sensor_interface.h>

class SimPos : public PosSensorInterface
{
public:
  SimPos(double xnoise1, double ynoise1, double thetanoise1);
  double getX() override;
  double getY() override;
  double getTheta() override;
  bool isFloating() override;
  ReadableSensors::ReadStatus receiveData() override;
  double getZ() override {return 0;}
  void update(double x1, double y1, double theta1);
  void setIsFloating(bool is_floating);

private:
  double x, y, theta;
  double xnoise, ynoise, thetanoise;
  bool is_floating;
};