#include <sensor_access/imu_sensor_interface.h>

class SimImu : public ImuSensorInterface
{
public:
  SimImu(double xnoise1, double ynoise1, double omeganoise1);
  double getX() override;
  double getY() override;
  double getOmega() override;
  tf2::Quaternion getOrientation () override;
  ReadableSensors::ReadStatus receiveData() override;

  void update(double x1, double y1, double omega1);

private:
  double x, y, omega;
  double xnoise, ynoise, omeganoise;
};
