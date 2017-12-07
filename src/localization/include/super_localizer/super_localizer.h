#include <localizer/localizer.h>
#include <can_sensors/imu_can_sensor_interface.h>
#include <can_sensors/pos_can_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>

#define XRESGAIN .05f
#define YRESGAIN .05f
#define THETARESGAIN .13f
#define DXRESGAIN .05f
#define DYRESGAIN .05f
#define OMEGARESGAIN .05f

class SuperLocalizer : public Localizer
{
public:
  SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                 iVescAccess *backLeftVesc, ImuCanSensorInterface *centerIMU, PosCanSensorInterface *posSensor,
                 Localizer::stateVector_s gains);
  SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                 iVescAccess *backLeftVesc, PosCanSensorInterface *posSensor, Localizer::stateVector_s gains);
  Localizer::UpdateStatus updateStateVector(float dt);

  //static constexpr Localizer::stateVector_s default_gains;

private:
  Localizer deadReck;

  ImuCanSensorInterface *cIMU;
  PosCanSensorInterface *pSensor;
  ReadableSensors *sensors[2];
  uint8_t num_sensors;
  bool have_imu;
  bool have_pos;

  Localizer::stateVector_s residual;
  Localizer::stateVector_s measured;
  Localizer::stateVector_s gainVector;

  Localizer::stateVector_s diff(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs);
  Localizer::stateVector_s multiply(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs);
  Localizer::stateVector_s addfrommodel(Localizer::stateVector_s lhs, Localizer::stateVector_s rhs, float dt);
};

//I couldn't figure out how to make this a static class member. I tried quite a few things...
//apparently, the declaration is supposed to go in this header and an actual definition in the 
//.cpp file, but I couldn't find the right way to do that... so here it is instead.
static constexpr Localizer::stateVector_s SuperLocalizer_default_gains =    {.x_pos = XRESGAIN,
                                                             .y_pos = YRESGAIN,
                                                             .theta = THETARESGAIN,
                                                             .x_vel = DXRESGAIN,
                                                             .y_vel = DYRESGAIN,
                                                             .omega = OMEGARESGAIN,
                                                             .x_accel = 0,
                                                             .y_accel = 0,
                                                             .alpha = 0 };