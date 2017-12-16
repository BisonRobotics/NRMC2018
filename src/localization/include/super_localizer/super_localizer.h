#include <localizer/localizer.h>
#include <can_sensors/imu_can_sensor_interface.h>
#include <can_sensors/pos_can_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>
#include <super_localizer/super_localizer_helper.h>

#define XRESGAIN .05f
#define YRESGAIN .05f
#define THETARESGAIN .13f
#define DXRESGAIN .05f
#define DYRESGAIN .05f
#define OMEGARESGAIN .05f

class SuperLocalizer : public LocalizerInterface::LocalizerInterface_c
{
public:
  SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                 iVescAccess *backLeftVesc, ImuCanSensorInterface *centerIMU, PosCanSensorInterface *posSensor,
                 LocalizerInterface::stateVector gains);
  SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                 iVescAccess *backLeftVesc, PosCanSensorInterface *posSensor, LocalizerInterface::stateVector gains);
  UpdateStatus updateStateVector(float dt);
  ~SuperLocalizer();

  //static constexpr Localizer::stateVector_s default_gains;
  LocalizerInterface::stateVector getStateVector();
private:
  Localizer *deadReck;

  ImuCanSensorInterface *cIMU;
  PosCanSensorInterface *pSensor;
  ReadableSensors *sensors[2];
  uint8_t num_sensors;
  bool have_imu;
  bool have_pos;

  LocalizerInterface::stateVector residual;
  LocalizerInterface::stateVector measured;
  LocalizerInterface::stateVector gainVector;

  LocalizerInterface::stateVector state_vector;
};

//I couldn't figure out how to make this a static class member. I tried quite a few things...
//apparently, the declaration is supposed to go in this header and an actual definition in the 
//.cpp file, but I couldn't find the right way to do that... so here it is instead.
static constexpr LocalizerInterface::stateVector SuperLocalizer_default_gains =    {.x_pos = XRESGAIN,
                                                             .y_pos = YRESGAIN,
                                                             .theta = THETARESGAIN,
                                                             .x_vel = DXRESGAIN,
                                                             .y_vel = DYRESGAIN,
                                                             .omega = OMEGARESGAIN,
                                                             .x_accel = 0,
                                                             .y_accel = 0,
                                                             .alpha = 0 };
