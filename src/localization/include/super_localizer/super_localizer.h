#include <localizer/localizer.h>
#include <can_sensors/imu_can_sensor.h>
#include <can_sensors/pos_can_sensor.h>
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
    SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, IMUCanSensor *centerIMU, POSCanSensor* posSensor, Localizer::stateVector_s gains);
    SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, POSCanSensor* posSensor, Localizer::stateVector_s gains);
	Localizer::UpdateStatus updateStateVector(float dt);

    static constexpr Localizer::stateVector_s default_gains = {
                                                     .x_pos = XRESGAIN, .y_pos = YRESGAIN, .theta = THETARESGAIN,
                                                     .x_vel = DXRESGAIN, .y_vel = DYRESGAIN, .omega = OMEGARESGAIN,
                                                     .x_accel =0, .y_accel = 0, .alpha =0 };
private:
    Localizer deadReck;

    IMUCanSensor * cIMU;
    POSCanSensor * pSensor;
    ReadableSensors* sensors[2];
    uint8_t num_sensors;
    bool have_imu;
	bool have_pos;

    Localizer::stateVector_s residual;
    Localizer::stateVector_s measured;
    Localizer::stateVector_s gainVector;

    Localizer::stateVector_s diff(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs);
    Localizer::stateVector_s multiply(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs);
};
