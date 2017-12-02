#include <localizer/localizer.h>
#include <can_sensors/imu_can_sensor.h>
#include <can_sensors/pos_can_sensor.h>
#include <readable_sensors/readable_sensors.h>

#define XRESGAIN .05
#define YRESGAIN .05
#define THETARESGAIN .13
#define DXRESGAIN .05
#define DYRESGAIN .05
#define OMEGARESGAIN .05

class SuperLocalizer : public Localizer
{
public:
    SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, IMUCanSensor *centerIMU, POSCanSensor* posSensor);
    SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, POSCanSensor* posSensor);
	Localizer::UpdateStatus updateStateVector(float dt);
private:
    Localizer deadReck;
    IMUCanSensor * cIMU;
    POSCanSensor * pSensor;
    ReadableSensors* sensors[2];

    Localizer::stateVector_s residual;
    Localizer::stateVector_s measured;
    Localizer::stateVector_s gainVector;

    Localizer::stateVector_s diff(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs);
    Localizer::stateVector_s multiply(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs);

    uint8_t num_sensors;
    bool have_imu;
	bool have_pos;
};
