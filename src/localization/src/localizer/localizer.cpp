#include <localizer.h>

Localizer::Localizer(VescAccess * frontLeftVesc, VescAccess * frontRightVesc, VescAccess * backRightVesc, VescAccess * backLeftVesc)
{
	stateVector.xPos =0;
	stateVector.yPos =0;
	stateVector.theta =0;

	stateVector.xVel =0;
	stateVector.yVel =0;
	stateVector.omega =0;

	stateVector.xAccel =0;
	stateVector.yAccel =0;
	stateVector alpha =0;

	fleftVesc = frontLeftVesc;
	frightVesc = frontRightVesc;
	brightVesc = backRightVesc;
	bleftVesc = backLeftVesc;
}

Localizer::UpdateStatus Localizer::updateStateVector()
{
	if (false) return Localizer::UpdateStatus::UPDATE_FAILED_SENSOR_ERROR;
	else
	{
		//take current time and subtract it from previous time to get dt
		//multiply dt through forward kinematic model of robot
	}
}