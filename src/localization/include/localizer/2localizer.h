#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <vesc_access.h>
#include <imu_can_sensor.h>
#include <readable_sensors.h>
#include <vector>

class Localizer
{
	public:
		struct stateVector_s {
			float xPos; //in world coordinates
			float yPos; //in world coordinates
			float theta; //robot rotation about its own center with reference to the 
						 //world's x axis and positive angles CCW

			float xVel; //derivitive of xPos
			float yVel; //derivitive of yPos
			float omega;//derivitive of theta
			
			float xAccel;
			float yAccel;
			float alpha;
		} stateVector;

		enum class UpdateStatus {UPDATE_FAILED_SENSOR_ERROR, UPDATE_SUCCESS};

		Localizer(VescAccess * frontLeftVesc, VescAccess * frontRightVesc, VescAccess * backRightVesc, VescAccess * backLeftVesc); //pass wheel linear vel sensors in as FL, FR, BR, BL
		UpdateStatus updateStateVector();
	private:
		float dt;
		float prevtime;
		VescAccess * fleftVesc;
		VescAccess * frightVesc;
		VescAccess * bleftVesc;
		VescAccess * brightVesc;
}

#endif