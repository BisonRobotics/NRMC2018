#include <imu_can_sensor.h>

IMUCanSensor::IMUCanSensor(int cID, char* interface)
	: CanSensor(cID, interface)
{

}

int IMUCanSensor::recieveData(void * ret)
{
	//do a canRecieve,
	if (canRecieve(recieveBuffer) ==1)
	{

		//recieved data is in the form [X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB]
		//representing 16bits of signed data from -2g to +2g
	
		imuData *realret;
		realret = (imuData*)ret;

		//populate ret, this isn't accurate (correct) yet
		int tempx = recieveBuffer[0] | recieveBuffer[1]<<8; //make an int out of 2 uints
		realret->x = tempx/32768.0 * 2.0 * 9.8; //convert to m/s^2, tempx/max value * full scale
		int tempy = recieveBuffer[2] | recieveBuffer[3]<<8;
		realret->y = tempy/32768.0 * 2.0 * 9.8;
		int tempz = recieveBuffer[4] | recieveBuffer[5]<<8;
		realret->z = tempz/32768.0 * 2.0 * 9.8;

		//return some sensible coding of how it went
		return 1;
	}
	else return -1;
}

//int IMUCanSensor::setRefreshRate(uint8_t Hz); //future feature