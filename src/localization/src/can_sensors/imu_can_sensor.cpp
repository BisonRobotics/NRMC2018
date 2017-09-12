#include <imu_can_sensor.h>

#define MAXREADVALUE 32768.0f
#define GRAVITY 9.81f
#define TWO 2.0f //+/- range of sensor in g's

IMUCanSensor::IMUCanSensor(int cID, char* interface)
	: CanSensor(cID, interface)
{

}

ReadableSensors::ReadStatus IMUCanSensor::receiveData()
{
	//do a canRecieve,
	if (canReceive(receiveBuffer) ==CanSensor::CanReadStatus::CAN_READ_SUCCESS)
	{
		//recieved data is in the form [X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB]
		//representing 16bits of signed data from -2g to +2g


		//populate ret, this should be correct
		int16_t tempx = receiveBuffer[0] | receiveBuffer[1]<<8; //make an int out of 2 uints
		this->imuData.x = tempx/MAXREADVALUE * TWO * GRAVITY; //convert to m/s^2, tempx/max value * full scale
		int16_t tempy = receiveBuffer[2] | receiveBuffer[3]<<8;
		imuData.y = tempy/MAXREADVALUE * TWO * GRAVITY;
		int16_t tempz = receiveBuffer[4] | receiveBuffer[5]<<8;
		imuData.z = tempz/MAXREADVALUE * TWO * GRAVITY;

		

		//return some sensible coding of how it went
		return ReadableSensors::ReadStatus::READ_SUCCESS;
	}
	else return ReadableSensors::ReadStatus::READ_FAILED;
}

//int IMUCanSensor::setRefreshRate(uint8_t Hz); //future feature