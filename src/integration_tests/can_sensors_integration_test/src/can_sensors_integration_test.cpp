#include <imu_can_sensor.h>
#include <readable_sensors.h>
#include <iostream>

int main()
{
	char interface[] = "can0";
	IMUCanSensor mySensor(20, interface);
	//CanSensor mySensor2(21, interface);
	//uint8_t mybuffer[8];
	char printbuffer[30];
	ReadableSensors::ReadStatus canReturn;

	while(1)
	{
		canReturn = mySensor.receiveData();
		if (canReturn ==ReadableSensors::ReadStatus::READ_SUCCESS) 
		{
			sprintf(printbuffer, "message dataS1: \nX: %f,\nY: %f,\nZ: %f\n\n", mySensor.imuData.x, mySensor.imuData.y, mySensor.imuData.z);
			std::cout << printbuffer <<std::endl;
		}
		//else if (canReturn == -1) std::cout << "no message! recieve returned " <<canReturn <<std::endl;
	}
	return 0;
}