#include <imu_can_sensor.h>
#include <iostream>

int main()
{
	char interface[] = "can0";
	IMUCanSensor mySensor(20, interface);
	//CanSensor mySensor2(21, interface);
	//uint8_t mybuffer[8];
	IMUCanSensor::imuData myData;
	char printbuffer[30];
	int canReturn;

	while(1)
	{
		canReturn = mySensor.recieveData(&myData);
		if (canReturn ==1) 
		{
			sprintf(printbuffer, "message dataS1: \nX: %f,\nY: %f,\nZ: %f\n\n", myData.x, myData.y, myData.z);
			std::cout << printbuffer <<std::endl;
		}
		//else if (canReturn == -1) std::cout << "no message! recieve returned " <<canReturn <<std::endl;
	}
	return 0;
}