#include <can_sensors.h>
#include <iostream>

int main()
{
	char interface[] = "can0";
	CanSensor mySensor(20, interface);
	CanSensor mySensor2(21, interface);
	uint8_t mybuffer[8];
	char printbuffer[30];
	int canReturn;

	while(1)
	{
		canReturn = mySensor.canRecieve(mybuffer);
		if (canReturn ==1) 
		{
			sprintf(printbuffer, "message dataS1: %d, %d, %d, %d, %d, %d, %d, %d", mybuffer[0], mybuffer[1], mybuffer[2], mybuffer[3], mybuffer[4], mybuffer[5], mybuffer[6], mybuffer[7]);
			std::cout << printbuffer <<std::endl;
		}
		canReturn = mySensor2.canRecieve(mybuffer);
		if (canReturn ==1) 
		{
			sprintf(printbuffer, "message dataS2: %d, %d, %d, %d, %d, %d, %d, %d", mybuffer[0], mybuffer[1], mybuffer[2], mybuffer[3], mybuffer[4], mybuffer[5], mybuffer[6], mybuffer[7]);
			std::cout << printbuffer <<std::endl;
		}
		else if (canReturn != -1) std::cout << "no message! recieve returned " <<canReturn <<std::endl;
	}
	return 0;
}