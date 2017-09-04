#include <can_sensors.h>
#include <iostream>

int main()
{
	char interface[] = "can0";
	CanSensor mySensor(20, interface);
	uint8_t mybuffer[8];
	int canReturn;

	while(1)
	{
		canReturn = mySensor.canRecieve(mybuffer);
		if (canReturn ==1) std::cout << mybuffer <<std::endl;
		else std::cout << "no message! recieve returned " <<canReturn <<std::endl;
	}
	return 0;
}