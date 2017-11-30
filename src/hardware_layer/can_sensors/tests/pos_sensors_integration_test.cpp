#include <can_sensors/pos_can_sensor.h>
#include <readable_sensors/readable_sensors.h>
#include <iostream>

int main()
{
  char interface[] = "can0";
  POSCanSensor mySensor(40, interface);
  // CanSensor mySensor2(21, interface);
  // uint8_t mybuffer[8];
  char printbuffer[70];
  ReadableSensors::ReadStatus canReturn;

  while (1)
  {
    canReturn = mySensor.receiveData();
    if (canReturn == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      sprintf(printbuffer, "message dataS1: \nX: %f,\nY: %f,\ntheta: %f,\nservoTheta: %f\n\n", mySensor.posData.x,
              mySensor.posData.y, mySensor.posData.theta, mySensor.posData.servoTheta);
      std::cout << printbuffer << std::endl;
    }
    // else if (canReturn == -1) std::cout << "no message! recieve returned " <<canReturn <<std::endl;
  }
  return 0;
}
