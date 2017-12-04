#ifndef POS_CAN_SENSOR_H
#define POS_CAN_SENSOR_H

#include <can_sensors/can_sensors.h>
#include <can_sensors/pos_can_sensor_interface.h>
#include <readable_sensors/readable_sensors.h>

class POSCanSensor : public CanSensor, public ReadableSensors, public PosCanSensorInterface
{
public:
  POSCanSensor(int cID, char* interface);

  ReadableSensors::ReadStatus receiveData();

  float getX();
  float getY();
  float getTheta();
  float getServoTheta();

private:
  uint8_t receiveBuffer[8];
  struct data_s
  {
    float x;
    float y;
    float theta;
    float servoTheta;
  } posData;
};

#endif
