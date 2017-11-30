#ifndef POS_CAN_SENSOR_H
#define POS_CAN_SENSOR_H

#include <can_sensors/can_sensors.h>
#include <readable_sensors/readable_sensors.h>

class POSCanSensor : public CanSensor, public ReadableSensors
{
public:
    struct data_s
    {
        float x;
        float y;
        float theta;
        float servoTheta;
    } posData;

    POSCanSensor(int cID, char* interface);

    ReadableSensors::ReadStatus receiveData();

private:
    uint8_t receiveBuffer[8];
};

#endif
