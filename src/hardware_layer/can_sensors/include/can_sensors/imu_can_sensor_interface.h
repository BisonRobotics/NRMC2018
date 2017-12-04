#ifndef IMU_CAN_SENSOR_INTERFACE
#define IMU_CAN_SENSOR_INTERFACE

class ImuCanSensorInterface
{
public:
    float getX();
    float getY();
    float getTheta();
};

#endif