#ifndef POS_CAN_SENSOR_INTERFACE
#define POS_CAN_SENSOR_INTERFACE

class PosCanSensorInterface
{
public:
  virtual float getX() = 0;
  virtual float getY() = 0;
  virtual float getTheta() = 0;
  virtual float getServoTheta() = 0;
};

#endif