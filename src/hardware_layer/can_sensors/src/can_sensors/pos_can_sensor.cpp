#include <can_sensors/pos_can_sensor.h>

POSCanSensor::POSCanSensor(int cID, char* interface) : CanSensor(cID, interface)
{
}

ReadableSensors::ReadStatus POSCanSensor::receiveData()
{
  // do a canRecieve,
  if (canReceive(receiveBuffer) == CanSensor::CanReadStatus::CAN_READ_SUCCESS)
  {
    // recieved data is in the form [X MSB, X LSB, Y MSB, Y LSB, Th MSB, Th LSB, sTh MSB, sTH LSB]
    // representing 16bits of data

    posData.x = .001f * (float)((int16_t)((receiveBuffer[0] << 8) | receiveBuffer[1]));
    posData.y = .001f * (float)((int16_t)((receiveBuffer[2] << 8) | receiveBuffer[3]));
    posData.theta = .001f * (float)((int16_t)((receiveBuffer[4] << 8) | receiveBuffer[5]));
    posData.servoTheta = .001f * (float)((int16_t)((receiveBuffer[6] << 8) | receiveBuffer[7]));

    // return some sensible coding of how it went
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  }
  else
    return ReadableSensors::ReadStatus::READ_FAILED;
}

float POSCanSensor::getX()
{
  return posData.x;
}

float POSCanSensor::getY()
{
  return posData.y;
}

float POSCanSensor::getTheta()
{
  return posData.theta;
}

float POSCanSensor::getServoTheta()
{
  return posData.servoTheta;
}