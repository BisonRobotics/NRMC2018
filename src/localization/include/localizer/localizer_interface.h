#ifndef LOCALIZER_INTERFACE_H
#define LOCALIZER_INTERFACE_H

namespace LocalizerInterface
{
typedef struct stateVector_s
{
  float x_pos;  // in world coordinates
  float y_pos;  // in world coordinates
  float theta;  // robot rotation about its own center with reference to the
  // world's x axis and positive angles CCW

  float x_vel;  // derivitive of xPos
  float y_vel;  // derivitive of yPos
  float omega;  // derivitive of theta

  float x_accel;
  float y_accel;
  float alpha;
} stateVector;

class LocalizerInterface_c
{
public:
  virtual stateVector getStateVector() = 0;
  enum class UpdateStatus
  {
    UPDATE_FAILED_SENSOR_ERROR,
    UPDATE_SUCCESS
  };

  virtual UpdateStatus updateStateVector(float dt) = 0;
};
}

#endif
