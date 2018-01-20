#ifndef LOCALIZER_INTERFACE_H
#define LOCALIZER_INTERFACE_H

namespace LocalizerInterface
{
typedef struct stateVector_s
{
  double x_pos;  // in world coordinates
  double y_pos;  // in world coordinates
  double theta;  // robot rotation about its own center with reference to the
  // world's x axis and positive angles CCW

  double x_vel;  // derivitive of xPos
  double y_vel;  // derivitive of yPos
  double omega;  // derivitive of theta

  double x_accel;
  double y_accel;
  double alpha;

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

  virtual UpdateStatus updateStateVector(double dt) = 0;
};
}

#endif
