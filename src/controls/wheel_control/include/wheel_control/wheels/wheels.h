#ifndef PROJECT_WHEELS_H
#define PROJECT_WHEELS_H

#include <vector>
#include <map>
#include <sensor_msgs/JointState.h>

namespace wheel_control
{

const int FLI = 0;
const int FRI = 1;
const int BLI = 2;
const int BRI = 3;

class Wheels
{
public:

  Wheels();
  Wheels(double x, double y);

  std::vector<std::string> name;
  std::vector<double> x_pos, y_pos;
  std::vector<int> id;
  sensor_msgs::JointState current_state, desired_state;

  virtual void set_position(int index, double position) = 0;
  virtual void set_velocity(int index, double velocity) = 0;
  virtual void set_effort(int index, double effort) = 0;
  virtual double get_position(int index) = 0;
  virtual double get_velocity(int index) = 0;
  virtual double get_effort(int index) = 0;

  void set_distance(double x, double y);
  void update_current_state();
};
}

#endif  // PROJECT_WHEELS_H
