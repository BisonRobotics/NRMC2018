#ifndef PROJECT_WHEELS_H
#define PROJECT_WHEELS_H

#include <vector>
#include <map>

namespace wheel_control
{
class JointState
{
public:
  JointState();
  JointState(double position, double velocity, double effort);

  double position;
  double velocity;
  double effort;
};
typedef std::map<std::string, JointState> JointStates;

class Wheel
{
public:
  Wheel(std::string name);
  Wheel(std::string name, double x_pos, double y_pos);

  std::string name;
  int id;
  double x_pos, y_pos;
  JointState *current_state, *desired_state;
};

class Wheels
{
public:
  Wheels();
  Wheels(double x, double y);
  ~Wheels();

  Wheel *get_wheel(std::string name);
  std::vector<Wheel *> get();
  Wheel *right_front, *right_back, *left_front, *left_back;
  void set_distance(double x, double y);
};
}

#endif  // PROJECT_WHEELS_H
