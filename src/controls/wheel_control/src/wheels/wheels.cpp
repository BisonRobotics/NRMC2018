#include <string>
#include <stdexcept>
#include "wheel_control/wheels/wheels.h"

using namespace wheel_control;

JointState::JointState() {}
JointState::JointState(double position, double velocity, double effort)
{
  this->position = position;
  this->velocity = velocity;
  this->effort = effort;
}

Wheel::Wheel(std::string name)
{
  this->name = name;
  this->id = -1;
  this->x_pos = 0.0;
  this->y_pos = 0.0;

  this->current_state = new JointState(0.0, 0.0, 0.0);
  this->desired_state = new JointState(0.0, 0.0, 0.0);
}

Wheel::Wheel(std::string name, double x_pos, double y_pos) : Wheel(name)
{
  this->x_pos = x_pos;
  this->y_pos = y_pos;
}

Wheels::Wheels()
{
  this->right_front = new Wheel("right_front");
  this->right_back = new Wheel("right_back");
  this->left_front = new Wheel("left_front");
  this->left_back = new Wheel("left_back");
}

Wheels::Wheels(double x, double y) : Wheels()
{
  set_distance(x, y);
}

void Wheels::set_distance(double x, double y)
{
  this->right_front->x_pos =  x / 2.0;
  this->right_back->x_pos  = -x / 2.0;
  this->left_front->x_pos  =  x / 2.0;
  this->left_back->x_pos   = -x / 2.0;

  this->right_front->y_pos =  y / 2.0;
  this->right_back->y_pos  =  y / 2.0;
  this->left_front->y_pos  = -y / 2.0;
  this->left_back->y_pos   = -y / 2.0;
}

Wheel* Wheels::get_wheel(std::string name)
{
  for (auto wheel : this->get())
  {
    if (wheel->name == name)
    {
      return wheel;
    }
  }
  std::string error_msg = name + " wheel not found";
  throw std::invalid_argument(error_msg);
}

std::vector<Wheel*> Wheels::get()
{
  std::vector<Wheel*> wheels;
  wheels.push_back(this->right_front);
  wheels.push_back(this->right_back);
  wheels.push_back(this->left_front);
  wheels.push_back(this->left_back);
  return wheels;
}

// TODO figure out way of testing
Wheels::~Wheels()
{
  delete right_front, right_back, left_front, left_back;
}
