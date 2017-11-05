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
  this->front_left = new Wheel("front_left");
  this->front_right = new Wheel("front_right");
  this->back_left = new Wheel("back_left");
  this->back_right = new Wheel("back_right");
}

Wheels::Wheels(double x, double y) : Wheels()
{
  set_distance(x, y);
}

void Wheels::set_distance(double x, double y)
{
  this->front_right->x_pos =  x / 2.0;
  this->back_right->x_pos  = -x / 2.0;
  this->front_left->x_pos  =  x / 2.0;
  this->back_left->x_pos   = -x / 2.0;

  this->front_right->y_pos =  y / 2.0;
  this->back_right->y_pos  =  y / 2.0;
  this->front_left->y_pos  = -y / 2.0;
  this->back_left->y_pos   = -y / 2.0;
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
  wheels.push_back(this->front_left);
  wheels.push_back(this->front_right);
  wheels.push_back(this->back_left);
  wheels.push_back(this->back_right);
  return wheels;
}

// TODO figure out way of testing
Wheels::~Wheels()
{
  delete front_right, back_right, front_left, back_left;
}
