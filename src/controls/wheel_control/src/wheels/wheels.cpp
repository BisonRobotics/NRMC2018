#include <string>
#include <wheel_control/wheels/wheels.h>

using namespace wheel_control;

Wheels::Wheels()
{
  this->name.push_back("wheel_front_left");
  this->name.push_back("wheel_front_right");
  this->name.push_back("wheel_back_left");
  this->name.push_back("wheel_back_right");

  for (int i = 0; i < 4; i++)
  {
    // TODO Initialize header
    this->id.push_back(-1);
    this->x_pos.push_back(0.0);
    this->y_pos.push_back(0.0);
    this->current_state.name.push_back(name[i]); // TODO Add test
    this->current_state.position.push_back(0.0);
    this->current_state.velocity.push_back(0.0);
    this->current_state.effort.push_back(0.0);
    this->desired_state.name.push_back(name[i]); // TODO Add test
    this->desired_state.position.push_back(0.0);
    this->desired_state.velocity.push_back(0.0);
    this->desired_state.effort.push_back(0.0);
  }

  set_distance(1.0, 1.0);
}

Wheels::Wheels(double x, double y) : Wheels()
{
  set_distance(x, y);
}

void Wheels::set_distance(double x, double y)
{
  for (int i = 0; i < 4; i++)
  {
    if (name[i] == "wheel_front_left")
    {
      x_pos[i] =  x / 2.0;
      y_pos[i] =  y / 2.0;
      continue;
    }
    if (name[i] == "wheel_front_right")
    {

      x_pos[i] =  x / 2.0;
      y_pos[i] = -y / 2.0;
      continue;
    }
    if (name[i] == "wheel_back_left")
    {
      x_pos[i] = -x / 2.0;
      y_pos[i] =  y / 2.0;

      continue;
    }
    if (name[i] == "wheel_back_right")
    {
      x_pos[i]  = -x / 2.0;
      y_pos[i]  = -y / 2.0;
      continue;
    }
  }
}

void Wheels::update_current_state()
{
  for (int i = 0; i < 4; i++)
  {
    current_state.position[i] = get_position(i);
    current_state.velocity[i] = get_velocity(i); //TODO figure out which index this should be
    current_state.effort[i]   = get_effort(i);
  }
}
