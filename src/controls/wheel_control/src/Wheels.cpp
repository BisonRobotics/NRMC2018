#include <string>
#include "Wheels.h"

using namespace wheel_control;

Wheel::Wheel(std::string name)
{
    this->name   = name;
    this->id     = -1;
    this->x_pos  = 0.0;
    this->y_pos  = 0.0;
    this->radius = 0.0;

    this->current_state.position = 0.0;
    this->current_state.velocity = 0.0;
    this->current_state.effort   = 0.0;
}

Wheel::Wheel(std::string name, double x_pos, double y_pos, double radius) : Wheel(name)
{
    this->x_pos  = x_pos;
    this->y_pos  = y_pos;
    this->radius = radius;
}

Wheels::Wheels() {
    this->right_front = new Wheel("right_front");
    this->right_back  = new Wheel("right_back");
    this->left_front  = new Wheel("left_front");
    this->left_back   = new Wheel("left_back");
}

Wheels::Wheels(double radius, double x_dist, double y_dist) : Wheels()
{
    for (auto wheel : this->get()) wheel->radius = radius;

    this->right_front->x_pos =  x_dist / 2.0;
    this->right_back ->x_pos = -x_dist / 2.0;
    this->left_front ->x_pos =  x_dist / 2.0;
    this->left_back  ->x_pos = -x_dist / 2.0;

    this->right_front->y_pos =  y_dist / 2.0;
    this->right_back ->y_pos =  y_dist / 2.0;
    this->left_front ->y_pos = -y_dist / 2.0;
    this->left_back  ->y_pos = -y_dist / 2.0;
}

Wheel* Wheels::get_wheel(std::string name) {
    for (auto wheel : this->get())
    {
        if (wheel->name == name) {
            return wheel;
        }
    }
}

std::vector<Wheel*> Wheels::get() {
    std::vector<Wheel*> wheels;
    wheels.push_back(this->right_front);
    wheels.push_back(this->right_back);
    wheels.push_back(this->left_front);
    wheels.push_back(this->left_back);
    return wheels;
}

