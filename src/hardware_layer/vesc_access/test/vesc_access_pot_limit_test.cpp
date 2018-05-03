//
// Created by marcintrosh on 2/24/18.
//
#include <wheel_params/wheel_params.h>
#include "vesc_access/vesc_access.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pot_limit_test");
  float transmission_ratio = 1.0f;
  float output_ratio = 1.0f;
  float velocity_limit = 10000.0f;
  float torque_limit = 2.0f;
  unsigned int pole_pairs = 14;
  float torque_const = 1.0f;

  VescAccess *vesc = new VescAccess(linear_param, true);

  std::cout << "starting" << std::endl;
  ros::Rate rate(10);
  while (ros::ok())
  {
    try
    {
      std::cout << "limit: ";
      if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::inTransit)
      {
        std::cout << "in transit ";
      }
      else if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
      {
        std::cout << "bottom of motion ";
      }
      else if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion)
      {
        std::cout << "Top of motion ";
      }
    }
    catch (VescException vescException)
    {
      vescException.what();
    }
    std::cout << std::endl
              << "Pot position: " << vesc->getPotPosition() << std::endl;
    rate.sleep();
  }
}
