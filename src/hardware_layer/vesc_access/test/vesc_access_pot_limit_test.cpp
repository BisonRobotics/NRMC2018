//
// Created by marcintrosh on 2/24/18.
//
#include "vesc_access/vesc_access.h"


int main (int argc, char** argv){

  float transmission_ratio = 1.0f;
  float output_ratio = 1.0f;
  float velocity_limit = 10000.0f;
  float torque_limit = 2.0f;
  unsigned int pole_pairs = 14;
  float torque_const = 1.0f;

  VescAccess *vesc = new VescAccess(0, transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_const,
                                    (char *)"can0", pole_pairs);

  std::cout << "starting" << std::endl;

  while (1){
    std::cout << "limit: ";
    if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::inTransit)
    {
      std::cout << "in transit ";
    } else if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::bottomOfMotion)
    {
      std::cout << "bottom of motion ";
    } else if (vesc->getLimitSwitchState() == nsVescAccess::limitSwitchState::topOfMotion){
      std::cout << "Top of motion ";
    }
    std::cout << std::endl << "Pot position: " << vesc->getPotPosition() << std::endl;
    sleep (1);
  }


}
