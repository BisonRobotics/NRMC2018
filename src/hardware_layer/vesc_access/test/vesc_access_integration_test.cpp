#include <vesc_access/vesc_access.h>
#include <unistd.h>
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integration_test");
  float transmission_ratio = 1.0f;
  float output_ratio = 1.0f;
  float velocity_limit = 10000.0f;
  float torque_limit = 2.0f;
  unsigned int pole_pairs = 14;
  float torque_const = 1.0f;

  VescAccess *vesc = new VescAccess(7, transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_const,
                                    (char *)"can0", pole_pairs);

  std::cout << "starting" << std::endl;

  unsigned int number_of_cycles = 4;
  for (unsigned int ctr = 0; ctr < number_of_cycles; ctr++)
  {
    std::cout << "setting forward" << std::endl;
    vesc->setLinearVelocity(1000.0f);
    sleep(3);
    vesc->setLinearVelocity(0.0f);
    sleep(3);
    vesc->setLinearVelocity(-1000.0f);
    sleep(3);
    ros::spinOnce();
  }
}
