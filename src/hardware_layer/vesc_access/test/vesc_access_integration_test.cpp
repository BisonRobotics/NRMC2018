#include <vesc_access/vesc_access.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  float transmission_ratio = 1.0f;
  float output_ratio = 1.0f;
  float velocity_limit = 100.0f;
  float torque_limit = 2.0f;
  unsigned int pole_pairs = 14;
  float torque_const = 1.0f;
  VescAccess *vesc = new VescAccess(0, transmission_ratio, output_ratio, velocity_limit, torque_limit, torque_const,
                                    (char *)"can0", pole_pairs);

  unsigned int number_of_cycles = 4;
  for (unsigned int ctr = 0; ctr < number_of_cycles; ctr++)
  {
    vesc->setLinearVelocity(60.0f);
    sleep(1);
    vesc->setLinearVelocity(0.0f);
    sleep(1);
    vesc->setLinearVelocity(-60.0f);
    sleep(1);
  }
}
