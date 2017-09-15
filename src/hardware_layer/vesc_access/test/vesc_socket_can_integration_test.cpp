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
  Vesc *vesc = new Vesc((char *)"can0", 0);

  unsigned int number_of_cycles = 10;
  for (unsigned int ctr = 0; ctr < number_of_cycles; ctr++)
  {
    vesc->setRpm(60.0f);
    sleep(1);
    vesc->setRpm(0.0f);
    sleep(1);
    vesc->setRpm(-60.0f);
    sleep(1);
  }
}
