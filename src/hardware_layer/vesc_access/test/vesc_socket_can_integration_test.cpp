#include <vesc_control/vesc_socket_can.h>
#include <unistd.h>

#define rpm_fast        16000.0f


int main(int argc, char **argv)
{
 Vesc *vesc = new Vesc((char *)"can0", 7);

  unsigned int number_of_cycles = 4;
  for (unsigned int ctr = 0; ctr < number_of_cycles; ctr++)
  {
    std::cout << "going forward" << std::endl; 
    vesc->setRpm(rpm_fast);
    sleep(1);
    vesc->setRpm(0.0f);
    sleep(1);
    vesc->setRpm(-rpm_fast);
    sleep(1);
  }
}
