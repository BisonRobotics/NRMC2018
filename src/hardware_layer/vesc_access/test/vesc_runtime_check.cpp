#include <ros/ros.h>
#include "vesc_control/vesc_socket_can.h"
#include "wheel_params/wheel_params.h"

class VescDiagnostic
{
public:
  VescDiagnostic(char *network, uint8_t can_id, std::string motor_name)
    : vesc(network, can_id, motor_name), can_network(network), id(can_id), name(motor_name)
  {
  }
  Vesc vesc;
  std::string name;
  uint8_t id;
  std::string can_network;
  std::string toString()
  {
    std::stringstream ss;
    ss << "vesc: " + name + " on network: " + can_network + " with id: ";
    ss << id;
    return ss.str();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "runtime_check");
  VescDiagnostic front_left_wheel =
      VescDiagnostic(front_left_param.can_network, front_left_param.can_id, std::string("front left wheel"));
  VescDiagnostic front_right_wheel =
      VescDiagnostic(front_right_param.can_network, front_right_param.can_id, std::string("front right wheel"));
  VescDiagnostic back_left_wheel =
      VescDiagnostic(back_left_param.can_network, back_left_param.can_id, std::string("back_left_wheel"));
  VescDiagnostic back_right_wheel =
      VescDiagnostic(back_right_param.can_network, back_right_param.can_id, std::string("back_right_wheel"));
  VescDiagnostic central_drive =
      VescDiagnostic(shoulder_param.can_network, shoulder_param.can_id, std::string("central_drive"));
  VescDiagnostic linear = VescDiagnostic(linear_param.can_network, front_right_param.can_id, std::string("linear"));
  VescDiagnostic sifter = VescDiagnostic(sifter_param.can_network, sifter_param.can_id, std::string("sifter"));
  VescDiagnostic small_conveyor =
      VescDiagnostic(small_conveyor_param.can_network, small_conveyor_param.can_id, std::string("small_conveyor"));
  VescDiagnostic large_conveyor =
      VescDiagnostic(large_conveyor_param.can_network, large_conveyor_param.can_id, std::string("large_conveyor"));
  VescDiagnostic left_outrigger =
      VescDiagnostic(left_outrigger_param.can_network, left_outrigger_param.can_id, std::string("left_outrigger"));
  VescDiagnostic right_outrigger =
      VescDiagnostic(right_outrigger_param.can_network, right_outrigger_param.can_id, std::string("right_outrigger"));
  VescDiagnostic *vesc_array[NUMBER_OF_MOTORS] = { &front_left_wheel, &front_right_wheel, &back_left_wheel,
                                                   &back_right_wheel, &central_drive, &linear, &sifter, &small_conveyor,
                                                   &large_conveyor, &left_outrigger, &right_outrigger };
  while (ros::ok())
  {
    bool motor_down = false;
    for (unsigned int ctr = 0; ctr < NUMBER_OF_MOTORS; ctr++)
    {
      if (!vesc_array[ctr]->vesc.isAlive())
      {
        std::cout << vesc_array[ctr]->toString() + " is dead!" << std::endl;
        motor_down = true;
      }
    }
    if (!motor_down)
    {
      std::cout << "all motors good" << std::endl;
    }
    sleep(1);
    ros::spinOnce();
  }
}