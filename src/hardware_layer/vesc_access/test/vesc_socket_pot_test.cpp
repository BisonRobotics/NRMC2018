#include "vesc_control/vesc_socket_can.h"
#include <iomanip>
#include <stdlib.h>
#define VESC_ID 0
#define CAN_INTERFACE "can0"
#include "ros/ros.h"
#include "wheel_params/wheel_params.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pot_test");
  Vesc motor((char *)"can0", shoulder_param.can_id, 1, "pot_test");  // TODO: this char* thing is a bit iffy, is this the right way to do it
  // motor.setCustom(0.1);
  // float current = 0;
  ros::Rate r(20);
  std::stringstream ss;
  while (ros::ok())
  {
    ros::spinOnce();
    ss << std::setprecision(4) << "mcur " << motor.getCurrent() << std::setw(9) << "\tduty " << motor.getDutyCycle()
       << std::setw(9) << "\trpm " << motor.getRpm() << std::setw(9) << "\tpos " << motor.getPosition() << std::setw(9)
       << "\ttach " << motor.getTachometer() << std::setw(9) << "\twatth " << motor.getWattHours() << std::setw(9)
       << "\tincur " << motor.getInCurrent() << std::setw(9) << "\tVin " << motor.getVin() << std::setw(9) << "\ttempM "
       << motor.getTempMotor() << std::setw(9) << "\ttempP " << motor.getTempPCB() << std::setw(9) << "\tfault  "
       << motor.getFaultCode() << std::setw(9) << "\tstate " << motor.getState() << "\tflim " << motor.getForLimit()
       << "\trlim " << motor.getRevLimit() << "\tadc " << motor.getADC() << "\tencind " << motor.encoderIndexFound()
       << "\talive " << motor.isAlive() << std::endl;
    ROS_INFO("%s", ss.str().c_str());

    // motor.getCurrent();
    // motor.setCurrent(current);
    // current = current + 0.01;
    // motor.setCustom(3.14159);
    r.sleep();
  }
}
