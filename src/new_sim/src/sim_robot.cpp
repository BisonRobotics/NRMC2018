#include <sim_robot.h>

SimRobot::SimRobot(double axelLen, double xi, double yi, double thi)
{
  front_left_vesc  = new SimVesc(.2, .01);
  front_right_vesc = new SimVesc(.2, .01);
  back_right_vesc  = new SimVesc(.2, .01);
  back_left_vesc   = new SimVesc(.2, .01);

  imu = new SimImu(0,0,0);
  pos = new SimPos(0,0,0);

  deadReck = new Localizer(axelLen, xi, yi, thi, front_left_vesc, front_right_vesc, back_right_vesc, back_left_vesc);
}

void SimRobot::update(double dt)
{
  deadReck->updateStateVector(dt);
  fl->update(dt);
  fr->update(dt);
  br->update(dt);
  bl->update(dt);
  imu->update(deadReck->getStateVector(), dt);
  pos->update(deadReck->getStateVector(), dt);

  //post pose/statevector to \sim_robot_base_link

}