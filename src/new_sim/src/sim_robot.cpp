

#include <sim_robot.h>

SimRobot::SimRobot(double axelLen, double xi, double yi, double thi)
{
  front_left_vesc  = new sim_vesc();
  front_right_vesc = new sim_vesc();
  back_right_vesc  = new sim_vesc();
  back_left_vesc   = new sim_vesc();

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