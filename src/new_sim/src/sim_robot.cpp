#include "sim_robot/sim_robot.h"

SimRobot::SimRobot(double axelLen, double xi, double yi, double thi)
{
  fl = new SimVesc(.2, .01);
  fr = new SimVesc(.2, .01);
  br = new SimVesc(.2, .01);
  bl = new SimVesc(.2, .01);

  imu = new SimImu(0,0,0);
  pos = new SimPos(0,0,0);

  deadReck = new Localizer(axelLen, xi, yi, thi, fl, fr, br, bl);
  prevXVel =0;
  prevYVel =0;
}

void SimRobot::update(double dt)
{
  deadReck->updateStateVector(dt);
  fl->update(dt);
  fr->update(dt);
  br->update(dt);
  bl->update(dt);
  LocalizerInterface::stateVector states = deadReck->getStateVector();
  double xaccel = (states.x_vel - prevXVel) / dt;
  double yaccel = (states.y_vel - prevYVel) / dt;
  prevXVel = states.x_vel;
  prevYVel = states.y_vel;
  imu->update(xaccel, yaccel, states.omega);
  pos->update(states.x_pos, states.y_pos, states.theta);

  //post pose/statevector to \sim_robot_base_link

}

iVescAccess * SimRobot::getFLVesc(){
  return fl;
}

iVescAccess * SimRobot::getFRVesc(){
  return fr;
}
iVescAccess * SimRobot::getBRVesc(){
  return br;
}
iVescAccess * SimRobot::getBLVesc(){
  return bl;
}

ImuSensorInterface * SimRobot::getImu(){
  return imu;
}
PosSensorInterface * SimRobot::getPos(){
  return pos;
}

 
double SimRobot::getX()
{
  return deadReck->getStateVector().x_pos;
}

double SimRobot::getY()
{
  return deadReck->getStateVector().y_pos;
}

double SimRobot::getTheta()
{
  return deadReck->getStateVector().theta;
}
