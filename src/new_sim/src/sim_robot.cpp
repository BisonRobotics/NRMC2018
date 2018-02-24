#include "sim_robot/sim_robot.h"

SimRobot::SimRobot(double axelLen, double xi, double yi, double thi)
{
  fl = new SimVesc(16, 0, 1.0);
  fr = new SimVesc(16, 0, 1.0);
  br = new SimVesc(16, 0, 1.0);
  bl = new SimVesc(16, 0, 1.0);

  imu = new SimImu(0, 0, 0);
  pos = new SimPos(0, 0, 0);

  deadReck = new Localizer(axelLen, xi, yi, thi, fl, fr, br, bl);
  prevXVel = 0;
  prevYVel = 0;
}

SimRobot::SimRobot(double axelLen, double xi, double yi, double thi, double VescPgain)
{
  fl = new SimVesc(VescPgain, 0, 1.0);
  fr = new SimVesc(VescPgain, 0, 1.0);
  br = new SimVesc(VescPgain, 0, 1.0);
  bl = new SimVesc(VescPgain, 0, 1.0);

  imu = new SimImu(0, 0, 0);
  pos = new SimPos(0, 0, 0);

  deadReck = new Localizer(axelLen, xi, yi, thi, fl, fr, br, bl);
  prevXVel = 0;
  prevYVel = 0;
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
}

iVescAccess *SimRobot::getFLVesc()
{
  return fl;
}

iVescAccess *SimRobot::getFRVesc()
{
  return fr;
}
iVescAccess *SimRobot::getBRVesc()
{
  return br;
}
iVescAccess *SimRobot::getBLVesc()
{
  return bl;
}

ImuSensorInterface *SimRobot::getImu()
{
  return imu;
}
PosSensorInterface *SimRobot::getPos()
{
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

LocalizerInterface::stateVector SimRobot::getStates()
{
  return deadReck->getStateVector();
}
