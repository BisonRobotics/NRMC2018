#include <super_localizer/super_localizer.h>

SuperLocalizer::SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, IMUCanSensor *centerIMU, POSCanSensor* posSensor)
{
  deadReck = Localizer(frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  cIMU = centerIMU;
  pSensor = posSensor;
  sensors[0] = centerIMU;
  sensors[1] = posSensor;
  num_sensors = 2;
  have_pos = true;
  have_imu = true;

  residual = {0};
  measured = {0};

  gainVector.x_accel = 0;
  gainVector.y_accel = 0;
  gainVector.alpha   = 0;
  gainVector.x_vel   = DXRESGAIN;
  gainVector.y_vel   = DYRESGAIN;
  gainVector.omega   = OMEGARESGAIN;
  gainVector.x_pos   = XRESGAIN;
  gainVector.y_pos   = YRESGAIN;
  gainVector.theta   = THETARESGAIN;

}

SuperLocalizer::SuperLocalizer(iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                   iVescAccess *backLeftVesc, POSCanSensor* posSensor)
{
  deadReck = Localizer(frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  pSensor = posSensor;
  num_sensors =1;
  have_pos = true;
  have_imu = false;

  sensors[0] = posSensor;

  residual = {0};
  measured = {0};

  gainVector.x_accel = 0;
  gainVector.y_accel = 0;
  gainVector.alpha   = 0;
  gainVector.x_vel   = DXRESGAIN;
  gainVector.y_vel   = DYRESGAIN;
  gainVector.omega   = OMEGARESGAIN;
  gainVector.x_pos   = XRESGAIN;
  gainVector.y_pos   = YRESGAIN;
  gainVector.theta   = THETARESGAIN;

}

Localizer::UpdateStatus SuperLocalizer::updateStateVector(float dt)
{
  //do dead reckoning calculation
  this->deadReck.updateStateVector(dt);
  //read sensors
  for (int a = 0; a<num_sensors;a++) sensors[a]->receiveData();

  //get IMU data, integrate it. This is measured vel
  if (have_imu)
  {
    measured.x_vel += cIMU->imuData.x * dt;
    measured.y_vel += cIMU->imuData.y * dt;
    measured.omega = deadReck.state_vector.omega; //unimplemented sensor, set to model value so residual stays 0
  }
  else
  {
    measured.x_vel = deadReck.state_vector.x_vel; //unimplemented sensor, set to model value so residual stays 0
    measured.y_vel = deadReck.state_vector.y_vel; //unimplemented sensor, set to model value so residual stays 0
    measured.omega = deadReck.state_vector.omega; //unimplemented sensor, set to model value so residual stays 0
  }

  //get Pos data, This is measured pos
  if (have_pos)
  {
    measured.x_pos = pSensor->posData.x;
    measured.y_pos = pSensor->posData.y;
    measured.theta = pSensor->posData.theta;
  }
  else
  {
    measured.x_pos = deadReck.state_vector.x_pos; //unimplemented sensor, set to model value so residual stays 0
    measured.y_pos = deadReck.state_vector.y_pos; //unimplemented sensor, set to model value so residual stays 0
    measured.theta = deadReck.state_vector.theta; //unimplemented sensor, set to model value so residual stays 0
  }

  //take difference between estimated data and measured data, this is residual
  residual = diff(measured, deadReck.state_vector);
  //revise estimate by subtracting residual from it * some gain
  state_vector = diff(deadReck.state_vector, multiply(gainVector,residual));
}

Localizer::stateVector_s diff(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs)
{
   Localizer::stateVector_s ret;
   ret.alpha = lhs.alpha - rhs.alpha;
   ret.omega = lhs.omega - rhs.omega;
   ret.theta = lhs.theta - rhs.omega;
   ret.x_accel = lhs.x_accel - rhs.x_accel;
   ret.y_accel = lhs.y_accel - rhs.y_accel;
   ret.x_vel = lhs.x_vel - rhs.x_vel;
   ret.y_vel = lhs.y_vel - rhs.y_vel;
   ret.x_pos = lhs.x_pos - rhs.x_pos;
   ret.y_pos = lhs.y_pos - rhs.y_pos;
   return ret;
}

Localizer::stateVector_s multiply(Localizer::stateVector_s const& lhs, Localizer::stateVector_s const& rhs)
{
   Localizer::stateVector_s ret;
   ret.alpha = lhs.alpha * rhs.alpha;
   ret.omega = lhs.omega * rhs.omega;
   ret.theta = lhs.theta * rhs.omega;
   ret.x_accel = lhs.x_accel * rhs.x_accel;
   ret.y_accel = lhs.y_accel * rhs.y_accel;
   ret.x_vel = lhs.x_vel * rhs.x_vel;
   ret.y_vel = lhs.y_vel * rhs.y_vel;
   ret.x_pos = lhs.x_pos * rhs.x_pos;
   ret.y_pos = lhs.y_pos * rhs.y_pos;
   return ret;
}

