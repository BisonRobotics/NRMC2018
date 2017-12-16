#include <super_localizer/super_localizer.h>

SuperLocalizer::SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                               iVescAccess *backLeftVesc, ImuCanSensorInterface *centerIMU, PosCanSensorInterface *posSensor,
                               LocalizerInterface::stateVector gains)
{
  deadReck = new Localizer(axleLen, xi, yi, thi, frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  cIMU = centerIMU;
  pSensor = posSensor;
  sensors[0] = centerIMU;
  sensors[1] = posSensor;
  num_sensors = 2;
  have_pos = true;
  have_imu = true;

  state_vector = { 0 };
  residual = { 0 };
  measured = { 0 };

  gainVector = gains;
}

SuperLocalizer::SuperLocalizer(float axleLen, float xi, float yi, float thi, iVescAccess *frontLeftVesc, iVescAccess *frontRightVesc, iVescAccess *backRightVesc,
                               iVescAccess *backLeftVesc, PosCanSensorInterface *posSensor, LocalizerInterface::stateVector gains)
{
  deadReck = new Localizer(axleLen, xi, yi, thi, frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  pSensor = posSensor;
  num_sensors = 1;
  have_pos = true;
  have_imu = false;

  sensors[0] = posSensor;

  state_vector = { 0 };
  residual = { 0 };
  measured = { 0 };

  gainVector = gains;
}

SuperLocalizer::UpdateStatus SuperLocalizer::updateStateVector(float dt)
{
  // do dead reckoning calculation, this uses information from the vescs
  this->deadReck->updateStateVector(dt);
  // read sensors
  for (int a = 0; a < num_sensors; a++)
    sensors[a]->receiveData();

  // get IMU data, integrate it. This is measured vel
  if (have_imu)
  {
    measured.x_vel += cIMU->getX() * dt;
    measured.y_vel += cIMU->getY() * dt;
    measured.omega = deadReck->getStateVector().omega;  // unimplemented sensor, set to model value so residual stays 0
  }
  else
  {
    measured.x_vel = deadReck->getStateVector().x_vel;  // unimplemented sensor, set to model value so residual stays 0
    measured.y_vel = deadReck->getStateVector().y_vel;  // unimplemented sensor, set to model value so residual stays 0
    measured.omega = deadReck->getStateVector().omega;  // unimplemented sensor, set to model value so residual stays 0
  }

  // get Pos data, This is measured pos
  if (have_pos)
  {
    measured.x_pos = pSensor->getX();
    measured.y_pos = pSensor->getY();
    measured.theta = pSensor->getTheta();
  }
  else
  {
    measured.x_pos = deadReck->getStateVector().x_pos;  // unimplemented sensor, set to model value so residual stays 0
    measured.y_pos = deadReck->getStateVector().y_pos;  // unimplemented sensor, set to model value so residual stays 0
    measured.theta = deadReck->getStateVector().theta;  // unimplemented sensor, set to model value so residual stays 0
  }

  // take difference between estimated data and measured data, this is residual
  residual = diff(state_vector, measured);
  // revise estimate by subtracting residual from it * some gain

  //it would be more efficient for these calculations to happen in place maybe, instead of all these return shits
  LocalizerInterface::stateVector intermediateStateVector = addfrommodel(state_vector, deadReck->getStateVector(),dt);
  //state_vector = addfrommodel(state_vector, deadReck->getStateVector(),dt);
  LocalizerInterface::stateVector intermediate = multiply(gainVector, residual);
  state_vector = diff(intermediateStateVector, intermediate);
}

SuperLocalizer::~SuperLocalizer()
{
	delete deadReck;
}

LocalizerInterface::stateVector SuperLocalizer::getStateVector()
{
  return state_vector;
}
