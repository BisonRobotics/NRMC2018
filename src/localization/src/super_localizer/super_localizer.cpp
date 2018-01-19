#include <super_localizer/super_localizer.h>

SuperLocalizer::SuperLocalizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                               iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc,
                               ImuSensorInterface *centerIMU, PosSensorInterface *posSensor,
                               LocalizerInterface::stateVector gains)
{
  this->deadReck = new Localizer(axleLen, xi, yi, thi, frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  this->cIMU = centerIMU;
  this->pSensor = posSensor;
  this->sensors[0] = centerIMU;
  this->sensors[1] = posSensor;
  this->num_sensors = 2;
  this->have_pos = true;
  this->have_imu = true;

  this->state_vector = LocalizerInterface::initState(xi, yi, thi);
  this->residual = LocalizerInterface::initState(0, 0, 0);
  this->measured = LocalizerInterface::initState(0, 0, 0);
  this->data_is_good = false;
  this->gainVector = gains;
}

SuperLocalizer::SuperLocalizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                               iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc,
                               PosSensorInterface *posSensor, LocalizerInterface::stateVector gains)

{
  this->deadReck = new Localizer(axleLen, xi, yi, thi, frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  this->pSensor = posSensor;
  this->num_sensors = 1;
  this->have_pos = true;
  this->have_imu = false;
  this->data_is_good = false;
  this->sensors[0] = posSensor;

  this->state_vector = LocalizerInterface::initState(xi, yi, thi);
  this->residual = LocalizerInterface::initState(0, 0, 0);
  this->measured = LocalizerInterface::initState(0, 0, 0);

  this->gainVector = gains;
}

SuperLocalizer::UpdateStatus SuperLocalizer::updateStateVector(double dt)
{

  // do dead reckoning calculation, this uses information from the vescs
  this->deadReck->updateStateVector(dt);
  // read sensors
  for (int a = 0; a < num_sensors; a++){
    if(this->sensors[a]->receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS){
	data_is_good = true;
    }
  }

  if (data_is_good){
	  // get IMU data, integrate it. This is measured vel
	  if (have_imu)
	  {
	    this->measured.x_vel += cIMU->getX() * dt;
	    this->measured.y_vel += cIMU->getY() * dt;
	    this->measured.omega = cIMU->getOmega();
	    this->measured.x_accel = cIMU->getX();
	    this->measured.y_accel = cIMU->getY();
	  }
	  else
	  {
	    this->measured.x_vel = deadReck->getStateVector().x_vel;
	    // unimplemented sensor, set to model value so residual stays 0
	    this->measured.y_vel = deadReck->getStateVector().y_vel;
	    // unimplemented sensor, set to model value so residual stays 0
	    this->measured.omega = deadReck->getStateVector().omega;
	    // unimplemented sensor, set to model value so residual stays 0
	  }

	  // get Pos data, This is measured pos
	  if (have_pos)
	  {
	    this->measured.x_pos = pSensor->getX();
	    this->measured.y_pos = pSensor->getY();
	    this->measured.theta = pSensor->getTheta();
	  }
	  else
	  {
	    this->measured.x_pos = deadReck->getStateVector().x_pos;
	    // unimplemented sensor, set to model value so residual stays 0
	    this->measured.y_pos = deadReck->getStateVector().y_pos;
	    // unimplemented sensor, set to model value so residual stays 0
	    this->measured.theta = deadReck->getStateVector().theta;
	    // unimplemented sensor, set to model value so residual stays 0
	  }

	  // take difference between estimated data and measured data, this is residual
	  this->residual = LocalizerInterface::diff(state_vector, measured);
	  // revise estimate by subtracting residual from it * some gain

	  LocalizerInterface::stateVector intermediateStateVector =
	      LocalizerInterface::addFromModel(this->state_vector, this->deadReck->getStateVector(), dt, have_imu);
	  LocalizerInterface::stateVector intermediate = LocalizerInterface::multiply(this->gainVector, this->residual);
	  this->state_vector = LocalizerInterface::diff(intermediateStateVector, intermediate);
  }
}

SuperLocalizer::~SuperLocalizer()
{
  delete deadReck;
}

LocalizerInterface::stateVector SuperLocalizer::getStateVector()
{
  return state_vector;
}

LocalizerInterface::stateVector SuperLocalizer::getResidual()
{
  return residual;
}

LocalizerInterface::stateVector SuperLocalizer::getGainVector()
{
  return gainVector;
}

LocalizerInterface::stateVector SuperLocalizer::getMeasured()
{
  return measured;
}

bool SuperLocalizer::getHaveImu()
{
  return have_imu;
}

bool SuperLocalizer::getHavePosition()
{
  return have_pos;
}
