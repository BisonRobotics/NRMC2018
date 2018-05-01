#include <super_localizer/super_localizer.h>
#define _USE_MATH_DEFINES
#include <cmath>

SuperLocalizer::SuperLocalizer(double axleLen, double xi, double yi, double thi, iVescAccess *frontLeftVesc,
                               iVescAccess *frontRightVesc, iVescAccess *backRightVesc, iVescAccess *backLeftVesc,
                               ImuSensorInterface *centerIMU, PosSensorInterface *posSensor,
                               LocalizerInterface::stateVector gains)
{
  this->deadReck = new Localizer(axleLen, xi, yi, thi, frontLeftVesc, frontRightVesc, backRightVesc, backLeftVesc);

  this->cIMU = centerIMU;
  this->pSensor = posSensor;
  this->sensors[imu_sensor_index] = centerIMU;
  this->sensors[position_sensor_index] = posSensor;
  this->num_sensors = 2;
  this->have_pos = true;
  this->have_imu = true;
  this->imu_is_good = false;
  this->pos_is_good = false;
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
  this->sensors[position_sensor_index] = posSensor;
  this->imu_is_good = false;
  this->pos_is_good = false;
  this->state_vector = LocalizerInterface::initState(xi, yi, thi);
  this->residual = LocalizerInterface::initState(0, 0, 0);
  this->measured = LocalizerInterface::initState(0, 0, 0);
  this->gainVector = gains;
}

void SuperLocalizer::setDataIsGood(void)
{
    if (have_imu && have_pos)
    {
      this->data_is_good = imu_is_good && pos_is_good;
    }
    else if (have_pos)
    {
      this->data_is_good = pos_is_good;
    }
}


SuperLocalizer::UpdateStatus SuperLocalizer::updateStateVector(double dt)
{
  bool floating = false;
  // do dead reckoning calculation, this uses information from the vescs
  this->deadReck->updateStateVector(dt, state_vector.theta);
  // read sensor_
  for (int a = 0; a < num_sensors; a++)
  {
    if (this->sensors[a]->receiveData() == ReadableSensors::ReadStatus::READ_SUCCESS)
    {
      if (a==position_sensor_index){
        pos_is_good = true;
      }
      else if (a==imu_sensor_index)
      {
        imu_is_good = true;
      }
    }
  }

  setDataIsGood();

  if (data_is_good)
  {
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
      floating = pSensor->isFloating();  // this is done this way to prevent accessing a null method on pSensor
    }
    else
    {
      floating = true;
    }

    if (!floating)
    {
      this->measured.x_pos = pSensor->getX();
      this->measured.y_pos = pSensor->getY();
      this->measured.theta = pSensor->getTheta();
    }
    else
    {
      this->measured.x_pos += deadReck->getStateVector().x_vel * dt;
      // unimplemented sensor, set to model value so residual stays 0
      this->measured.y_pos += deadReck->getStateVector().y_vel * dt;
      // unimplemented sensor, set to model value so residual stays 0
      this->measured.theta += deadReck->getStateVector().omega * dt;
      // unimplemented sensor, set to model value so residual stays 0
      // if there is no position sensor, measured begins to integrate dead reck vel's
    }

    // take difference between estimated data and measured data, this is residual
    this->residual = LocalizerInterface::diff(state_vector, measured);  // measured pos
    // revise estimate by subtracting residual from it * some gain

    LocalizerInterface::stateVector intermediateStateVector =  // this is current estimate + model data
    LocalizerInterface::addFromModel(this->state_vector, this->deadReck->getStateVector(), dt, have_imu);
    LocalizerInterface::stateVector intermediate = LocalizerInterface::multiply(this->gainVector, this->residual);
    this->state_vector = LocalizerInterface::diff(intermediateStateVector, intermediate);
    if (this->state_vector.theta > M_PI)
    {
      this->state_vector.theta -= 2.0 * M_PI;
    }
    else if (this->state_vector.theta < -M_PI)
    {
      this->state_vector.theta += 2.0 * M_PI;
    }
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

bool SuperLocalizer::getIsDataGood()
{
  return data_is_good;
}
