#include "bucket_controller/bucket_controller.h"

BucketController::BucketController(iVescAccess *bigConveyorVesc, iVescAccess *littleConveyorVesc,
                                   iVescAccess *sifterVesc)
{
  bc = bigConveyorVesc;
  lc = littleConveyorVesc;
  sf = sifterVesc;
  big_conveyor_state = false;
  little_conveyor_state = false;
  sifter_state = false;
}

bool BucketController::init ()
{
    static constexpr double time_to_init=3;
    bool ret_val = false;
    if (!has_been_init) {
      bc->setLinearVelocity(15000);
      initial_time = ros::Time::now();
    } else if ((ros::Time::now()-initial_time).toSec() > time_to_init)
    {
      bc->setTorque(0);
      ret_val = true;
    }
  return ret_val;
}


void BucketController::turnBigConveyorOn()
{
  bc->setLinearVelocity(35000); // in erpm
  big_conveyor_state = true;
}

void BucketController::turnBigConveyorOff()
{
  bc->setTorque(0.0);
  big_conveyor_state = false;
}

void BucketController::turnLittleConveyorOn()
{

  lc->setDuty(.4); // in Amps
  little_conveyor_state = true;
}

void BucketController::turnLittleConveyorOff()
{
  lc->setTorque(0);
  little_conveyor_state = false;
}

void BucketController::turnSifterOn()
{
  sf->setTorque(3.4);
  sifter_state = true;
}

void BucketController::turnSifterOff()
{
  sf->setTorque(0);
  sifter_state = false;
}

void BucketController::toggleBigConveyor()
{
  if (big_conveyor_state)
  {
    turnBigConveyorOff();
  }
  else
  {
    turnBigConveyorOn();
  }
}

void BucketController::toggleSifter()
{
  if (sifter_state)
  {
    turnSifterOff();
  }
  else
  {
    turnSifterOn();
  }
}

void BucketController::toggleLittleConveyor()
{
  if (little_conveyor_state)
  {
    turnLittleConveyorOff();
  }
  else
  {
    turnLittleConveyorOn();
  }
}