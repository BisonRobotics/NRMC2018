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

void BucketController::turnBigConveyorOn()
{
  bc->setDuty(.25);
  big_conveyor_state = true;
}

void BucketController::turnBigConveyorOff()
{
  bc->setTorque(0.0);
  big_conveyor_state = false;
}

void BucketController::turnLittleConveyorOn()
{
  lc->setDuty(.9);
  little_conveyor_state = true;
}

void BucketController::turnLittleConveyorOff()
{
  lc->setTorque(0);
  little_conveyor_state = false;
}

void BucketController::turnSifterOn()
{
  sf->setDuty(.75);
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