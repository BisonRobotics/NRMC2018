#include "bucket_controller/bucket_controller.h"

BucketController::BucketController(iVescAccess *bigConveyorVesc, iVescAccess *littleConveyorVesc, iVescAccess *sifterVesc)
{
  bc = bigConveyorVesc;
  lc = littleConveyorVesc;
  sf = sifterVesc; 
}

void BucketController::turnBigConveyorOn()
{
  bc->setLinearVelocity(10);
}

void BucketController::turnBigConveyorOff()
{
  bc->setLinearVelocity(0);
}

void BucketController::turnLittleConveyorOn()
{
  lc->setLinearVelocity(10);
}

void BucketController::turnLittleConveyorOff()
{
  lc->setLinearVelocity(0);
}

void BucketController::turnSifterOn()
{
  sf->setLinearVelocity(10);
}

void BucketController::turnSifterOff()
{
  sf->setLinearVelocity(0);
}