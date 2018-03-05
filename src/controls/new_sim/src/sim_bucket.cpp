#include "sim_robot/sim_bucket.h"

SimBucket::SimBucket()
{
  bc = new SimVesc(16, 0, 1);
  lc = new SimVesc(16, 0, 1);
  sf = new SimVesc(16, 0, 1);
}

void SimBucket::update(double dt)
{
  // Do nothing for now!
  // Could eventually read vesc speeds and update an
  // isRunning bool for each vesc, but not really necessary
}

iVescAccess *SimBucket::getBigConveyorVesc()
{
  return bc;
}

iVescAccess *SimBucket::getLittleConveyorVesc()
{
  return lc;
}

iVescAccess *SimBucket::getSifterVesc()
{
  return sf;
}