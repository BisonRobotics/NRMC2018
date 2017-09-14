#include <localizer.h>
#include <Eigen/Dense>
#include <cmath>
#define CONSTANTTOAVERAGETWONUMBERS 2.0f
#define AXELLEN .5f  // meters between wheels side to side (width of robot)

Localizer::Localizer(VescAccess *frontLeftVesc, VescAccess *frontRightVesc, VescAccess *backRightVesc,
                     VescAccess *backLeftVesc)
{
  stateVector.xPos = 0;
  stateVector.yPos = 0;
  stateVector.theta = 0;

  stateVector.xVel = 0;
  stateVector.yVel = 0;
  stateVector.omega = 0;

  stateVector.xAccel = 0;
  stateVector.yAccel = 0;
  stateVector.alpha = 0;

  fleftVesc = frontLeftVesc;
  frightVesc = frontRightVesc;
  brightVesc = backRightVesc;
  bleftVesc = backLeftVesc;

  gettimeofday(&currtime, NULL);  // initialize _prevmsgtime with something
  currtime.tv_sec -= 1;           // make it in the past to avoid false positives
}

Localizer::UpdateStatus Localizer::updateStateVector()
{
  if (false)
    return Localizer::UpdateStatus::UPDATE_FAILED_SENSOR_ERROR;
  else
  {
    // take current time and subtract it from previous time to get dt
    prevtime = currtime;
    gettimeofday(&currtime, NULL);
    dtms = timediffms(currtime, prevtime);
    float dt = dtms / 1000.0f;
    // get linear velocities of wheels
    float frontleftvel = fleftVesc->getLinearVelocity();
    float frontrightvel = frightVesc->getLinearVelocity();
    float backrightvel = brightVesc->getLinearVelocity();
    float backleftvel = bleftVesc->getLinearVelocity();

    float avgLeftVel = (frontleftvel + backleftvel) / CONSTANTTOAVERAGETWONUMBERS;
    float avgRightVel = (frontrightvel + backrightvel) / CONSTANTTOAVERAGETWONUMBERS;

    float w = (avgRightVel - avgLeftVel) / AXELLEN;
    float R;
    Eigen::Matrix2f rot;
    Eigen::Vector2f dPos;
    Eigen::Vector2f RonY;
    float dTheta;

    if (w != 0.0f)  // no dividing by zero
    {
      R = AXELLEN / 2 * (avgRightVel + avgLeftVel) / (avgRightVel - avgLeftVel);  // turn radius
      rot << cos(w * dt), -sin(w * dt), sin(w * dt), cos(w * dt);                 // rotation matrix
      RonY << 0, -R;
      dPos = rot * (RonY)-RonY;
      dTheta = w * dt;
    }
    else
    {
      dPos << avgRightVel *dt, 0;
      dTheta = 0;
    }

    // dPos is in robot coordinates, must transform to world
    // rotate by worldrobot theta

    Eigen::Matrix2f wrot;
    wrot << cos(stateVector.theta), -sin(stateVector.theta), sin(stateVector.theta), cos(stateVector.theta);
    Eigen::Vector2f dPosWorld;
    dPosWorld = wrot * dPos;

    // add it all up

    stateVector.xPos += dPosWorld(0);
    stateVector.yPos += dPosWorld(1);
    stateVector.theta += dTheta;
  }
}

int Localizer::timediffms(struct timeval curr, struct timeval prev)
{
  // stolen from candump.c, pretty gross
  struct timeval diff;
  diff.tv_sec = curr.tv_sec - prev.tv_sec;
  diff.tv_usec = curr.tv_usec - prev.tv_usec;
  if (diff.tv_usec < 0)
    diff.tv_sec--, diff.tv_usec += 1000000;
  if (diff.tv_sec < 0)
    diff.tv_sec = diff.tv_usec = 0;
  return diff.tv_sec * 1000 + diff.tv_usec / 1000;
}