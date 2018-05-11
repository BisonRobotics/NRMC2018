#ifndef BUCKET_CONTROLLER_H
#define BUCKET_CONTROLLER_H

#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

class BucketController
{
public:
  BucketController(iVescAccess *bigConveyorVesc, iVescAccess *littleConveyorVesc, iVescAccess *sifterVesc);

  void turnBigConveyorOn();
  void turnBigConveyorOff();
  void turnLittleConveyorOn();
  void turnLittleConveyorOff();
  void turnSifterOn();
  void turnSifterOff();
  void toggleBigConveyor();
  void toggleSifter();
  void toggleLittleConveyor();
  bool init ();
private:
  iVescAccess *bc, *lc, *sf;
  bool sifter_state;
  bool little_conveyor_state;
  bool big_conveyor_state;
  bool has_been_init;
  ros::Time initial_time;
};

#endif
