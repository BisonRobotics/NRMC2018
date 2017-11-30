#ifndef CAN_SENSORS_H
#define CAN_SENSORS_H

#include <iostream>
// socketcan includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <stdint.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>

#include <sys/time.h>

// TODO, Move includes to cpp file for betterness
// TODO, Add timeout to canRecieve

class CanSensor
{
private:
  struct ifreq ifr;
  struct sockaddr_can addr;
  int s;
  int sbcm;
  unsigned int canID;

  struct timeval prevmsgtime;

  struct can_msg
  {
    struct bcm_msg_head msg_head;
    struct can_frame frame[1];
  } msg;

protected:
  enum class CanReadStatus  // this requires cpp11
  {
    CAN_READ_FAILED,
    CAN_READ_SUCCESS
  };
  enum class CanWriteStatus  // this requires cpp11
  {
    CAN_WRITE_FAILED,
    CAN_WRITE_SUCCESS
  };
  CanWriteStatus canSend(uint8_t *data, uint8_t len);  // future feature, not tested
  CanReadStatus canReceive(uint8_t *databuffer);

public:
  CanSensor(unsigned int cID, char *interface);
  // int setRefreshRate(uint8_t Hz); //future feature
  // virtual int recieveData(void * ret)=0; //moved to readable_sensors
};

#endif
