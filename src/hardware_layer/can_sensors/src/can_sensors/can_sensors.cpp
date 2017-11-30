#include <can_sensors/can_sensors.h>

CanSensor::CanSensor(unsigned int cID, char *interface)
{
  canID = cID;
  s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (s == -1)
    throw "Unable to create raw CAN socket";
  strcpy(ifr.ifr_name, interface);
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  int ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
    throw "Unable to bind raw CAN socket";

  sbcm = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
  if (sbcm == -1)
    throw "Unable to create bcm socket";
  ret = connect(sbcm, (struct sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
    throw "Unable to connect bcm socket";
}

CanSensor::CanWriteStatus CanSensor::canSend(uint8_t *data, uint8_t len)
{
  msg.msg_head.opcode = TX_SETUP;
  msg.msg_head.can_id = canID;
  msg.msg_head.flags = SETTIMER | STARTTIMER | TX_CP_CAN_ID;
  msg.msg_head.nframes = 1;
  msg.msg_head.count = 0;
  msg.msg_head.ival1.tv_sec = 0;
  msg.msg_head.ival1.tv_usec = 0;
  msg.msg_head.ival2.tv_sec = 0;
  msg.msg_head.ival2.tv_usec = 1000 * 10;
  msg.frame[0].can_dlc = len;
  memcpy(msg.frame[0].data, data, len);
  //need to check return value of write or else there is a warning

  if (write(sbcm, &msg, sizeof(msg)) > 0) return CanSensor::CanWriteStatus::CAN_WRITE_SUCCESS;
  else return CanSensor::CanWriteStatus::CAN_WRITE_FAILED;
}

CanSensor::CanReadStatus CanSensor::canReceive(uint8_t *databuffer)
{
  struct can_frame readMsg;
  while (1)
  {
    int a = read(s, &readMsg, sizeof(msg)); //why are we reading something of size msg into something of size can_frame...
    //especially when a msg contains a can frame!
    if (a == -1)
      return CanSensor::CanReadStatus::CAN_READ_FAILED;  // no message?
    if (readMsg.can_id == canID)
    {
      memcpy(databuffer, readMsg.data, readMsg.can_dlc); //then here we copy the msg data into the buffer
      return CanSensor::CanReadStatus::CAN_READ_SUCCESS;
    }
    // else return -readMsg.can_id;
  }
}
