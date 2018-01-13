#ifndef __READABLE_SENSORS_H__
#define __READABLE_SENSORS_H__

class ReadableSensors
{
public:
  enum class ReadStatus
  {
    READ_FAILED,
    READ_SUCCESS
  };  // this requires cpp11
  virtual ReadStatus receiveData() = 0;
};

#endif