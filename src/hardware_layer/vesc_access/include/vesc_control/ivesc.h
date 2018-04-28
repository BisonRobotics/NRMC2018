#ifndef __VESC_INTERFACE_H_
#define __VESC_INTERFACE_H_
class iVesc
{
public:
  //    virtual ~iVesc();
  //    virtual iVesc();
  virtual void setRpm(float rpm) = 0;
  virtual void setCurrent(float current) = 0;
  virtual float getCurrent(void) = 0;
  virtual int getRpm(void) = 0;
  virtual int getADC(void) = 0;
  virtual bool getForLimit(void) = 0;
  virtual bool getRevLimit(void) = 0;
  virtual void setDuty(float d) = 0;
};

class VescException : public std::runtime_error
{
public:
  explicit VescException(const char* msg) : std::runtime_error(msg)
  {
  }
  explicit VescException(std::string msg) : std::runtime_error(msg){

  }
};
#endif
