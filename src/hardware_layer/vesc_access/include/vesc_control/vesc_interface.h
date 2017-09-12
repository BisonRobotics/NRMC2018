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
};
#endif
