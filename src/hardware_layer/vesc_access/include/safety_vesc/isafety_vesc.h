#ifndef PROJECT_iSAFETY_VESC_H
#define PROJECT_iSAFETY_VESC_H

namespace safetyvesc
{
  typedef struct joint_params {
    double minimum_pos;
    double maximum_pos;
    double safety_check_pos;
    double gain;
    double setpoint_tolerance;
    double lower_limit_position;
    double upper_limit_position;
  }joint_params_t;

}


class iSafetyVesc
{
public:
  virtual void setPositionSetpoint (double pos) = 0;
  virtual void setVelocitySetpoint (double velocity) = 0;
  virtual double getPosition (void) = 0;
  virtual double updateVelocity (void) = 0;
  virtual bool isInit () = 0;
  virtual bool isAtSetpoint () = 0;
  virtual void init () = 0;
  virtual void updatePosition (double dt) = 0;
  virtual void stop () = 0;
  virtual double getSafetyPosition () = 0;
  virtual double getVelocity () = 0;
};



#endif //PROJECT_SAFETY_VESC_H
