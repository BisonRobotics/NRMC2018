//#include <ros.h>
//#include <vesc_control/VescSocketCAN.h>
#include <robot_parameter_wrapper/robotparameterwrapper.h>


RobotParameterWrapper::RobotParameterWrapper (uint8_t VESC_ID, float transmission_ratio, float output_ratio, float velocity_liimit, float torque_limit,char * can_network)
{
  this->vesc = new Vesc (can_network, VESC_ID);
  this->transmission_ratio = transmission_ratio;
  this->output_ratio = output_ratio;
  this->torque_limit = torque_limit;
  this->velocity_limit = velocity_limit;
}

void RobotParameterWrapper::setOutputRatio (float output_ratio){
  this->output_ratio = output_ratio;
}

void RobotParameterWrapper::setTransmissionRatio (float transmission_ratio){
  this->transmission_ratio = transmission_ratio;
}

void RobotParameterWrapper::setLinearVelocity (float meters_per_second){
  float rpm = meters_per_second / (this->output_ratio*this->transmission_ratio);  
  this->vesc->setRpm (rpm);
}

void RobotParameterWrapper::setTorque (float newton_meters){
  this->vesc->setCurrent (newton_meters);
}

void RobotParameterWrapper::setTorqueLimit (float newton_meters){
  this->torque_limit = newton_meters;
}

void RobotParameterWrapper::setLinearVelocityLimit (float meters_per_second){
  this->velocity_limit = meters_per_second;
}

