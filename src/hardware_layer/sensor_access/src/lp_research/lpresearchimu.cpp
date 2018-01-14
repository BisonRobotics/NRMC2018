#include "lp_research/lpresearchimu.h"


LpResearchImu::LpResearchImu (std::string topic) : nh_(){
  this->sub = this->nh_.subscribe(topic, 100, &LpResearchImu::imu_callback, this);
  x_acc = 0.0f;
  y_acc = 0.0f;
  omega = 0.0f;
  is_data_valid = false;
}


float LpResearchImu::getX (void){
  return x_acc;
}

float LpResearchImu::getY (void){
  return y_acc;
}


float LpResearchImu::getOmega(void){
  return omega;
}

ReadableSensors::ReadStatus LpResearchImu::receiveData(){
  if (is_data_valid){
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  } else {
    return ReadableSensors::ReadStatus::READ_FAILED;
  }
}

void LpResearchImu::imu_callback (const sensor_msgs::Imu::ConstPtr &msg){
  x_acc =(float) msg->linear_acceleration.x;
  y_acc = (float) msg->linear_acceleration.y;
  omega = (float) msg->angular_velocity.z;
  is_data_valid = true;
}
