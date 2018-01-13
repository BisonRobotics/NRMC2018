#include "lp_research/lpresearchimu.h"


LpResearchImu::LpResearchImu (std::string topic) : nh_(){
  this->sub = this->nh_.subscribe(topic, 100, &LpResearchImu::imu_callback, this);
  x_acc = 0.0f;
  y_acc = 0.0f;
  alpha = 0.0f;
  is_data_valid = false;
}


float LpResearchImu::getX (void){
  return x_acc;
}

float LpResearchImu::getY (void){
  return y_acc;
}


float LpResearchImu::getAlpha(void){
  return alpha;
}

ReadableSensors::ReadStatus LpResearchImu::receiveData(){
  if (is_data_valid){
    return ReadableSensors::ReadStatus::READ_SUCCESS;
  } else {
    return ReadableSensors::ReadStatus::READ_FAILED;
  }
}

void LpResearchImu::imu_callback (const sensor_msgs::Imu::ConstPtr &msg){
  x_acc = msg->linear_acceleration.x;
  y_acc = msg->linear_acceleration.y;
  alpha = msg->linear_acceleration.z;
  is_data_valid = true;
}
