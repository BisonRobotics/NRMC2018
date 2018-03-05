
#include "dig_dump_action/dig_dump_action.h"

DigDumpAction::DigDumpAction(BackhoeController *backhoe, BucketController *bucket)
  : dig_as_(nh_, "dig_server", boost::bind(&DigDumpAction::digExecuteCB, this, _1), false)
  , dump_as_(nh_, "dump_server", boost::bind(&DigDumpAction::dumpExecuteCB, this, _1), false)
{
  this->is_digging = false;
  this->is_dumping = false;
  this->digging_state = dig_state_enum::dig_idle;
  this->dumping_state = dump_state_enum::dump_idle;
  this->dig_as_.start();
  this->dump_as_.start();
  this->bucket = bucket;
  this->backhoe = backhoe;
}

void DigDumpAction::digExecuteCB(const dig_control::DigGoalConstPtr &goal)
{
  ros::Rate(50);
  if (is_dumping)
  {
    dig_as_.setAborted();
  }
  else
  {
    is_digging = true;
    while (ros::ok() && is_digging)
    {
      switch (digging_state)
      {
        case dig_state_enum::dig_idle:
          break;
        case dig_state_enum::moving_to_setpoint:
          break;
        case dig_state_enum::curling_backhoe:
          break;
        case dig_state_enum::moving_arm_to_initial:
          break;
        case dig_state_enum::dumping_into_bucket:
          break;
        default:
          dig_as_.setAborted();
          is_digging = false;
          break;
          // hell
      }
    }
  }
}

void DigDumpAction::dumpExecuteCB(const dig_control::DumpGoalConstPtr &goal)
{
  if (is_digging)
  {
    dump_as_.setAborted();
  }
  else
  {
    is_dumping = true;
    while (ros::ok() && is_dumping)
    {
      switch (dumping_state)
      {
        case dump_state_enum::dump_idle:
          break;
        case dump_state_enum::moving_bucket_to_setpoint:
          break;
        case dump_state_enum::actuating_conveyor:
          break;
        case dump_state_enum::moving_bucket_to_initial:
          break;
        default:
          is_dumping = false;
          dump_as_.setAborted();
          break;
          // hell
      }
    }
  }
}