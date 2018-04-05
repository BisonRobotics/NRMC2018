#ifndef PROJECT_DIG_DUMP_ACTION_H
#define PROJECT_DIG_DUMP_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DumpAction.h>
#include <dig_control/DigAction.h>
#include <bucket_controller/bucket_controller.h>
#include <backhoe_controller/backhoe_controller.h>

enum dig_state_enum
{
  dig_idle,
  ensure_at_measurement_start,
  moving_to_ground,
  finding_ground,
  curling_backhoe,
  moving_arm_to_initial,
  dumping_into_bucket,
  returning_backhoe_to_initial,
  dig_error
};

enum dump_state_enum
{
  dump_idle,
  moving_bucket_to_setpoint,
  actuating_conveyor,
  moving_bucket_to_initial,
  dump_error
};

class DigDumpAction
{
public:
  DigDumpAction(BackhoeController *backhoe, BucketController *bucket);

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<dig_control::DumpAction> dump_as_;
  actionlib::SimpleActionServer<dig_control::DigAction> dig_as_;
  dig_control::DigFeedback dig_feedback;
  dig_control::DigResult dig_result;
  dig_control::DumpFeedback dump_feedback;
  dig_control::DumpResult dump_result;
  bool is_digging;
  bool is_dumping;
  dig_state_enum digging_state;
  dump_state_enum dumping_state;
  void digExecuteCB(const dig_control::DigGoalConstPtr &goal);
  void dumpExecuteCB(const dig_control::DumpGoalConstPtr &goal);
  BackhoeController *backhoe;
  BucketController *bucket;
  double weightMetric;
};

#endif  // PROJECT_DIG_DUMP_ACTION_H
