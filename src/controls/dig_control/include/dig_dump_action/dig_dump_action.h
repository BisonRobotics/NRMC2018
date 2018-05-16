#ifndef PROJECT_DIG_DUMP_ACTION_H
#define PROJECT_DIG_DUMP_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DumpAction.h>
#include <dig_control/DigAction.h>
#include <bucket_controller/bucket_controller.h>
#include <backhoe_controller/backhoe_controller.h>

#include <std_msgs/Empty.h>


enum dig_state_enum
{
  dig_idle,
  ensure_at_measurement_start,
  moving_to_ground,
  finding_ground,
  curling_backhoe,
  moving_arm_to_initial,
  dumping_into_bucket,
  moving_rocks_into_holder,
  waiting_for_rocks,
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
  DigDumpAction(BackhoeController *backhoe, BucketController *bucket, int argc, char **argv);
  dig_state_enum digging_state;
  dump_state_enum dumping_state;

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<dig_control::DumpAction> dump_as_;
  actionlib::SimpleActionServer<dig_control::DigAction> dig_as_;
  ros::Publisher scoot_back_pub;
  dig_control::DigFeedback dig_feedback;
  dig_control::DigResult dig_result;
  dig_control::DumpFeedback dump_feedback;
  dig_control::DumpResult dump_result;
  double initial_dig_time;
  bool is_digging;
  bool is_dumping;
  double initial_time;
  static constexpr float dump_time = 90;
  static constexpr float time_to_move_rocks_to_holder=1;
  void digExecuteCB(const dig_control::DigGoalConstPtr &goal);
  void dumpExecuteCB(const dig_control::DumpGoalConstPtr &goal);
  BackhoeController *backhoe;
  BucketController *bucket;
  double weightMetric;
  double ground_metric;
  double prev_backhoe_position;
};

#endif  // PROJECT_DIG_DUMP_ACTION_H
