
#include "dig_dump_action/dig_dump_action.h"
#include <std_msgs/Empty.h>

#include <wheel_params/wheel_params.h>


#define GROUND_ALPHA .1

DigDumpAction::DigDumpAction(BackhoeController *backhoe, BucketController *bucket, int argc, char **argv)
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
  weightMetric = 0;
  this->ground_metric = 10;
  ros::init(argc, argv, "the_dig_control_server");
  scoot_back_pub = nh_.advertise<std_msgs::Empty>("scoot_back", 30);
}

void DigDumpAction::digExecuteCB(const dig_control::DigGoalConstPtr &goal)
{
  ros::Rate r(50);
  if (is_dumping)
  {
    dig_as_.setAborted();
  }
  else
  {
    is_digging = true;
    // digging_state = dig_state_enum::dig_idle;
    while (ros::ok() && is_digging)
    {
      r.sleep();
      switch (digging_state)
      {
        case dig_state_enum::dig_idle:  // state 0//not digging, should start here
          bucket->turnSifterOn();
          bucket->turnLittleConveyorOn();
          backhoe->setShoulderSetpoint(CENTRAL_MEASUREMENT_START_ANGLE);  // put system in known starting config
          backhoe->setWristSetpoint(LINEAR_RETRACTED_POINT);
          digging_state = ensure_at_measurement_start;
          break;
        case dig_state_enum::ensure_at_measurement_start:  // state 1
          if (backhoe->shoulderAtSetpoint() && backhoe->wristAtSetpoint())
          {
            backhoe->setShoulderSetpoint(CENTRAL_MEASUREMENT_STOP_ANGLE);  // take it to close to ground
            digging_state = moving_to_ground;
            //weightMetric = 0;
            weightMetric += 3.2;
          }
          break;
        case dig_state_enum::moving_to_ground:  // state 2
          // integrate work moving from measurement start to where we think the ground is
          //weightMetric += 1.0;//backhoe->getShoulderTorque() * backhoe->getShoulderVelocity() * .02;  //.02 is dt
          ground_metric = 10;
          if (backhoe->shoulderAtSetpoint())
          {
            backhoe->setShoulderSetpoint(0);  // send it into the ground
            digging_state = finding_ground;
            dig_result.weight_harvested = weightMetric;// / 100;
            prev_backhoe_position = 3*CENTRAL_MEASUREMENT_STOP_ANGLE;
          }
          break;
        case dig_state_enum::finding_ground:  // state 3 //going to find the ground
          // this method checks the torque reported by the motor
          // it waits for the p gain on the velocity control to increase the torque above
          // a threshold after the RPM drops when the ground is hit
          ground_metric = GROUND_ALPHA * (prev_backhoe_position - backhoe->getPositionEstimate())/.02 
                    + (1 - GROUND_ALPHA) * ground_metric;
          prev_backhoe_position = backhoe->getPositionEstimate();
          if (ground_metric < .05 /*backhoe->hasHitGround()*/)
          {
            backhoe->abandonShoulderPositionSetpointAndSetTorqueWithoutStopping(CENTRAL_HOLD_TORQUE);
            backhoe->setWristSetpoint(LINEAR_EXTENDED_POINT);  // curl it in
            digging_state = curling_backhoe;
          }
          break;
        case dig_state_enum::curling_backhoe:  // state 4 //curling wrist into dirt
          if (backhoe->wristAtSetpoint())
          {
            backhoe->setShoulderSetpoint(CENTRAL_DUMP_ANGLE);  // lift to angle appropiate for dropping dirt into bucket
            digging_state = moving_arm_to_initial;
          }
          break;
        case dig_state_enum::moving_arm_to_initial:  // state 5 //lifting dug dirt up
          if (backhoe->shoulderAtSetpoint())
          {
            backhoe->setWristSetpoint(LINEAR_RETRACTED_POINT);  // curl it out
            digging_state = dumping_into_bucket;
          }
          break;
        case dig_state_enum::dumping_into_bucket:  // state 6 //uncurling wrist to release dirt into bucket
          if (backhoe->wristAtSetpoint()) //skip this check
          {
            backhoe->setShoulderSetpoint(CENTRAL_MOVE_ROCKS_INTO_HOPPER_ANGLE);  // wherever transit/initial should be
            digging_state = moving_rocks_into_holder;
            //publish backup thing here
            std_msgs::Empty myMsg;
            scoot_back_pub.publish(myMsg);
            //ros::spinOnce();
          }
          break;
        case dig_state_enum::moving_rocks_into_holder:  // state 7
          if (backhoe->shoulderAtSetpoint())
          {
            initial_dig_time = 0;//ros::Time::now();
            digging_state = waiting_for_rocks;
          }
          break;
        case dig_state_enum::waiting_for_rocks:  //state 8
          initial_dig_time+= .02;
          if (initial_dig_time /*(ros::Time::now()-initial_dig_time).toSec()*/ > time_to_move_rocks_to_holder)
          {
            digging_state = returning_backhoe_to_initial;
            backhoe->setShoulderSetpoint(CENTRAL_TRANSPORT_ANGLE);
            // central_transport_angle and CENTRAL_MOVE_ROCKS_INTO_HOPPER_ANGLE are so close,
            // it make sense tto get rid of this but lets see what makes sense
          }
          break;
        case dig_state_enum::returning_backhoe_to_initial:  // state 9 //moving back to same position as dig idle
          if (backhoe->shoulderAtSetpoint())
          {
            bucket->turnLittleConveyorOff();
            bucket->turnSifterOff();
            is_digging = false;
            digging_state = dig_idle;
            dig_as_.setSucceeded(dig_result);
          }
          break;
        case dig_state_enum::dig_error:
        default:
          bucket->turnSifterOff();
          bucket->turnLittleConveyorOff();
          dig_as_.setAborted();
          is_digging = false;
          break;
      }
    }
  }
}

void DigDumpAction::dumpExecuteCB(const dig_control::DumpGoalConstPtr &goal)
{
  ros::Rate r(50);
  if (is_digging)
  {
    dump_as_.setAborted();
  }
  else
  {
    is_dumping = true;
    while (ros::ok() && is_dumping)
    {
      r.sleep();
      switch (dumping_state)
      {
        case dump_state_enum::dump_idle: //state 0
          backhoe->setShoulderSetpoint(CENTRAL_DEPOSITION_ANGLE);  // move central drive to appropiate spot
          bucket->turnSifterOn();
          dumping_state = moving_bucket_to_setpoint;
          break;
        case dump_state_enum::moving_bucket_to_setpoint: //state 1
          if (backhoe->shoulderAtSetpoint())
          {
            bucket->turnBigConveyorOn();
            initial_time = 0;//ros::Time::now();
            dumping_state = actuating_conveyor;
          }
          break;
        case dump_state_enum::actuating_conveyor: //state 2
          // keep conveyor actuated for some time? until some current feedback?
          initial_time += .02;
          if(/*ros::Time::now()-initial_time).toSec()*/ initial_time > dump_time)
          {
            bucket->turnBigConveyorOff();
            bucket->turnSifterOff();
            backhoe->setShoulderSetpoint(CENTRAL_TRANSPORT_ANGLE);
            dumping_state = moving_bucket_to_initial;
          }
          break;
        case dump_state_enum::moving_bucket_to_initial:
          if (backhoe->shoulderAtSetpoint())
          {
            weightMetric = 0;
            is_dumping = false;
            dumping_state = dump_idle;
            dump_as_.setSucceeded();
          }
          break;
        case dump_state_enum::dump_error:
          break;
        default:
          is_dumping = false;
          dump_as_.setAborted();
          break;
      }
    }
  }
}
