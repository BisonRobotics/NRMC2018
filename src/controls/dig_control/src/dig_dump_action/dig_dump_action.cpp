
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
  weightMetric =0;
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
    //digging_state = dig_state_enum::dig_idle;
    while (ros::ok() && is_digging)
    {
      r.sleep();
      switch (digging_state)
      {
        case dig_state_enum::dig_idle: //state 0//not digging, should start here
          bucket->turnSifterOn();
          bucket->turnLittleConveyorOn();
          backhoe->setShoulderSetpoint(.7); //put system in known starting config
          backhoe->setWristSetpoint(.02);
          digging_state = ensure_at_measurement_start;
        break;
        case dig_state_enum::ensure_at_measurement_start: //state 1
          if (backhoe->shoulderAtSetpoint() && backhoe->wristAtSetpoint())
          {
            backhoe->setShoulderSetpoint(.01); //take it to close to ground
            digging_state = moving_to_ground;
            weightMetric =0;
          }
          break;
        case dig_state_enum::moving_to_ground: //state 2
          //integrate work moving from measurement start to where we think the ground is
          weightMetric += backhoe->getShoulderTorque() * backhoe->getShoulderVelocity() * .02; //.02 is dt
          if (backhoe->shoulderAtSetpoint())
          {
              backhoe->setShoulderSetpoint(-.2); //send it into the ground
              digging_state = finding_ground;
              dig_result.weight_harvested = weightMetric;
          }
          break;
        case dig_state_enum::finding_ground: //state 3 //going to find the ground
          //this method checks the torque reported by the motor
          //it waits for the I gain on the velocity control to increase the torque above
          //a threshold
          if (backhoe->hasHitGround())
          {
              backhoe->abandonShoulderPositionSetpointAndSetTorqueWithoutStopping(1.0f);
              backhoe->setWristSetpoint(0); //curl it in
              digging_state = curling_backhoe;
          }
          break;
        case dig_state_enum::curling_backhoe: //state 4 //curling wrist into dirt
            if (backhoe->wristAtSetpoint())
            {
                backhoe->setShoulderSetpoint(.7); //lift to angle appropiate for dropping dirt into bucket
                digging_state = moving_arm_to_initial;
            }
          break;
        case dig_state_enum::moving_arm_to_initial: //state 5 //lifting dug dirt up
            if (backhoe->shoulderAtSetpoint())
            {
              backhoe->setWristSetpoint(0); //curl it out
              digging_state = dumping_into_bucket;
            }
          break;
        case dig_state_enum::dumping_into_bucket: //state 6 //uncurling wrist to release dirt into bucket
            if (backhoe->wristAtSetpoint())
            {
                backhoe->setShoulderSetpoint(1.1); //wherever transit/initial should be
                digging_state = returning_backhoe_to_initial;
            }
          break;
        case dig_state_enum::returning_backhoe_to_initial: //state 7 //moving back to same position as dig idle
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
          bucket->turnSifterOff();
          bucket->turnLittleConveyorOff();
          break;
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
        case dump_state_enum::dump_idle:
          backhoe->setShoulderSetpoint(1); //move central drive to appropiate spot
          dumping_state = moving_bucket_to_setpoint;
          break;
        case dump_state_enum::moving_bucket_to_setpoint:
          if (backhoe->shoulderAtSetpoint())
          {
              bucket->turnBigConveyorOn();
              dumping_state = actuating_conveyor;
          }
          break;
        case dump_state_enum::actuating_conveyor:
          //keep conveyor actuated for some time? until some current feedback?
            if (/*dirt_been_dumped*/ true)
            {
              bucket->turnBigConveyorOff();
              backhoe->setShoulderSetpoint(.3);
              dumping_state = moving_bucket_to_initial;
            }
          break;
        case dump_state_enum::moving_bucket_to_initial:
            if (backhoe->shoulderAtSetpoint())
            {
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
