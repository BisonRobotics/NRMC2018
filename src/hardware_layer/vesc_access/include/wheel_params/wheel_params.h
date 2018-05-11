#ifndef WHEEL_PARAMS_H
#define WHEEL_PARAMS_H

#include "vesc_access/ivesc_access.h"
#include "safety_vesc/isafety_controller.h"

#define FRONT_LEFT_WHEEL_ID 1
#define FRONT_RIGHT_WHEEL_ID 2
#define BACK_RIGHT_WHEEL_ID 3
#define BACK_LEFT_WHEEL_ID 4
#define CENTRAL_DRIVE_ID 5
#define LINEAR_MOTOR_ID 6
#define VIBRATOR_MOTOR_ID 7
#define SMALL_CONVEYOR_ID 8
#define LARGE_CONVEYOR_ID 9
#define LEFT_OUTRIGGER_ID 10
#define RIGHT_OUTRIGGER_ID 11

#define WHEEL_CAN_NETWORK ("can0")

#define MAX_WHEEL_VELOCITY .5f
#define MAX_WHEEL_TORQUE 176.0f
#define MAX_WHEEL_DUTY .4f
#define WHEEL_GEAR_RATIO 256.667f
#define WHEEL_OUTPUT_RATIO (.03048f)
#define WHEEL_POLE_PAIRS 10
#define WHEEL_TORQUE_CONSTANT .02120f

#define MAX_CENTRAL_DRIVE_VELOCITY 0.9425f  // in rad/s
#define MAX_CENTRAL_DRIVE_TORQUE 10.0f     // in Nm -> the real number is 675 but we should never approach that
#define MAX_CENTRAL_DRIVE_DUTY .4f
#define CENTRAL_DRIVE_GEAR_RATIO -1.0f
#define CENTRAL_DRIVE_OUTPUT_RATIO 1.0f  //  purely rotational
#define CENTRAL_DRIVE_POLE_PAIRS 5
#define CENTRAL_DRIVE_TORQUE_CONSTANT 1.0f  // Nm/A

#define MAX_LINEAR_ACTUATOR_VELOCITY .05f  // this should be change
#define MAX_LINEAR_ACTUATOR_TORQUE 4.0f
#define MAX_LINEAR_DUTY .4
#define LINEAR_ACTUATOR_GEAR_RATIO 1
#define LINEAR_ACTUATOR_OUTPUT_RATIO 1.0/(5.206E6)  // this should be the pitch of the screw in m/s
#define LINEAR_ACTUATOR_POLE_PAIRS 1
#define LINEAR_ACTUATOR_TORQUE_CONSTANT 1.0f

#define MAX_SMALL_CONVEYOR_VELOCITY 4900.0f
#define MAX_SMALL_CONVEYOR_TORQUE 10.0f
#define MAX_SMALL_CONVEYOR_DUTY .9f
#define SMALL_CONVEYOR_GEAR_RATIO 1.0f
#define SMALL_CONVEYOR_OUTPUT_RATIO 1.0f
#define SMALL_CONVEYOR_POLE_PAIRS 1
#define SMALL_CONVEYOR_TORQUE_CONSTANT 1.0f

#define MAX_LARGE_CONVEYOR_VELOCITY 140000.0f // in erpm
#define MAX_LARGE_CONVEYOR_TORQUE 15.0f
#define MAX_LARGE_CONVEYOR_DUTY .7f
#define LARGE_CONVEYOR_GEAR_RATIO 1.0f
#define LARGE_CONVEYOR_OUTPUT_RATIO 1.0f
#define LARGE_CONVEYOR_POLE_PAIRS 1
#define LARGE_CONVEYOR_TORQUE_CONSTANT 1.0f

#define MAX_VIBRATOR_VELOCITY 6000.0f   // in erpm
#define MAX_VIBRATOR_TORQUE 11.0f    // in Amps
#define MAX_VIBRATOR_DUTY .5f
#define VIBRATOR_GEAR_RATIO 1.0f
#define VIBRATOR_OUTPUT_RATIO 1.0f
#define VIBRATOR_POLE_PAIRS 1
#define VIBRATOR_TORQUE_CONSTANT 1.0f

#define MAX_OUTRIGGER_VELOCITY 10.0f
#define MAX_OUTRIGGER_TORQUE 10.0f
#define MAX_OUTRIGGER_DUTY .4f
#define OUTRIGGER_GEAR_RATIO 1.0f
#define OUTRIGGER_OUTPUT_RATIO 1.0f
#define OUTRIGGER_POLE_PAIRS 1
#define OUTRIGGER_TORQUE_CONSTANT 1.0f

#define NUMBER_OF_MOTORS 11
#define ROBOT_AXLE_LENGTH 0.64f
#define ROBOT_MAX_SPEED 0.5f

#define LINEAR_ACTUATOR_LENGTH .186
#define MINIMUM_CENTRAL_ANGLE 0
#define MAXIMUM_CENTRAL_ANGLE 2.96
#define SAFE_CENTRAL_ANGLE 2.55
#define SAFE_LINEAR_DISTANCE .04985

#define LINEAR_RETRACTED_POINT .03
#define LINEAR_EXTENDED_POINT .175
#define CENTRAL_MEASUREMENT_START_ANGLE 2.0
#define CENTRAL_MEASUREMENT_STOP_ANGLE 1.5
#define CENTRAL_HOLD_TORQUE -1          //increase the magintude
#define CENTRAL_TRANSPORT_ANGLE 2.65                 // move this up
#define CENTRAL_MOVE_ROCKS_INTO_HOPPER_ANGLE  2.9 // move this up
#define CENTRAL_DUMP_ANGLE 2.5        // must be below safety point, where backhoe dumps into bucket
#define CENTRAL_DEPOSITION_ANGLE 2.95  // must be below max position

nsVescAccess::vesc_param_struct_t front_left_param = {.max_velocity = MAX_WHEEL_VELOCITY,
                                                      .max_torque = MAX_WHEEL_TORQUE,
                                                      .max_duty = MAX_WHEEL_DUTY,
                                                      .gear_ratio = WHEEL_GEAR_RATIO,
                                                      .output_ratio = WHEEL_OUTPUT_RATIO,
                                                      .pole_pairs = WHEEL_POLE_PAIRS,
                                                      .torque_constant = WHEEL_TORQUE_CONSTANT,
                                                      WHEEL_CAN_NETWORK,
                                                      .can_id = FRONT_LEFT_WHEEL_ID,
                                                      .name = "front_left_wheel" };

nsVescAccess::vesc_param_struct_t front_right_param = {.max_velocity = MAX_WHEEL_VELOCITY,
                                                       .max_torque = MAX_WHEEL_TORQUE,
                                                       .max_duty = MAX_WHEEL_DUTY,
                                                       .gear_ratio = WHEEL_GEAR_RATIO,
                                                       .output_ratio = -1.0f * WHEEL_OUTPUT_RATIO,
                                                       .pole_pairs = WHEEL_POLE_PAIRS,
                                                       .torque_constant = WHEEL_TORQUE_CONSTANT,
                                                       WHEEL_CAN_NETWORK,
                                                       .can_id = FRONT_RIGHT_WHEEL_ID,
                                                       .name = "front_right_wheel" };

nsVescAccess::vesc_param_struct_t back_right_param = {.max_velocity = MAX_WHEEL_VELOCITY,
                                                      .max_torque = MAX_WHEEL_TORQUE,
                                                      .max_duty = MAX_WHEEL_DUTY,
                                                      .gear_ratio = WHEEL_GEAR_RATIO,
                                                      .output_ratio = -1.0f * WHEEL_OUTPUT_RATIO,
                                                      .pole_pairs = WHEEL_POLE_PAIRS,
                                                      .torque_constant = WHEEL_TORQUE_CONSTANT,
                                                      WHEEL_CAN_NETWORK,
                                                      .can_id = BACK_RIGHT_WHEEL_ID,
                                                      .name = "back_right_wheel" };

nsVescAccess::vesc_param_struct_t back_left_param = {.max_velocity = MAX_WHEEL_VELOCITY,
                                                     .max_torque = MAX_WHEEL_TORQUE,
                                                     .max_duty = MAX_WHEEL_DUTY,
                                                     .gear_ratio = WHEEL_GEAR_RATIO,
                                                     .output_ratio =  WHEEL_OUTPUT_RATIO,
                                                     .pole_pairs = WHEEL_POLE_PAIRS,
                                                     .torque_constant = WHEEL_TORQUE_CONSTANT,
                                                     WHEEL_CAN_NETWORK,
                                                     .can_id = BACK_LEFT_WHEEL_ID,
                                                     .name = "back_left_wheel" };

nsVescAccess::vesc_param_struct_t sifter_param = {.max_velocity = MAX_VIBRATOR_VELOCITY,
                                                  .max_torque = MAX_VIBRATOR_TORQUE,
                                                  .max_duty = MAX_VIBRATOR_DUTY,
                                                  .gear_ratio = VIBRATOR_GEAR_RATIO,
                                                  .output_ratio = VIBRATOR_OUTPUT_RATIO,
                                                  .pole_pairs = VIBRATOR_POLE_PAIRS,
                                                  .torque_constant = VIBRATOR_TORQUE_CONSTANT,
                                                  WHEEL_CAN_NETWORK,
                                                  .can_id = VIBRATOR_MOTOR_ID,
                                                  .name = "sifter" };

nsVescAccess::vesc_param_struct_t small_conveyor_param = {.max_velocity = MAX_SMALL_CONVEYOR_VELOCITY,
                                                          .max_torque = MAX_SMALL_CONVEYOR_TORQUE,
                                                          .max_duty = MAX_SMALL_CONVEYOR_DUTY,
                                                          .gear_ratio = SMALL_CONVEYOR_GEAR_RATIO,
                                                          .output_ratio = SMALL_CONVEYOR_OUTPUT_RATIO,
                                                          .pole_pairs = SMALL_CONVEYOR_POLE_PAIRS,
                                                          .torque_constant = SMALL_CONVEYOR_TORQUE_CONSTANT,
                                                          WHEEL_CAN_NETWORK,
                                                          .can_id = SMALL_CONVEYOR_ID,
                                                          .name = "small_conveyor"};

nsVescAccess::vesc_param_struct_t large_conveyor_param = {.max_velocity = MAX_LARGE_CONVEYOR_VELOCITY,
                                                          .max_torque = MAX_LARGE_CONVEYOR_TORQUE,
                                                          .max_duty = MAX_LARGE_CONVEYOR_DUTY,
                                                          .gear_ratio = LARGE_CONVEYOR_GEAR_RATIO,
                                                          .output_ratio = LARGE_CONVEYOR_OUTPUT_RATIO,
                                                          .pole_pairs = LARGE_CONVEYOR_POLE_PAIRS,
                                                          .torque_constant = LARGE_CONVEYOR_TORQUE_CONSTANT,
                                                          WHEEL_CAN_NETWORK,
                                                          .can_id = LARGE_CONVEYOR_ID,
                                                          .name = "large_conveyor"};

nsVescAccess::vesc_param_struct_t left_outrigger_param = {.max_velocity = MAX_OUTRIGGER_VELOCITY,
                                                          .max_torque = MAX_OUTRIGGER_TORQUE,
                                                          .max_duty = MAX_OUTRIGGER_DUTY,
                                                          .gear_ratio = OUTRIGGER_GEAR_RATIO,
                                                          .output_ratio = OUTRIGGER_OUTPUT_RATIO,
                                                          .pole_pairs = OUTRIGGER_POLE_PAIRS,
                                                          .torque_constant = OUTRIGGER_TORQUE_CONSTANT,
                                                          WHEEL_CAN_NETWORK,
                                                          .can_id = LEFT_OUTRIGGER_ID,
                                                          .name = "left_outrigger"};

nsVescAccess::vesc_param_struct_t right_outrigger_param = {.max_velocity = MAX_OUTRIGGER_VELOCITY,
                                                           .max_torque = MAX_OUTRIGGER_TORQUE,
                                                           .max_duty = MAX_OUTRIGGER_DUTY,
                                                           .gear_ratio = OUTRIGGER_GEAR_RATIO,
                                                           .output_ratio = OUTRIGGER_OUTPUT_RATIO,
                                                           .pole_pairs = OUTRIGGER_POLE_PAIRS,
                                                           .torque_constant = OUTRIGGER_TORQUE_CONSTANT,
                                                           WHEEL_CAN_NETWORK,
                                                           .can_id = RIGHT_OUTRIGGER_ID,
                                                           .name = "right_outrigger"};

nsVescAccess::vesc_param_struct_t linear_param = {.max_velocity = MAX_LINEAR_ACTUATOR_VELOCITY,
                                                  .max_torque = MAX_LINEAR_ACTUATOR_TORQUE,
                                                  .max_duty = MAX_LINEAR_DUTY,
                                                  .gear_ratio = LINEAR_ACTUATOR_GEAR_RATIO,
                                                  .output_ratio = LINEAR_ACTUATOR_OUTPUT_RATIO,
                                                  .pole_pairs = LINEAR_ACTUATOR_POLE_PAIRS,
                                                  .torque_constant = LINEAR_ACTUATOR_TORQUE_CONSTANT,
                                                  WHEEL_CAN_NETWORK,
                                                  .can_id = LINEAR_MOTOR_ID,
                                                  .name = "linear_actuator" };

nsVescAccess::vesc_param_struct_t shoulder_param = {.max_velocity = MAX_CENTRAL_DRIVE_VELOCITY,
                                                    .max_torque = MAX_CENTRAL_DRIVE_TORQUE,
                                                    .max_duty = MAX_CENTRAL_DRIVE_DUTY,
                                                    .gear_ratio = CENTRAL_DRIVE_GEAR_RATIO,
                                                    .output_ratio = CENTRAL_DRIVE_OUTPUT_RATIO,
                                                    .pole_pairs = CENTRAL_DRIVE_POLE_PAIRS,
                                                    .torque_constant = CENTRAL_DRIVE_TORQUE_CONSTANT,
                                                    WHEEL_CAN_NETWORK,
                                                    .can_id = CENTRAL_DRIVE_ID,
                                                    .name = "central_drive" };

safetycontroller::joint_params_t linear_joint_params = {.minimum_pos = 0,
                                                        .maximum_pos = LINEAR_ACTUATOR_LENGTH,
                                                        .safety_check_pos = SAFE_LINEAR_DISTANCE,
                                                        .gain = .11,
                                                        .setpoint_tolerance = .005,
                                                        .lower_limit_position = .0286,
                                                        .upper_limit_position = LINEAR_ACTUATOR_LENGTH,
                                                        .max_abs_velocity = MAX_LINEAR_ACTUATOR_VELOCITY,
                                                        .limit_switch_safety_margin = .001,
                                                        .max_abs_torque = MAX_LINEAR_ACTUATOR_TORQUE,
                                                        .name = "linear" };

safetycontroller::joint_params_t central_joint_params = {.minimum_pos = MINIMUM_CENTRAL_ANGLE,
                                                         .maximum_pos = MAXIMUM_CENTRAL_ANGLE,
                                                         .safety_check_pos = SAFE_CENTRAL_ANGLE,
                                                         .gain = .15,
                                                         .setpoint_tolerance = 0.04,
                                                         .lower_limit_position = MINIMUM_CENTRAL_ANGLE,
                                                         .upper_limit_position = MAXIMUM_CENTRAL_ANGLE,
                                                         .max_abs_velocity = .2,
                                                         .limit_switch_safety_margin = .01,
                                                         .max_abs_torque = MAX_CENTRAL_DRIVE_TORQUE,
                                                         .name = "central" };
#endif