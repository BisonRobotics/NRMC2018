#ifndef WHEEL_PARAMS_H
#define WHEEL_PARAMS_H
#include "vesc_access/ivesc_access.h"
#define FRONT_LEFT_WHEEL_ID     4
#define FRONT_RIGHT_WHEEL_ID    1
#define BACK_RIGHT_WHEEL_ID     2
#define BACK_LEFT_WHEEL_ID      3
#define CENTRAL_DRIVE_ID        5
#define LINEAR_MOTOR_ID         6
#define VIBRATOR_MOTOR_ID       7
#define SMALL_CONVEYOR_ID       8
#define LARGE_CONVEYOR_ID       9
#define LEFT_OUTRIGGER_ID       10
#define RIGHT_OUTRIGGER_ID      11

#define WHEEL_CAN_NETWORK ("can0")

#define MAX_WHEEL_VELOCITY .5f
#define MAX_WHEEL_TORQUE 176.0f
#define WHEEL_GEAR_RATIO 181.4f
#define WHEEL_OUTPUT_RATIO .03048f
#define WHEEL_POLE_PAIRS 1
#define WHEEL_TORQUE_CONSTANT 4.0f

#define MAX_CENTRAL_DRIVE_VELOCITY  0.9425f // in rad/s
#define MAX_CENTRAL_DRIVE_TORQUE    300.0f     // in Nm -> the real number is 675 but we should never approach that
#define CENTRAL_DRIVE_GEAR_RATIO    540.0f
#define CENTRAL_DRIVE_OUTPUT_RATIO  1.0f        //  purely rotational
#define CENTRAL_DRIVE_POLE_PAIRS    14
#define CENTRAL_DRIVE_TORQUE_CONSTANT   .0641f   // Nm/A

#define MAX_LINEAR_ACTUATOR_VELOCITY 560.0f     // this should be change
#define MAX_LINEAR_ACTUATOR_TORQUE      10.0f
#define LINEAR_ACTUATOR_GEAR_RATIO      15.0f
#define LINEAR_ACTUATOR_OUTPUT_RATIO    1.0f    // this should be the pitch of the screw
#define LINEAR_ACTUATOR_POLE_PAIRS      14      // update this
#define LINEAR_ACTUATOR_TORQUE_CONSTANT .0341f

#define MAX_SMALL_CONVEYOR_VELOCITY 4900.0f
#define MAX_SMALL_CONVEYOR_TORQUE   .24f
#define SMALL_CONVEYOR_GEAR_RATIO    1.0f
#define SMALL_CONVEYOR_OUTPUT_RATIO  1.0f
#define SMALL_CONVEYOR_POLE_PAIRS   7
#define SMALL_CONVEYOR_TORQUE_CONSTANT .0367f

#define MAX_LARGE_CONVEYOR_VELOCITY 230.0f
#define MAX_LARGE_CONVEYOR_TORQUE    15.0f
#define LARGE_CONVEYOR_GEAR_RATIO   21.0f
#define LARGE_CONVEYOR_OUTPUT_RATIO 1.0f
#define LARGE_CONVEYOR_POLE_PAIRS   7
#define LARGE_CONVEYOR_TORQUE_CONSTANT  .0341f

#define MAX_VIBRATOR_VELOCITY   6000.0f
#define MAX_VIBRATOR_TORQUE     .13f
#define VIBRATOR_GEAR_RATIO     1.0f
#define VIBRATOR_OUTPUT_RATIO   1.0f
#define VIBRATOR_POLE_PAIRS     1
#define VIBRATOR_TORQUE_CONSTANT    .0367f

#define MAX_OUTRIGGER_VELOCITY  10.0f
#define MAX_OUTRIGGER_TORQUE    10.0f
#define OUTRIGGER_GEAR_RATIO    1.0f
#define OUTRIGGER_OUTPUT_RATIO  1.0f
#define OUTRIGGER_POLE_PAIRS    1
#define OUTRIGGER_TORQUE_CONSTANT   1.0f


nsVescAccess::vesc_param_struct_t front_left_param =
    {
        .max_velocity = MAX_WHEEL_VELOCITY,
        .max_torque = MAX_WHEEL_TORQUE,
        .gear_ratio = WHEEL_GEAR_RATIO,
        .output_ratio = WHEEL_OUTPUT_RATIO,
        .pole_pairs = WHEEL_POLE_PAIRS,
        .torque_constant = WHEEL_TORQUE_CONSTANT,
         WHEEL_CAN_NETWORK,
        .can_id = FRONT_LEFT_WHEEL_ID
    };

nsVescAccess::vesc_param_struct_t front_right_param =
    {
        .max_velocity = MAX_WHEEL_VELOCITY,
        .max_torque = MAX_WHEEL_TORQUE,
        .gear_ratio = WHEEL_GEAR_RATIO,
        .output_ratio = -1.0f*WHEEL_OUTPUT_RATIO,
        .pole_pairs = WHEEL_POLE_PAIRS,
        .torque_constant = WHEEL_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = FRONT_RIGHT_WHEEL_ID
    };

nsVescAccess::vesc_param_struct_t back_right_param =
   {
        .max_velocity = MAX_WHEEL_VELOCITY,
        .max_torque = MAX_WHEEL_TORQUE,
        .gear_ratio = WHEEL_GEAR_RATIO,
        .output_ratio = -1.0f*WHEEL_OUTPUT_RATIO,
        .pole_pairs = WHEEL_POLE_PAIRS,
        .torque_constant = WHEEL_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = BACK_RIGHT_WHEEL_ID
   };

nsVescAccess::vesc_param_struct_t back_left_param =
   {
        .max_velocity = MAX_WHEEL_VELOCITY,
        .max_torque = MAX_WHEEL_TORQUE,
        .gear_ratio = WHEEL_GEAR_RATIO,
        .output_ratio = WHEEL_OUTPUT_RATIO,
        .pole_pairs = WHEEL_POLE_PAIRS,
        .torque_constant = WHEEL_TORQUE_CONSTANT,
         WHEEL_CAN_NETWORK,
        .can_id = BACK_LEFT_WHEEL_ID
   };

nsVescAccess::vesc_param_struct_t sifter_param =
   {
        .max_velocity = MAX_VIBRATOR_VELOCITY,
        .max_torque = MAX_VIBRATOR_TORQUE,
        .gear_ratio = VIBRATOR_GEAR_RATIO,
        .output_ratio = VIBRATOR_OUTPUT_RATIO,
        .pole_pairs = VIBRATOR_POLE_PAIRS,
        .torque_constant = VIBRATOR_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = VIBRATOR_MOTOR_ID
   };

nsVescAccess::vesc_param_struct_t small_conveyor_param =
   {
        .max_velocity = MAX_SMALL_CONVEYOR_VELOCITY,
        .max_torque = MAX_SMALL_CONVEYOR_TORQUE,
        .gear_ratio = SMALL_CONVEYOR_GEAR_RATIO,
        .output_ratio = SMALL_CONVEYOR_OUTPUT_RATIO,
        .pole_pairs = SMALL_CONVEYOR_POLE_PAIRS,
        .torque_constant = VIBRATOR_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = SMALL_CONVEYOR_ID
   };

nsVescAccess::vesc_param_struct_t large_conveyor_param =
   {
        .max_velocity = MAX_LARGE_CONVEYOR_VELOCITY,
        .max_torque = MAX_LARGE_CONVEYOR_TORQUE,
        .gear_ratio = LARGE_CONVEYOR_GEAR_RATIO,
        .output_ratio = LARGE_CONVEYOR_OUTPUT_RATIO,
        .pole_pairs = LARGE_CONVEYOR_POLE_PAIRS,
        .torque_constant = LARGE_CONVEYOR_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = LARGE_CONVEYOR_ID
   };

nsVescAccess::vesc_param_struct_t left_outrigger_param =
   {
        .max_velocity = MAX_OUTRIGGER_VELOCITY,
        .max_torque = MAX_OUTRIGGER_TORQUE,
        .gear_ratio = OUTRIGGER_GEAR_RATIO,
        .output_ratio = OUTRIGGER_OUTPUT_RATIO,
        .pole_pairs = OUTRIGGER_POLE_PAIRS,
        .torque_constant = OUTRIGGER_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = LEFT_OUTRIGGER_ID
   };

nsVescAccess::vesc_param_struct_t right_outrigger_param =
   {
        .max_velocity = MAX_OUTRIGGER_VELOCITY,
        .max_torque = MAX_OUTRIGGER_TORQUE,
        .gear_ratio = OUTRIGGER_GEAR_RATIO,
        .output_ratio = OUTRIGGER_OUTPUT_RATIO,
        .pole_pairs = OUTRIGGER_POLE_PAIRS,
        .torque_constant = OUTRIGGER_TORQUE_CONSTANT,
         WHEEL_CAN_NETWORK,
        .can_id = RIGHT_OUTRIGGER_ID
   };

nsVescAccess::vesc_param_struct_t linear_param =
    {
        .max_velocity = MAX_LINEAR_ACTUATOR_VELOCITY,
        .max_torque = MAX_LINEAR_ACTUATOR_TORQUE,
        .gear_ratio = LINEAR_ACTUATOR_GEAR_RATIO,
        .output_ratio = LINEAR_ACTUATOR_OUTPUT_RATIO,
        .pole_pairs = LINEAR_ACTUATOR_POLE_PAIRS,
        .torque_constant = LINEAR_ACTUATOR_TORQUE_CONSTANT,
         WHEEL_CAN_NETWORK,
        .can_id = LINEAR_MOTOR_ID
    };

nsVescAccess::vesc_param_struct_t shoulder_param =
    {
        .max_velocity = MAX_CENTRAL_DRIVE_VELOCITY,
        .max_torque = MAX_CENTRAL_DRIVE_TORQUE,
        .gear_ratio = CENTRAL_DRIVE_GEAR_RATIO,
        .output_ratio = CENTRAL_DRIVE_OUTPUT_RATIO,
        .pole_pairs = CENTRAL_DRIVE_POLE_PAIRS,
        .torque_constant = CENTRAL_DRIVE_TORQUE_CONSTANT,
        WHEEL_CAN_NETWORK,
        .can_id = CENTRAL_DRIVE_ID
    };

#define ROBOT_AXLE_LENGTH 0.5f
#define ROBOT_MAX_SPEED 0.5f
#define LINEAR_ACTUATOR_LENGTH .1778
#define MINIMUM_CENTRAL_ANGLE  -1.4
#define MAXIMUM_CENTRAL_ANGLE  1.3
#define SAFE_CENTRAL_ANGLE      -1.22
#define SAFE_LINEAR_DISTANCE    .06985

#endif