#include "teleop_safety/teleop_safety.h"
#include "wheel_params/wheel_params.h"
#include <cmath>

TeleopSafety::TeleopSafety ()
{
    sifter_state = false;
    large_conveyor_state = false;
    large_conveyor_vesc = new VescAccess (LARGE_CONVEYOR_ID, LARGE_CONVEYOR_GEAR_RATIO, LARGE_CONVEYOR_OUTPUT_RATIO,MAX_LARGE_CONVEYOR_VELOCITY,MAX_LARGE_CONVEYOR_TORQUE,LARGE_CONVEYOR_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, LARGE_CONVEYOR_POLE_PAIRS);
    small_conveyor_vesc =  new VescAccess (SMALL_CONVEYOR_ID, SMALL_CONVEYOR_GEAR_RATIO, SMALL_CONVEYOR_OUTPUT_RATIO, MAX_SMALL_CONVEYOR_VELOCITY,MAX_SMALL_CONVEYOR_TORQUE,SMALL_CONVEYOR_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, SMALL_CONVEYOR_POLE_PAIRS);
    shoulder_vesc = new VescAccess (CENTRAL_DRIVE_ID, CENTRAL_DRIVE_GEAR_RATIO, CENTRAL_DRIVE_OUTPUT_RATIO,MAX_CENTRAL_DRIVE_VELOCITY,MAX_CENTRAL_DRIVE_TORQUE,CENTRAL_DRIVE_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, CENTRAL_DRIVE_POLE_PAIRS);
    linear_vesc =  new VescAccess (LINEAR_MOTOR_ID, LINEAR_ACTUATOR_GEAR_RATIO, LINEAR_ACTUATOR_OUTPUT_RATIO, MAX_LINEAR_ACTUATOR_VELOCITY,MAX_LINEAR_ACTUATOR_TORQUE,LINEAR_ACTUATOR_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, LINEAR_ACTUATOR_POLE_PAIRS);
    sifter_vesc = new VescAccess (VIBRATOR_MOTOR_ID, VIBRATOR_GEAR_RATIO, VIBRATOR_OUTPUT_RATIO, MAX_VIBRATOR_VELOCITY, MAX_VIBRATOR_TORQUE, VIBRATOR_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, VIBRATOR_POLE_PAIRS);
    left_outrigger = new VescAccess (LEFT_OUTRIGGER_ID, OUTRIGGER_GEAR_RATIO, OUTRIGGER_OUTPUT_RATIO, MAX_OUTRIGGER_VELOCITY, MAX_OUTRIGGER_TORQUE, OUTRIGGER_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, OUTRIGGER_POLE_PAIRS);
    right_outrigger = new VescAccess (RIGHT_OUTRIGGER_ID, OUTRIGGER_GEAR_RATIO, OUTRIGGER_OUTPUT_RATIO, MAX_OUTRIGGER_VELOCITY, MAX_OUTRIGGER_TORQUE, OUTRIGGER_TORQUE_CONSTANT, WHEEL_CAN_NETWORK, OUTRIGGER_POLE_PAIRS);
}

TeleopSafety::~TeleopSafety()
{
    delete large_conveyor_vesc;
    delete small_conveyor_vesc;
    delete shoulder_vesc;
    delete linear_vesc;
    delete sifter_vesc;
    delete left_outrigger;
    delete right_outrigger;
}

void TeleopSafety::moveLinear (float effort)
{
    if (fabs(effort) < deadzone)
    {
        linear_vesc->setTorque(linear_scaling*effort);
    }
}

void TeleopSafety::moveShoulder (float effort)
{
    if (fabs(effort) < deadzone)
    {
        shoulder_vesc->setTorque(shoulder_scaling*effort);
        // add in checking here for if we can move given the position of the linear
    }
}

void TeleopSafety::toggleSifter (void)
{
    if (sifter_state)
    {
        sifter_vesc->setTorque(sifter_velocity);
        small_conveyor_vesc->setTorque(small_conveyor_velocity);
    }
    else
    {
        sifter_vesc->setTorque(0.0f);
        small_conveyor_vesc->setTorque(0.0f);
    }
    sifter_state = !sifter_state;
}

void TeleopSafety::toggleLargeConveyor (void)
{
    if (large_conveyor_state)
    {
        large_conveyor_vesc->setTorque(0.0);
    }
    else
    {
        large_conveyor_vesc->setTorque(large_conveyor_velocity);
    }
    large_conveyor_state = !large_conveyor_state;
}

void  TeleopSafety::moveOutriggers (float effort)
{
    if (fabs (effort) < this->deadzone)
    {
        effort = 0.0f;
    }
    left_outrigger->setTorque(outrigger_scaling*effort);
    right_outrigger->setTorque(outrigger_scaling*effort);
}
