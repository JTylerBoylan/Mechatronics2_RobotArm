#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <math.h>
#include <stdlib.h>

#define ArmLength 1.0f
#define GripperEffectiveRadius 0.1f

#define MotorStepAngleIncrement 0.0314159f

// Motor types
enum MotorType {DirectCurrent, Stepper};

// Full step outputs
const int StepperMotorSteps[4] = {0b0001, 0b1000, 0b0010, 0b0100};

struct Motor {
    MotorType type;
    int position;
};

// Convert arm coordinates to arm angles
float * toYawPitchRoll(const float * coords);

// Convert arm angles to arm coordinates
float * toXYZ(const float * angles);


// Get orientation of DC Motor
float toAngleDirectCurrent(const int position);

// Get orientation of Stepper Motor
float toAngleStepper(const int position);

// Get motor position
float toAngle(const Motor motor);

// Get motor positions
float * toAngles(const Motor * motors);

// Get difference in angle
float * deltaAngle(const float * angles1, const float * angles2);


// Move stepper motor one full step forward
int stepForward(Motor& motor);

// Move stepper motor one full step backwards
int stepBackward(Motor& motor);


// Move to position
int * moveTo(Motor * motors, const float * angles);

// Move to coordinates
int * moveToCoordinates(Motor * motors, const float * coords);

#endif