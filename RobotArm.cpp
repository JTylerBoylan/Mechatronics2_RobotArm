#include <RobotArm.hpp>

float * toYawPitchRoll(const float * coords) {
    float x = coords[0], y = coords[1], z = coords[2];
    float angles[3];
    angles[0] = atanf(y/x); // Yaw [-pi/2, +pi/2]
    angles[1] = acosf(sqrtf(x*x + y*y) / ArmLength); // Pitch [0, +pi]
    angles[2] = (z - ArmLength * sinf(angles[1])) / GripperEffectiveRadius; // Roll [-inf, inf]
    return angles;
}

float * toXYZ(const float * angles) {
    float y = angles[0], p = angles[1], r = angles[2];
    float coords[3];
    coords[0] = ArmLength * cosf(y) * cosf(p); // X
    coords[1] = ArmLength * sinf(y) * cosf(p); // Y
    coords[2] = ArmLength * sinf(p) + r * GripperEffectiveRadius; // Z
    return coords;
}


float toAngleDirectCurrent(const int position) {
    // TODO
}

float toAngleStepper(const int position) {
    return MotorStepAngleIncrement * position;
}

float toAngle(const Motor motor) {
    switch (motor.type) {
        case MotorType::DirectCurrent:
            return toAngleDirectCurrent(motor.position);
        case MotorType::Stepper:
            return toAngleStepper(motor.position);
    }
}

float * toAngles(const Motor * motors) {
    float angles[3] ;
    for (int i = 0; i < 3; i++)
        angles[i] = toAngle(motors[i]);
    return angles;
}

float * deltaAngle(const float * angles1, const float * angles2) {
    float delta[3];
    for (int i = 0; i < 3; i++)
        delta[i] = angles2[i] - angles1[i];
}


int stepForward(Motor& motor) {
    return StepperMotorSteps[++motor.position % 4];
}

int stepBackward(Motor& motor) {
    return StepperMotorSteps[--motor.position % 4];
}


int * moveTo(Motor * motors, const float * desired) {
    float * current = toAngles(motors);
    float * delta = deltaAngle(current, desired);
    // TODO
}

int * moveToCoordinates(Motor * motors, const float * coords) {
    float * angles = toYawPitchRoll(coords);
    return moveTo(motors, angles);
}