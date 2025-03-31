/**
 * @file ArmPID.cpp
 * @author Andrew Hilton (2131H)
 * @brief A PID built on the normal lemlib PID class that adds gravity  
 * @version 0.1
 * @date 2025-03-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "2131H/Utilities/ArmPID.hpp"
#include "cmath"

namespace Utilities
{
ArmPID::ArmPID(float kP, float kI, float kD, float kG, float windupRange, bool signFlipReset)
    : kP(kP),
        kI(kI),
        kD(kD),
        kG(kG),
        windupRange(windupRange),
        signFlipReset(signFlipReset) {}

float ArmPID::update(const float error, const float theta) {
    // calculate integral
    integral += error;
    if ((0 > error != 0 > prevError) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD + kG * 9.81 * sin(theta);
}

void ArmPID::reset() {
    integral = 0;
    prevError = 0;
}
}