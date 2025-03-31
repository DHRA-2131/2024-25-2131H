/**
 * @file ArmPID.hpp
 * @author Andrew Hilton (2131H)
 * @brief ArmPID class declaration
 * @version 0.1
 * @date 2025-03-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

namespace Utilities
{
class ArmPID {
    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param kG gravity gain
         * @param windupRange integral anti windup range
         * @param signFlipReset whether to reset integral when sign of error flips
         */
        ArmPID(float kP, float kI, float kD, float kG, float windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Update the PID
         *
         * @param error target minus position - AKA error
         * @param theta Angle difference from upright position to current position
         * @return float output
         *
         */
        float update(const float error, const float theta);

        /**
         * @brief reset integral, derivative, and prevTime
         */
        void reset();
    protected:
        // gains
        const float kP;
        const float kI;
        const float kD;
        const float kG;

        // optimizations
        const float windupRange;
        const bool signFlipReset;

        float integral = 0;
        float prevError = 0;
};
}