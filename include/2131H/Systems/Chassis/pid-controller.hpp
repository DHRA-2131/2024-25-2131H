/**
 * @file pid-controller.hpp
 * @author Andrew Hilton (2131H)
 * @brief PID Controllers and Exit Conditions
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <vector>

#include "2131H/Systems/Chassis/Exit-Conditions/abstract-exit-condition.hpp"
#include "Exit-Conditions/abstract-exit-condition.hpp"

namespace Systems
{
class PID
{
 private:                   // === Member Variables  == //
  double m_error;           // (Target Position - Current Position)
  double m_prevError;       // Error of previous loop
  double m_kP, m_kI, m_kD;  // Tuning constants
  double I;                 // Integral Component

  // Exit Conditions
  std::vector<ExitCondition::AbstractExitCondition*> m_exits;

 public:  // === Constructors === //
  /**
   * @brief Construct a new PID controller
   *
   * @param kP Tuning Constant for Proportional Component
   * @param kI Tuning Constant for Integral Component
   * @param kD Tuning Constant for Derivative Component
   * @param exits Exit Conditions for PID Controller
   * @param error Starting Error of System (Defaulted to 0)
   */
  PID(double kP, double kI, double kD, std::vector<ExitCondition::AbstractExitCondition*> exits,
      double error = 0)
      : m_error(error), m_prevError(0), m_kP(kP), m_kI(kI), m_kD(kD), m_exits(std::move(exits))
  {
  }

 public:  // === Functions === //
  /**
   * @brief Calculate the output of the PID
   *
   * @param error Error of system
   * @param deltaTime Change in time since last called
   * @return double PID output
   */
  double getOutput(double error, double deltaTime)
  {
    // Set Current Error
    this->m_error = error;

    // Proportional is proportional to error. (Error * Constant)
    double P = this->m_error * this->m_kP;
    // Integral is previous errors added up. (Approximated with a Trapezoidal Riemann sum)
    I += (this->m_error + this->m_prevError) / 2 * deltaTime;
    // Derivative is the rate of change in error. (Delta Error / Delta Time)
    double D = (this->m_error - this->m_prevError) / deltaTime;

    this->m_prevError = this->m_error;  // Reset Previous Error

    return P * this->m_kP + I * this->m_kI + D * this->m_kD;  // Add Outputs
  }

  /**
   * @brief Can the PID exit a motion?
   *
   * @param deltaTime Change in time since last checked
   * @return true Yes. The exit conditions have been satisfied
   * @return false No. The exit conditions have not been reached.
   */
  bool exitConditionsMet(int deltaTime)
  {
    // Check each exit condition and if they are all true, then return true;
    return std::all_of(std::begin(m_exits), std::end(m_exits),
                       [this, &deltaTime](ExitCondition::AbstractExitCondition* i) {
                         return i->canExit(m_error, deltaTime);
                       });
  }

  /**
   * @brief Add an exit condition to the PID Controller.
   *
   * @param exit Exit Condition
   */
  void addExitCondition(ExitCondition::AbstractExitCondition* exit)
  {
    m_exits.push_back(exit);  // Add exit condition
  }

  /**
   * @brief Set the Exit Conditions for PID
   *
   * @param exits vector of exit conditions
   */
  void setExitConditions(std::vector<ExitCondition::AbstractExitCondition*> exits)
  {
    // Set Exit Conditions to new Exit Conditions
    this->m_exits = std::move(exits);
  }

  /**
   * @brief Resets PID and PIDs exit conditions
   *
   */
  void reset()
  {
    I = 0;  // Reset I Components (Other Components shouldn't need to be reset)
    for (auto ec : m_exits) { ec->reset(); }  // Reset exit conditions
  }
};
}  // namespace Systems