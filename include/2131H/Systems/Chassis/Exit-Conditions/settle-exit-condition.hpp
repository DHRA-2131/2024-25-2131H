/**
 * @file settle-exit-condition.hpp
 * @author Andrew Hilton (2131H)
 * @brief Settle exit condition declaration
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "abstract-exit-condition.hpp"

namespace Systems
{
namespace ExitCondition
{
class Settle : public AbstractExitCondition
{
 private:              // === Member Variables === //
  int m_settleTime;    // Total Time Settling
  int m_targetTime;    // Target Time in Settle before exit
  double m_tolerance;  // Bounds that is considered Settling (as an error)

 public:  // === Constructors === //
  /**
   * @brief Make a settle exit conditionals
   *
   * @param targetTime Time within tolerance to allow exit (in msec)
   * @param tolerance Tolerance between target and current (Abstract unit - Error)
   */
  Settle(int targetTime, double tolerance)
      : m_settleTime(0), m_targetTime(targetTime), m_tolerance(tolerance)
  {
  }

 public:  // === Functions === //
  /**
   * @brief Has the Exit Condition been met?
   *
   * @param error Error in state
   * @param deltaTime Change in time since last checked
   * @return true Yes! Exit Conditions have been met.
   * @return false No. Exit Conditions have yet to be satisfied
   */
  bool canExit(double error, int deltaTime) override
  {
    // If within settle tolerance, then add to settle time
    if (error < m_tolerance) { m_settleTime += deltaTime; }
    else { m_settleTime = 0; }  // Else, reset settle time to 0 (went out of settle tolerance)

    // Return if the settle time is greater than target time (Can exit = true)
    return (m_settleTime > m_targetTime);
  }

  void reset() override { m_settleTime = 0; }
};
}  // namespace ExitCondition
}  // namespace Systems