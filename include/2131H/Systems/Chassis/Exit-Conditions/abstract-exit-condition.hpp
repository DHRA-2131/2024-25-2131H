/**
 * @file abstract-exit-condition.hpp
 * @author Andrew Hilton (2131H)
 * @brief Abstract Class declaration for exit conditions
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

namespace Systems
{
namespace ExitCondition
{
class AbstractExitCondition
{
 public:  // === Functions === //
  /**
   * @brief Over-ridable function that returns if a controller should exit or not
   *
   * @param error Error in System
   * @param deltaTime Change in time since last called
   * @return true Yes! The controller can exit
   * @return false NO. The controller shouldn't exit
   */
  virtual bool canExit(double error, int deltaTime) = 0;

  virtual void reset() = 0;
};
}  // namespace ExitCondition
}  // namespace Systems