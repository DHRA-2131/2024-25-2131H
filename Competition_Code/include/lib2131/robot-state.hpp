/**
 * @file robot-state.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for robotState class that defines the robots: Position, Speed, and
 * Acceleration
 * @version 0.2
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "lib2131/angle.hpp"
#include "lib2131/vector3.hpp"

namespace lib2131
{

/**
 * @brief robotState Class. Defines Robot's Speed, Velocity, and Acceleration.
 *
 */
class RobotState
{
 private:  // Variables
  // {x, y, θ}
  Vector3<double, double, Angle> m_position;
  // {vx, vy, w}
  Vector3<double, double, Angle> m_velocity;
  // {ax, ax, α}
  Vector3<double, double, Angle> m_acceleration;

 public:  // Constructors
  /**
   * @brief Construct a empty robot State object
   *
   */
  RobotState();

  /**
   * @brief Construct a new robot State object
   *
   * @param Position {x, y, θ} as Vector3 Object
   * @param Velocity {vx, vy, w} as Vector3 Object
   * @param Acceleration {ax, ay, α} as Vector3 Object
   */
  RobotState(Vector3<double, double, Angle> Position,
             Vector3<double, double, Angle> Velocity,
             Vector3<double, double, Angle> Acceleration);

 public:  // Getters & Setters
  /**
   * @brief Get the Position of RobotState
   *
   * @return Vector3<double, double, Angle> Position
   */
  Vector3<double, double, Angle>* getPosition();

  /**
   * @brief Get the Velocity at RobotState
   *
   * @return Vector3<double, double, Angle> Velocity
   */
  Vector3<double, double, Angle>* getVelocity();

  /**
   * @brief Get the Acceleration at RobotState
   *
   * @return Vector3<double, double, Angle> Acceleration
   */
  Vector3<double, double, Angle>* getAcceleration();

  /**
   * @brief Set the Position of RobotState
   *
   * @param newPosition
   */
  void setPosition(Vector3<double, double, Angle> newPosition);

  /**
   * @brief Set the Velocity of RobotState
   *
   * @param newVelocity
   */
  void setVelocity(Vector3<double, double, Angle> newVelocity);

  /**
   * @brief Set the Acceleration of RobotState
   *
   * @param newAcceleration
   */
  void setAcceleration(Vector3<double, double, Angle> newAcceleration);

 public:  // Math Operators
  /**
   * @brief Addition Operator
   *
   * @param B Added RobotState
   * @return RobotState Sum of RobotStates
   */
  RobotState operator+(RobotState B);

  /**
   * @brief Subtraction Operator
   *
   * @param B Subtracting RobotState
   * @return RobotState Difference of RobotStates
   */
  RobotState operator-(RobotState B);

  /**
   * @brief Multiplication Operator
   *
   * @param B Scalar
   * @return RobotState Scaled RobotState
   */
  RobotState operator*(double B);

  /**
   * @brief Division Operator
   *
   * @param B Scalar
   * @return RobotState Scaled RobotState
   */
  RobotState operator/(double B);

 public:  // Compound Operators
  /**
   * @brief Addition Assignment
   * @note Adds to Parent RobotState
   * @param B Added RobotState
   */
  void operator+=(RobotState B);

  /**
   * @brief Subtraction Assignment
   * @note Subtracts from Parent RobotState
   * @param B Subtracting RobotState
   */
  void operator-=(RobotState B);

  /**
   * @brief Multiplication Assignment
   * @note Scales Parent RobotState
   * @param B Scalar RobotState
   */
  void operator*=(double B);

  /**
   * @brief Division Assignment
   * @note Scales Parent RobotState
   * @param B Scalar RobotState
   */
  void operator/=(double B);

  /**
   * @brief Print RobotState to Ostream
   *
   * @param os Output Stream
   * @param B RobotState
   * @return std::ostream& os
   */
  friend std::ostream& operator<<(std::ostream& os, const RobotState& B);
};
}  // namespace lib2131