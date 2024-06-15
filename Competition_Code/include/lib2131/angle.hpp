/**
 * @file angle.hpp
 * @author Andrew Hilton (2131H)
 * @brief Header File for Angle class
 * @version 0.1
 * @date 2024-06-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cmath>
#include <ostream>

namespace lib2131
{
class Angle
{
 private:
  double m_value;    // Value of Angle
  bool m_isDegrees;  // Is Value using Degrees?

 public:  // Constructors
  /**
   * @brief Construct a empty Angle object
   * @note Is defaulted to Radians with value of 0
   */
  Angle();

  /**
   * @brief Construct a new Angle object
   * @note isDegrees Defaults to true
   * @param value Value of Angle
   * @param isDegrees Is Angle in Degrees?
   */
  Angle(double value, bool isDegrees = true);

 public:  // Getters & Setters
  /**
   * @brief Get the [Radian] Measure of Angle
   *
   * @return double Radians
   */
  double getRadians();

  /**
   * @brief Get the [Degree] Measure of Angle
   *
   * @return double Degrees
   */
  double getDegrees();

  /**
   * @brief Get the [Raw] Measure of Angle
   *
   * @return double m_value
   */
  double getRawValue();

 public:  // Math Operations
  /**
   * @brief Addition Operator
   *
   * @param B Added Angle
   * @return Angle Sum of Angles
   */
  Angle operator+(Angle B);

  /**
   * @brief Subtraction Operator
   *
   * @param B Subtracting Angle
   * @return Angle Difference of Angles
   */
  Angle operator-(Angle B);

  /**
   * @brief Multiplication Operator
   *
   * @param B Scalar
   * @return Angle Scaled Angle
   */
  Angle operator*(double B);

  /**
   * @brief Division Operator
   *
   * @param B Scalar
   * @return Angle Scaled Angle
   */
  Angle operator/(double B);

 public:  // Compound Operator
  /**
   * @brief Addition Assignment
   * @note Adds to Parent Angle
   * @param B Added Angle
   */
  void operator+=(Angle B);

  /**
   * @brief Subtraction Assignment
   * @note Subtracts from Parent Angle
   * @param B Subtracting Angle
   */
  void operator-=(Angle B);

  /**
   * @brief Multiplication Assignment
   * @note Scales Parent Angle
   * @param B Scalar
   */
  void operator*=(double B);

  /**
   * @brief Division Assignment
   * @note Scales Parent Angle
   * @param B Scalar
   */
  void operator/=(double B);

  /**
   * @brief Print an Angle to an std::Ostream
   *
   * @param os Output Stream
   * @param B Angle
   * @return std::ostream& os
   */
  friend std::ostream& operator<<(std::ostream& os, const Angle& B);
};
}  // namespace lib2131