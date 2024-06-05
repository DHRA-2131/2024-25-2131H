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
/**
 * @brief Allows for degrees and radians to co-exist :D
 *
 */
class Angle
{
 private:
  double value;
  bool isDegrees;

 public:
  /**
   * @brief Construct a empty Angle object
   *
   */
  Angle();
  /**
   * @brief Construct a new Angle object
   *
   * @param Value Value of Angle
   * @param IsDegrees Is the Angle Unit Degrees? False if Radians.
   */
  Angle(double Value, bool IsDegrees);

  /**
   * @brief Get the Angle in Radians
   *
   * @return double Radians
   */
  double getRadians();

  /**
   * @brief Get the Angle in Degrees
   *
   * @return double Degrees
   */
  double getDegrees();

  /**
   * @brief Set the new Theta of an Angle
   *
   * @param newTheta New value of Angle.
   * @param isDegrees Is newTheta in Degrees; False if Radians.
   */
  void setTheta(double newTheta, bool isDegrees);

  /**
   * @brief Addition of Two Angles
   *
   * @param B Added Angle
   * @return Angle Sum
   */
  Angle operator+(Angle B);

  /**
   * @brief Subtraction of Two Angles
   *
   * @param B Subtracting Angle
   * @return Angle Difference
   */
  Angle operator-(Angle B);

  /**
   * @brief Scale Angle by an amount
   *
   * @param B Scalar
   * @return Angle Scaled_Angle
   */
  Angle operator*(double B);

  /**
   * @brief Scale Angle by amount
   *
   * @param B Scalar
   * @return Angle Scaled_Angle
   */
  Angle operator/(double B);

  /**
   * @brief Add to Angle
   *
   * @param B Added Angle
   */
  void operator+=(Angle B);

  /**
   * @brief Subtract from Angle
   *
   * @param B Subtracting Angle
   */
  void operator-=(Angle B);

  /**
   * @brief Scale Angle
   *
   * @param B Scalar
   */
  void operator*=(double B);

  /**
   * @brief Scale Angle
   *
   * @param B
   */
  void operator/=(double B);

  /**
   * @brief "Print" Angle to OStream
   *
   * @param os Output Stream
   * @param B Angle
   * @return std::ostream&
   */
  friend std::ostream &operator<<(std::ostream &os, const Angle &B);
};
}  // namespace lib2131