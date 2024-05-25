/**
 * @file utilities.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for different Utilities. Features include: average and Angle.
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <cmath>
#include <vector>

/**
 * @brief Averages all values in a Vector
 *
 * @tparam typename T
 * @param B Vector
 * @return T Average
 */
template <typename T>
T average(std::vector<T> B)
{
  T Total(0);  // Total
  for (int i = 0; i < B.size(); i++)
  {
    Total += B[i];  // Add each Element
  }
  return Total / B.size();  // Divide by total Elements
}

/**
 * @brief Allows for degrees and radians to co-exist :D
 *
 */
class angle
{
 private:
  double value;
  bool isDegrees;

 public:
  /**
   * @brief Construct a empty angle object
   *
   */
  angle() : value(0), isDegrees(1) {}
  /**
   * @brief Construct a new angle object
   *
   * @param Value Value of Angle
   * @param IsDegrees Is the Angle Unit Degrees? False if Radians.
   */
  angle(double Value, bool IsDegrees) : value(Value), isDegrees(IsDegrees) {}

  /**
   * @brief Get the angle in Radians
   *
   * @return double Radians
   */
  double getRadians()
  {
    if (this->isDegrees)
    {
      return this->value * M_PI / 180;
    }
    else
    {
      return this->value;
    }
  }

  /**
   * @brief Get the angle in Degrees
   *
   * @return double Degrees
   */
  double getDegrees()
  {
    if (this->isDegrees)
    {
      return this->value;
    }
    else
    {
      return this->value * 180 / M_PI;
    }
  }

  /**
   * @brief Set the new Theta of an Angle
   *
   * @param newTheta New value of Angle.
   * @param isDegrees Is newTheta in Degrees; False if Radians.
   */
  void setTheta(double newTheta, bool isDegrees)
  {
    this->value = newTheta;
    this->isDegrees = isDegrees;
  }

  /**
   * @brief Addition of Two Angles
   *
   * @param B Added Angle
   * @return angle Sum
   */
  angle operator+(angle B)
  {
    if (isDegrees)
    {
      return angle(this->value + B.getDegrees(), true);
    }
    else
    {
      return angle(this->value + B.getRadians(), false);
    }
  }

  /**
   * @brief Subtraction of Two Angles
   *
   * @param B Subtracting Angle
   * @return angle Difference
   */
  angle operator-(angle B)
  {
    if (isDegrees)
    {
      return angle(this->value - B.getDegrees(), true);
    }
    else
    {
      return angle(this->value - B.getRadians(), false);
    }
  }

  /**
   * @brief Scale angle by an amount
   *
   * @param B Scalar
   * @return angle Scaled_Angle
   */
  angle operator*(double B)
  {
    if (isDegrees)
    {
      return angle(this->value * B, true);
    }
    else
    {
      return angle(this->value * B, false);
    }
  }

  /**
   * @brief Scale angle by amount
   *
   * @param B Scalar
   * @return angle Scaled_Angle
   */
  angle operator/(double B)
  {
    if (isDegrees)
    {
      return angle(this->value / B, true);
    }
    else
    {
      return angle(this->value / B, false);
    }
  }

  /**
   * @brief Add to Angle
   *
   * @param B Added Angle
   */
  void operator+=(angle B)
  {
    if (isDegrees)
    {
      this->value += B.getDegrees();
    }
    else
    {
      this->value += B.getRadians();
    }
  }

  /**
   * @brief Subtract from Angle
   *
   * @param B Subtracting Angle
   */
  void operator-=(angle B)
  {
    if (isDegrees)
    {
      this->value -= B.getDegrees();
    }
    else
    {
      this->value -= B.getRadians();
    }
  }

  /**
   * @brief Scale Angle
   *
   * @param B Scalar
   */
  void operator*=(double B)
  {
    if (isDegrees)
    {
      this->value *= B;
    }
    else
    {
      this->value *= B;
    }
  }

  /**
   * @brief Scale Angle
   *
   * @param B
   */
  void operator/=(double B)
  {
    if (isDegrees)
    {
      this->value /= B;
    }
    else
    {
      this->value /= B;
    }
  }
};
