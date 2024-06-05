/**
 * @file utilities.hpp
 * @author Andrew Hilton (2131H)
 * @brief Source code for lib2131's Utility Functions.
 * @version 0.1
 * @date 2024-05-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lib2131/Angle.hpp"

namespace lib2131
{

/**
 * @brief Construct a empty Angle object
 *
 */
Angle::Angle() : value(0), isDegrees(1) {}
/**
 * @brief Construct a new Angle object
 *
 * @param Value Value of Angle
 * @param IsDegrees Is the Angle Unit Degrees? False if Radians.
 */
Angle::Angle(double Value, bool IsDegrees) : value(Value), isDegrees(IsDegrees) {}

/**
 * @brief Get the Angle in Radians
 *
 * @return double Radians
 */
double Angle::getRadians()
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
 * @brief Get the Angle in Degrees
 *
 * @return double Degrees
 */
double Angle::getDegrees()
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
void Angle::setTheta(double newTheta, bool isDegrees)
{
  this->value = newTheta;
  this->isDegrees = isDegrees;
}

/**
 * @brief Addition of Two Angles
 *
 * @param B Added Angle
 * @return Angle Sum
 */
Angle Angle::operator+(Angle B)
{
  if (this->isDegrees)
  {
    return Angle(this->value + B.getDegrees(), true);
  }
  else
  {
    return Angle(this->value + B.getRadians(), false);
  }
}

/**
 * @brief Subtraction of Two Angles
 *
 * @param B Subtracting Angle
 * @return Angle Difference
 */
Angle Angle::operator-(Angle B)
{
  if (this->isDegrees)
  {
    return Angle(this->value - B.getDegrees(), true);
  }
  else
  {
    return Angle(this->value - B.getRadians(), false);
  }
}

/**
 * @brief Scale Angle by an amount
 *
 * @param B Scalar
 * @return Angle Scaled_Angle
 */
Angle Angle::operator*(double B)
{
  if (this->isDegrees)
  {
    return Angle(this->value * B, true);
  }
  else
  {
    return Angle(this->value * B, false);
  }
}

/**
 * @brief Scale Angle by amount
 *
 * @param B Scalar
 * @return Angle Scaled_Angle
 */
Angle Angle::operator/(double B)
{
  if (this->isDegrees)
  {
    return Angle(this->value / B, true);
  }
  else
  {
    return Angle(this->value / B, false);
  }
}

/**
 * @brief Add to Angle
 *
 * @param B Added Angle
 */
void Angle::operator+=(Angle B)
{
  if (this->isDegrees)
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
void Angle::operator-=(Angle B)
{
  if (this->isDegrees)
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
void Angle::operator*=(double B)
{
  if (this->isDegrees)
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
void Angle::operator/=(double B)
{
  if (this->isDegrees)
  {
    this->value /= B;
  }
  else
  {
    this->value /= B;
  }
}

std::ostream &operator<<(std::ostream &os, const Angle &B)
{
  if (B.isDegrees)
  {
    os << B.value << "°";
  }
  else
  {
    os << B.value << "rad";
  }
  return os;
}

}  // namespace lib2131