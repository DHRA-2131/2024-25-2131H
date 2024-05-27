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
#include "lib2131/utilities.hpp"
namespace lib2131
{

/**
 * @brief Construct a empty angle object
 *
 */
angle::angle() : value(0), isDegrees(1) {}
/**
 * @brief Construct a new angle object
 *
 * @param Value Value of Angle
 * @param IsDegrees Is the Angle Unit Degrees? False if Radians.
 */
angle::angle(double Value, bool IsDegrees) : value(Value), isDegrees(IsDegrees) {}

/**
 * @brief Get the angle in Radians
 *
 * @return double Radians
 */
double angle::getRadians()
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
double angle::getDegrees()
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
void angle::setTheta(double newTheta, bool isDegrees)
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
angle angle::operator+(angle B)
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
angle angle::operator-(angle B)
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
angle angle::operator*(double B)
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
angle angle::operator/(double B)
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
void angle::operator+=(angle B)
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
void angle::operator-=(angle B)
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
void angle::operator*=(double B)
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
void angle::operator/=(double B)
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

std::ostream &operator<<(std::ostream &os, const angle &B)
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