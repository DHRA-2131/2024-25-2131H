/**
 * @file angle.cpp
 * @author Andrew Hilton (2131H)
 * @brief Angle Class Src File
 * @version 0.2
 * @date 2024-06-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "lib2131/angle.hpp"

namespace lib2131
{
/**
 * @brief Construct a empty Angle object
 * @note Is defaulted to Radians with value of 0
 */
Angle::Angle() : m_isDegrees(0) {}

/**
 * @brief Construct a new Angle object
 * @note isDegrees Defaults to true
 * @param value Value of Angle
 * @param isDegrees Is Angle in Degrees?
 */
Angle::Angle(double value, bool isDegrees) : m_value(value), m_isDegrees(isDegrees) {}

/**
 * @brief Get the [Radian] Measure of Angle
 *
 * @return double Radians
 */
double Angle::getRadians()
{
  if (m_isDegrees) { return m_value * M_PI / 180; }
  else { return m_value; };
}

/**
 * @brief Get the [Degree] Measure of Angle
 *
 * @return double Degrees
 */
double Angle::getDegrees()
{
  if (m_isDegrees) { return m_value; }
  else { return m_value * 180 / M_PI; }
}

/**
 * @brief Get the [Raw] Measure of Angle
 *
 * @return double m_value
 */
double Angle::getRawValue() { return m_value; }

/**
 * @brief Addition Operator
 *
 * @param B Added Angle
 * @return Angle Sum of Angles
 */
Angle Angle::operator+(Angle B)
{
  // If Angles have the same Unit
  if ((this->m_isDegrees && B.m_isDegrees) || (!this->m_isDegrees && !B.m_isDegrees))
  {
    return Angle(this->m_value + B.m_value, m_isDegrees);
  }
  // Else Convert Angle B to match unit
  else if (this->m_isDegrees) { return Angle(this->m_value + B.getDegrees(), true); }
  else { return Angle(this->m_value + B.getRadians(), false); }
}
/**
 * @brief Subtraction Operator
 *
 * @param B Subtracting Angle
 * @return Angle Difference of Angles
 */
Angle Angle::operator-(Angle B)
{
  // If Angles have the same Unit
  if ((this->m_isDegrees && B.m_isDegrees) || (!this->m_isDegrees && !B.m_isDegrees))
  {
    return Angle(this->m_value - B.m_value, m_isDegrees);
  }
  // Else Convert Angle B to match unit
  else if (this->m_isDegrees) { return Angle(this->m_value - B.getDegrees(), true); }
  else { return Angle(this->m_value - B.getRadians(), false); }
}
/**
 * @brief Multiplication Operator
 *
 * @param B Scalar
 * @return Angle Scaled Angle
 */
Angle Angle::operator*(double B) { return Angle(m_value * B, m_isDegrees); }

/**
 * @brief Division Operator
 *
 * @param B Scalar
 * @return Angle Scaled Angle
 */
Angle Angle::operator/(double B) { return Angle(m_value / B, m_isDegrees); }

/**
 * @brief Addition Assignment
 * @note Adds to Parent Angle
 * @param B Added Angle
 */
void Angle::operator+=(Angle B)
{
  // If Angles have the same Unit
  if ((this->m_isDegrees && B.m_isDegrees) || (!this->m_isDegrees && !B.m_isDegrees))
  {
    this->m_value += B.m_value;
  }
  // Else convert Angle B to match unit
  else if (this->m_isDegrees) { this->m_value += B.getDegrees(); }
  else { this->m_value += B.getRadians(); }
}

/**
 * @brief Subtraction Assignment
 * @note Subtracts from Parent Angle
 * @param B Subtracting Angle
 */
void Angle::operator-=(Angle B)
{
  if ((this->m_isDegrees && B.m_isDegrees) || (!this->m_isDegrees && !B.m_isDegrees))
  {
    this->m_value -= B.m_value;
  }
  else if (this->m_isDegrees) { this->m_value -= B.getDegrees(); }
  else { this->m_value -= B.getRadians(); }
}

/**
 * @brief Multiplication Assignment
 * @note Scales Parent Angle
 * @param B Scalar
 */
void Angle::operator*=(double B) { this->m_value *= B; }

/**
 * @brief Division Assignment
 * @note Scales Parent Angle
 * @param B Scalar
 */
void Angle::operator/=(double B) { this->m_value /= B; }

/**
 * @brief Print an Angle to an std::Ostream
 *
 * @param os Output Stream
 * @param B Angle
 * @return std::ostream& os
 */
std::ostream& operator<<(std::ostream& os, const Angle& B)
{
  os << B.m_value;
  B.m_isDegrees ? os << "°" : os << "rad";
  return os;
}
};  // namespace lib2131