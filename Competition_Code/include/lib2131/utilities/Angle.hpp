#pragma once

#include <cmath>
#include <iostream>

namespace lib2131::utilities
{
class Angle
{
 private:
  double m_value;    // Value of Angle
  bool m_isDegrees;  // Is Value using Degrees?

 public:
  Angle() {}

  Angle(double value, bool isDegrees) : m_value(value), m_isDegrees(isDegrees) {}

  double getRadians()
  {
    if (m_isDegrees) { return m_value * M_PI / 180; }
    else { return m_value; };
  }

  double getDegrees()
  {
    if (m_isDegrees) { return m_value; }
    else { return m_value * 180 / M_PI; }
  }

  double getRawValue() { return m_value; }

  Angle operator+(Angle B)
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

  Angle operator-(Angle B)
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

  Angle operator*(double B) { return Angle(m_value * B, m_isDegrees); }

  Angle operator/(double B) { return Angle(m_value / B, m_isDegrees); }

  void operator+=(Angle B)
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

  void operator-=(Angle B)
  {
    if ((this->m_isDegrees && B.m_isDegrees) || (!this->m_isDegrees && !B.m_isDegrees))
    {
      this->m_value -= B.m_value;
    }
    else if (this->m_isDegrees) { this->m_value -= B.getDegrees(); }
    else { this->m_value -= B.getRadians(); }
  }

  void operator*=(double B) { this->m_value *= B; }

  void operator/=(double B) { this->m_value /= B; }

  friend std::ostream& operator<<(std::ostream& os, const Angle& B)
  {
    os << B.m_value;
    B.m_isDegrees ? os << "°" : os << "rad";
    return os;
  }
};
}  // namespace lib2131::utilities