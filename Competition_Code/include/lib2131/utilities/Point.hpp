#pragma once
#include <cmath>

#include "lib2131/utilities/Units.h"

namespace lib2131::utilities
{

struct Point
{
  using angle_t = units::angle::radian_t;
  using distance_t = units::length::inch_t;

  distance_t x, y;

  distance_t magnitude(const Point& B)
  {
    return units::math::sqrt(units::math::pow<2>(B.x - this->x) +
                             units::math::pow<2>(B.y - this->y));
  }

  angle_t amplitude(const Point& B)
  {
    return angle_t(units::math::atan2(B.y - this->y, B.x - this->x));
  }

  void rotate(angle_t& B)
  {
    // Calculate rotated point
    distance_t newX = y * units::math::sin(B) + x * units::math::cos(B);
    distance_t newY = y * units::math::cos(B) + x * -units::math::sin(B);

    // Set new Point
    x = newX;
    y = newY;
  }

  Point operator+(Point B) { return {this->x + B.x, this->y + B.y}; }
  Point operator-(Point B) { return {this->x - B.x, this->y - B.y}; }
  Point operator*(double B) { return {this->x * B, this->y * B}; }
  Point operator/(double B) { return {this->x / B, this->y / B}; }

  Point& operator+=(const Point& B)
  {
    this->x += B.x;
    this->y += B.y;
    return *this;
  }
  Point& operator-=(const Point& B)
  {
    this->x -= B.x;
    this->y -= B.y;
    return *this;
  }
  Point& operator*=(const double B)
  {
    this->x *= B;
    this->y *= B;
    return *this;
  }
  Point& operator/=(const double B)
  {
    this->x /= B;
    this->y /= B;
    return *this;
  }
};

}  // namespace lib2131::utilities